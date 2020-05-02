#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "protocol.h"
//#include "datalink.h"

/* FRAME kind */
#define FRAME_DATA (1 << 6)
#define FRAME_ACK (2 << 6)
#define FRAME_NAK (3 << 6)

/*  
    DATA Frame
    +=============+============+================+=============+
    | KIND(2bits) | SEQ(6bits) | DATA(256Bytes) | CRC(4Bytes) |
    +=============+============+================+=============+

    ACK Frame
    +=============+============+=============+
    | KIND(2bits) | ACK(6bits) | CRC(4Bytes) |
    +=============+============+=============+

    NAK Frame
    +=============+============+=============+
    | KIND(2bits) | ACK(6bits) | CRC(4Bytes) |
    +=============+============+=============+
*/

#define DATA_TIMER 2000

#define MAX_SEQ ((1 << 6) - 1)
#define NR_BUFS ((MAX_SEQ + 1) / 2)

struct FRAME
{
    unsigned char control_code;
    unsigned char data[PKT_LEN];
    unsigned int padding;
};

static unsigned char next_frame_to_send = 0;
static unsigned char ack_expected = 0;
static unsigned char frame_expected = 0;
static unsigned char too_far = NR_BUFS;
static unsigned char in_buf[NR_BUFS][PKT_LEN];
static unsigned char out_buf[NR_BUFS][PKT_LEN];
static bool arrived[NR_BUFS] = {false};
static unsigned char nbuffered = 0;
static bool no_nak = true;
static bool phl_ready = false;

int inc(int k)
{
    k = (k + 1) % (MAX_SEQ + 1);
    return k;
}

int dec(int k)
{
    k = (k + MAX_SEQ) % (MAX_SEQ + 1);
    return k;
}

bool between(int a, int b, int c)
{
    return (a <= b && b < c) || (c < a && a <= b) || (b < c && c < a);
}

static void put_frame(unsigned char *frame, int len)
{
    *(unsigned int *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = false;
}

static void send_data_frame(unsigned char frame_nr)
{
    struct FRAME s;

    s.control_code = FRAME_DATA + frame_nr;
    memcpy(s.data, out_buf[frame_nr % NR_BUFS], PKT_LEN);

    dbg_frame("Send DATA %d, ID %d\n", frame_nr, *(short *)s.data);

    put_frame((unsigned char *)&s, 1 + PKT_LEN);
    start_timer(frame_nr, DATA_TIMER);
}

static void send_ack_frame(void)
{
    struct FRAME s;

    s.control_code = FRAME_ACK + dec(frame_expected);

    dbg_frame("Send ACK  %d\n", dec(frame_expected));

    put_frame((unsigned char *)&s, 1);
}

static void send_nak_frame(void)
{
    no_nak = false;

    struct FRAME s;

    s.control_code = FRAME_NAK + dec(frame_expected);

    dbg_frame("Send NAK  %d\n", dec(frame_expected));

    put_frame((unsigned char *)&s, 1);
}

int main(int argc, char **argv)
{
    int event, arg;
    struct FRAME f;
    int len = 0;
    bool ack_flag;

    unsigned char kind, seq, ack;

    protocol_init(argc, argv);
    lprintf("Designed by Jiang Yanjun, modified by YT, build: " __DATE__ "  "__TIME__
            "\n");

    disable_network_layer();

    while (true)
    {
        event = wait_for_event(&arg);
        switch (event)
        {
        case NETWORK_LAYER_READY:
            get_packet(out_buf[next_frame_to_send % NR_BUFS]);
            nbuffered++;
            send_data_frame(next_frame_to_send);
            next_frame_to_send = inc(next_frame_to_send);
            break;

        case PHYSICAL_LAYER_READY:
            phl_ready = true;
            break;

        case FRAME_RECEIVED:
            len = recv_frame((unsigned char *)&f, sizeof f);
            kind = f.control_code & 0xc0;
            seq = ack = f.control_code & MAX_SEQ;
            if (len < 5 || crc32((unsigned char *)&f, len) != 0)
            {
                dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                if (no_nak)
                    send_nak_frame();
                break;
            }
            if (kind == FRAME_ACK)
            {
                seq = 0xff;
                dbg_frame("Recv ACK  %d\n", ack);
            }
            if (kind == FRAME_DATA)
            {
                ack = 0xff;
                if (seq != frame_expected && no_nak)
                    send_nak_frame();
                if (between(frame_expected, seq, too_far) && !arrived[seq % NR_BUFS])
                {
                    dbg_frame("Recv DATA %d, ID %d\n", seq, *(short *)f.data);
                    arrived[seq % NR_BUFS] = true;
                    memcpy(in_buf[seq % NR_BUFS], f.data, PKT_LEN);
                    ack_flag = false;
                    while (arrived[frame_expected % NR_BUFS])
                    {
                        put_packet(in_buf[frame_expected % NR_BUFS], len - 5);
                        no_nak = true;
                        arrived[frame_expected % NR_BUFS] = false;
                        frame_expected = inc(frame_expected);
                        too_far = inc(too_far);
                        ack_flag = true;
                    }
                    if (ack_flag)
                        send_ack_frame();
                }
            }
            if (kind == FRAME_NAK)
            {
                seq = 0xff;
                dbg_frame("Recv NAK  %d\n", ack);
                if (between(ack_expected, inc(ack), next_frame_to_send))
                {
                    send_data_frame(inc(ack));
                }
            }
            if (ack != 0xff)
                while (between(ack_expected, ack, next_frame_to_send))
                {
                    nbuffered--;
                    stop_timer(ack_expected % NR_BUFS);
                    ack_expected = inc(ack_expected);
                }
            break;

        case DATA_TIMEOUT:
            dbg_event("---- DATA %d timeout\n", arg);
            if (between(ack_expected, arg, next_frame_to_send))
                send_data_frame(arg);
            break;
        }

        if (nbuffered < NR_BUFS && phl_ready)
            enable_network_layer();
        else
            disable_network_layer();
    }
}
