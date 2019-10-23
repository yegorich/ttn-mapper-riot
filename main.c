#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "thread.h"
#include "ringbuffer.h"
#include "periph/uart.h"
#include "stdio_uart.h"
#include "xtimer.h"
#include "minmea.h"

#define UART_BUFSIZE        (128U)
#define PRINTER_PRIO        (THREAD_PRIORITY_MAIN - 1)
#define PRINTER_TYPE        (0xabcd)
typedef struct {
    char rx_mem[UART_BUFSIZE];
    ringbuffer_t rx_buf;
} uart_ctx_t;

static uart_ctx_t ctx[UART_NUMOF];

static kernel_pid_t printer_pid;
static char printer_stack[THREAD_STACKSIZE_MAIN];

static void *printer(void *arg)
{
    (void)arg;
    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);
    char buf[UART_BUFSIZE];

    while (1) {
        msg_receive(&msg);
        uart_t dev = (uart_t)msg.content.value;

	ringbuffer_get(&(ctx[dev].rx_buf), buf, ctx[dev].rx_buf.avail);
	//printf("buf: %s\n", buf);
	switch (minmea_sentence_id(buf, false)) {
		case MINMEA_SENTENCE_RMC: {
		    struct minmea_sentence_rmc frame;
		    if (minmea_parse_rmc(&frame, buf)) {
			printf("$RMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
				frame.latitude.value, frame.latitude.scale,
				frame.longitude.value, frame.longitude.scale,
				frame.speed.value, frame.speed.scale);
			printf("$RMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
				minmea_rescale(&frame.latitude, 1000),
				minmea_rescale(&frame.longitude, 1000),
				minmea_rescale(&frame.speed, 1000));
			printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\n",
				minmea_tocoord(&frame.latitude),
				minmea_tocoord(&frame.longitude),
				minmea_tofloat(&frame.speed));
		    }
		} break;

		case MINMEA_SENTENCE_GGA: {
		    struct minmea_sentence_gga frame;
		    if (minmea_parse_gga(&frame, buf)) {
			printf("$GGA: fix quality: %d\n", frame.fix_quality);
		    }
		} break;

		case MINMEA_SENTENCE_GSV: {
		    struct minmea_sentence_gsv frame;
		    if (minmea_parse_gsv(&frame, buf)) {
			printf("$GSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
			printf("$GSV: sattelites in view: %d\n", frame.total_sats);
			for (int i = 0; i < 4; i++)
			    printf("$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
				frame.sats[i].nr,
				frame.sats[i].elevation,
				frame.sats[i].azimuth,
				frame.sats[i].snr);
		    }
		} break;
		case MINMEA_INVALID:
		case MINMEA_SENTENCE_GSA:
		case MINMEA_UNKNOWN:
		case MINMEA_SENTENCE_GLL:
		case MINMEA_SENTENCE_GST:
		case MINMEA_SENTENCE_VTG:
		case MINMEA_SENTENCE_ZDA:
			printf("Unknown command\n");
			break;
	    }
    }

    /* this should never be reached */
    return NULL;
}

static void rx_cb(void *arg, uint8_t data)
{
    uart_t dev = (uart_t)arg;

    ringbuffer_add_one(&(ctx[dev].rx_buf), data);
    if (data == '\n') {
        msg_t msg;
        msg.content.value = (uint32_t)dev;
        msg_send(&msg, printer_pid);
    }
}

int main(void)
{
    int dev = 1;
    puts("TTN Mapper");

    /* initialize ringbuffers */
    for (unsigned i = 0; i < UART_NUMOF; i++) {
        ringbuffer_init(&(ctx[i].rx_buf), ctx[i].rx_mem, UART_BUFSIZE);
    }

    /* start the printer thread */
    printer_pid = thread_create(printer_stack, sizeof(printer_stack),
                                PRINTER_PRIO, 0, printer, NULL, "printer");
    if (uart_init(UART_DEV(dev), 9600, rx_cb, (void *)dev) != 0) {
        puts("UART init failed\n");
        return 1;
    }

    while(1) {
        xtimer_sleep(1);
    }

    return 0;
}
