#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "platform-conf.h"
#include "lsm303dlh.h"
#include "l3g4200d.h"
#include "stcn75.h"
#include "dev/leds.h"
#include "rtimer.h"

PROCESS(mems_process, "MEMS process");

AUTOSTART_PROCESSES(&mems_process);

PROCESS_THREAD(mems_process, ev , data)
{
	static struct etimer timer;
	static int n;
	int res;
	uint8_t rawacc[6];
	uint8_t rawmag[6];
	uint8_t rawomega[6];
        int lsm303acc[3];
        int lsm303mag[3];
        int l3gomega[3];

	PROCESS_BEGIN();

	printf("Started mems process\n");

        lsm303_init(0);
        l3g4200d_init();

	while (1) {

		/* Delay */
		etimer_set(&timer, CLOCK_SECOND / 1);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

                /* Get raw data from acc and mag */
		res = lsm303_get(rawacc, rawmag);
		if (0 != res) {
                        printf("Coudln't read from LSM303 - exiting");
			break;
		}

                /* Get raw data */
		res = l3g4200d_get(rawomega);
		if (0 != res) {
                        printf("Coudln't read from L3Q4200D - exiting");
			break;
		}

                /* Process */
		lsm303_cook(rawacc, lsm303acc, 4);
		lsm303_cook(rawmag, lsm303mag, 4);
                l3g4200d_cook(rawomega, l3gomega);

                /* Display */
		if (1 || (n % 10) == 0) {
			printf("A: (%05d, %05d, %05d), ", 
			       lsm303acc[0], lsm303acc[1], lsm303acc[2]);
		}
		if (1 || (n % 10) == 0) {
			printf("M: (%05d, %05d, %05d), ",  
			       lsm303mag[0], lsm303mag[1], lsm303mag[2]);
		}
		if (1 || (n % 10) == 0) {
			printf("G: (%05d, %05d, %05d), ",  
			       l3gomega[0], l3gomega[1], l3gomega[2]);
		}
		n++;
	}	
	printf("Exiting lsm303 process\n");
	PROCESS_END();
}
