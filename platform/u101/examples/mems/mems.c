#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "platform-conf.h"
#include "lsm303dlh.h"
#include "stcn75.h"

PROCESS(lsm303_process, "LSM303 process");

AUTOSTART_PROCESSES(&lsm303_process);

PROCESS_THREAD(lsm303_process, ev , data)
{
	static struct etimer timer;
	static int n;
	int res;
	int i;
	uint8_t rawacc[6];
	uint8_t rawmag[6];

	PROCESS_BEGIN();

	printf("Started lsm303 process\n");

        lsm303_init(0);

	while (1) {
		/* Delay */
		etimer_set(&timer, CLOCK_SECOND / 1);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

                /* Get raw data */
		res = lsm303_get(rawacc, rawmag);
		if (0 != res) {
			printf("Problem getting\n");
			break;
		}

                /* Process */
		lsm303_cook(rawacc, lsm303acc, 4);
		lsm303_cook(rawmag, lsm303mag, 4);

                /* Display */
		if (1 || (n % 10) == 0) {
			printf("Acc:  X:%05d  Y:%05d  Z:%05d    ", 
			       lsm303acc[0], lsm303acc[1], lsm303acc[2]);
		}
		if (1 || (n % 10) == 0) {
			printf("Mag:  X:%05d  Y:%05d  Z:%05d\n", 
			       lsm303mag[0], lsm303mag[1], lsm303mag[2]);
		}
		n++;
	}	
	printf("Exiting lsm303 process\n");
	PROCESS_END();
}
