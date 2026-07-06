/* INCLUDE FILE DECLARATIONS */
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <linux/sockios.h>
#include <linux/types.h>
#include <pthread.h>
#include "ioctl.h"

/* LOCAL VARIABLES DECLARATIONS */

int main(int argc, char* argv[])
{

	char dev[16];
	int  select;
	int  mode;
	int  ret;
	int  check_as;

	int devfd;

	while (1) {

		printf("Please input the port of AX781x0/MCS78x0 (ex. /dev/ttyUSB0):");
		scanf("%s", dev);

		if (memcmp(dev, "/dev/ttyUSB", 11) != 0)
			printf("Wrong input !!\n");
		else
			break;
	}

	devfd = open(dev, O_RDWR);
	
	if (devfd == -1) {
		printf("Can't open %s\n", dev);
		return 0;
	}

	ret = ioctl(devfd, IOCTL_CHECK_AS, &check_as);

	if (ret < 0) {
		printf("IOCTL failed!\n");
		goto exit;
	}

	if (check_as != 1) {
		printf("\nAuto-switch is disable!\n");
		goto exit;
	}

	while (1) {

		printf("Please specift the number of the mode of %s.\n", dev);
		printf("1 : RS232\n");
		printf("2 : RS422 - No Terminating Register\n");
		printf("3 : RS422 - Terminating Register\n");
		printf("4 : RS485 - Half Duplex with Echo\n");
		printf("5 : RS485 - Half Duplex with No Echo\n");
		printf("6 : RS485 - Full Duplex\n");
		printf("7 : set 6M Baud\n");
		printf("0 : Exit\n");
		printf(":");
		scanf("%d", &select);


		if (select == 1) {
			mode = 0;
			break;
		} else if (select == 2) {	
			mode = 0x22;
			break;
		} else if (select == 3) {
			mode = 0x23;
			break;
		} else if (select == 4) {	
			mode = 0xC1;
			break;
		} else if (select == 5) {	
			mode = 0xC2;
			break;
		} else if (select == 6) {	
			mode = 0xC3;
			break;
		} else if (select == 7) {	
			if (ioctl(devfd, TIOCEXBAUD, 6000000) < 0) 
				printf("IOCTL failed!\n");	
			return 0;
		} else if (select == 0) {
			return 0;
		}
	}

	ret = ioctl(devfd, IOCTL_SET_RS_MODE, mode);

	if (ret < 0) {
		printf("IOCTL failed!\n");
	} else {
		printf("The %s will operating in selected mode after open(re-open) it.\n", dev);
	}

exit:
	printf("\n");

	return 0;
}
