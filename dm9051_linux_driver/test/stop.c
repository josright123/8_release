// gcc -o stop_all stop.c

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/ptp_clock.h>

#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>

int main(void)
{
	const char *ptp_device = "/dev/ptp0"; // 視你的裝置而定
	int fd = open(ptp_device, O_RDONLY);
	if (fd < 0) {
		perror("open");
		return EXIT_FAILURE;
	}

	struct ptp_extts_request extts;
	memset(&extts, 0, sizeof(extts));
	extts.index = 0;  // SDP pin index
	// Disable EXTSYNC
	extts.flags = 0;
	printf("Disabling external timestamping on %s (index=%d)...\n", ptp_device, extts.index);
	ioctl(fd, PTP_EXTTS_REQUEST, &extts);
	extts.index = 2;  // SDP pin index
	// Disable EXTSYNC
	extts.flags = 0;
	printf("Disabling external timestamping on %s (index=%d)...\n", ptp_device, extts.index);
	ioctl(fd, PTP_EXTTS_REQUEST, &extts);
	close(fd);

	return EXIT_SUCCESS;
}
