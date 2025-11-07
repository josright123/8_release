// gcc -o extts_test extts.c

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
	struct ptp_extts_event event;
	int n_events = 10; // 設定要讀取的事件數量
	const char *ptp_device = "/dev/ptp0"; // 視你的裝置而定
	int fd = open(ptp_device, O_RDONLY);
	if (fd < 0) {
		perror("open");
		return EXIT_FAILURE;
	}

	struct ptp_extts_request extts;
	memset(&extts, 0, sizeof(extts));
	extts.index = 2;  // SDP pin index
	extts.flags = PTP_ENABLE_FEATURE | PTP_RISING_EDGE;

	if (ioctl(fd, PTP_EXTTS_REQUEST, &extts) < 0) {
		perror("PTP_EXTTS_REQUEST");
		close(fd);
		return EXIT_FAILURE;
	}

	printf("Enabled external timestamping on %s (index=%d) with flags=0x%x\n",
	       ptp_device, extts.index, extts.flags);

	// 進入事件迴圈以讀取時間戳 (第二階段：read)
	for (int i = 0; i < n_events; i++) {
		int ret = read(fd, &event, sizeof(event));
		if (ret!= sizeof(event)) {
			if (ret < 0) {
				perror("read 事件失敗");
			} else {
				fprintf(stderr, "讀取到不完整的事件資料 (%d bytes)\n", ret);
			}
			break;
		}
		printf("事件 %2d: index %u, 時間戳: %lld.%09u\n",
		       i + 1, event.index, (long long)event.t.sec, event.t.nsec);
	}

        close(fd);

	return EXIT_SUCCESS;
}
