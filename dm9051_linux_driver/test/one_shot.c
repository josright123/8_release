// gcc -o perout_test perout.c

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/ptp_clock.h>
#include <time.h>
#include <errno.h>

#define DURATION 4 // Duration to observe the perout behavior

int main() {
    const char *ptp_device = "/dev/ptp0";
    int fd = open(ptp_device, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    struct ptp_perout_request perout;
    memset(&perout, 0, sizeof(perout));

    // 設定從現在起 2 秒後開始
    time_t now = time(NULL);
    perout.index = 1;
    perout.start.sec = now + 2;
    perout.start.nsec = 0;
    perout.period.sec = 1;
    perout.period.nsec = 0;
    perout.flags = PTP_PEROUT_ONE_SHOT; // 使用 one shot 模式
    // 後面 on time 必須清爲零, kernel 會檢查

    if (ioctl(fd, PTP_PEROUT_REQUEST2, &perout) < 0) {
        perror("ioctl PTP_PEROUT_REQUEST");
        close(fd);
        return 1;
    }

    printf("PTP perout started on %lld.%d sec, period %lld.%d sec, asserted %lld.%d sec, duration = %d sec.\n",
           perout.start.sec, perout.start.nsec,
	   perout.period.sec, perout.period.nsec,
	   perout.on.sec, perout.on.nsec,
	   DURATION);


    // 等待一段時間觀察 perout 行為
    sleep(DURATION);

    // 若要取消 perout，可再送一次 request，但 period 設為 0
    perout.period.sec = 0;
    perout.period.nsec = 0;
    perout.flags = 0;
    perout.on.sec = 0;
    perout.on.nsec = 0;

    if (ioctl(fd, PTP_PEROUT_REQUEST2, &perout) < 0) {
        perror("ioctl cancel PTP_PEROUT_REQUEST");
    } else {
        printf("PTP perout test finished.\n");
    }

    close(fd);
    return 0;
}
