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

#define DURATION 10 // Duration to observe the perout behavior

int main() {
    const char *ptp_device = "/dev/ptp0";
    int fd = open(ptp_device, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    struct ptp_perout_request perout;
    memset(&perout, 0, sizeof(perout));

    // 設定從現在起 500 ms 後開始，並每 1 秒觸發一次, assert high 0.5 秒
    perout.index = 1;
    perout.phase.sec = 0;
    perout.phase.nsec = 500000000;
    perout.period.sec = 1;
    perout.period.nsec = 0;
    perout.flags = PTP_PEROUT_DUTY_CYCLE | PTP_PEROUT_PHASE ;
    perout.on.sec = 0;
    perout.on.nsec = 500000000;

    if (ioctl(fd, PTP_PEROUT_REQUEST2, &perout) < 0) {
        perror("ioctl PTP_PEROUT_REQUEST2");
        close(fd);
        return 1;
    }

    time_t now = time(NULL);

    unsigned int start_nsec = perout.phase.nsec;
    unsigned long long start_sec = perout.phase.sec + now;

    printf("PTP perout started AFTER %lld.%d sec, period %lld.%d sec, asserted %lld.%d sec, duration = %d sec.\n",
           start_sec, start_nsec,
	   perout.period.sec, perout.period.nsec,
	   perout.on.sec, perout.on.nsec,
	   DURATION);

    // 等待一段時間觀察 perout 行為
    sleep(10);

    // 若要取消 perout，可再送一次 request，但 period 設為 0
    perout.period.sec = 0;
    perout.period.nsec = 0;
    perout.flags = 0;
    perout.on.sec = 0;
    perout.on.nsec = 0;

    if (ioctl(fd, PTP_PEROUT_REQUEST2, &perout) < 0) {
        perror("ioctl cancel PTP_PEROUT_REQUEST2");
    } else {
        printf("PTP perout test finished.\n");
    }

    close(fd);
    return 0;
}
