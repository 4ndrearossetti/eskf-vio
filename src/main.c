#include <stdio.h>
#include <stdlib.h>
#include "euroc.h"

int main(int argc, char *argv[]) {
        if (argc < 3) { fprintf(stderr, "usage: %s <file>\n", argv[0]); return 1; }

        imu_sample_t *samples_imu;
        size_t n = euroc_load_imu(argv[1], &samples_imu);
        if (n == 0) return 1;

        double dur = samples_imu[n-1].timestamp - samples_imu[0].timestamp;
        printf("%zu samples, %.1f s, %.0f Hz\n", n, dur, (double)n / dur);
        free(samples_imu);

        gt_sample_t *samples_gt;
        n = euroc_load_gt(argv[2], &samples_gt);
        if (n == 0) return 1;

        dur = samples_gt[n-1].timestamp - samples_gt[0].timestamp;
        printf("%zu samples, %.1f s, %.0f Hz\n", n, dur, (double)n / dur);
        free(samples_gt);

        return 0;
}

