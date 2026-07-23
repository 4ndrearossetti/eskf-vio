#include <stdio.h>
#include <stdlib.h>
#include "euroc.h"
#include "quat.h"

int main(int argc, char *argv[]) {
        if (argc < 2) { fprintf(stderr, "usage: %s <file>\n", argv[0]); return 1; }

        imu_sample_t *samples;
        size_t n = euroc_load_imu(argv[1], &samples);
        if (n == 0) return 1;

        double dur = samples[n-1].timestamp - samples[0].timestamp;
        printf("%zu samples, %.1f s, %.0f Hz\n", n, dur, (double)n / dur);
        free(samples);

        return 0;
}

