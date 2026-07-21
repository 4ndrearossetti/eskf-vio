#include <stdio.h>
#include <stdlib.h>

typedef struct {
        double timestamp;
        double gyro[3];
        double accel[3];
} imu_sample_t;

int main(int argc, char *argv[]) {
        if (argc < 2) { fprintf(stderr, "usage: %s <file>\n", argv[0]); return 1; }

        FILE *f = fopen(argv[1], "r");
        if (!f) { perror(argv[1]); return 1; }

        char line[256];
        size_t n = 0;  // lines counter
        while (fgets(line, sizeof line, f))
                if (line[0] != '#') n++;
        rewind(f);

        imu_sample_t *samples = malloc(n * sizeof *samples);
        if (!samples) { perror("malloc\n"); return 1; }

        size_t i = 0; // index for samples
        while (fgets(line, sizeof line, f)) {
                if (line[0] == '#') continue;  // skip header

                long long ts;
                double w[3], a[3];
                if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf",
                                &ts, &w[0], &w[1], &w[2], &a[0], &a[1], &a[2]) != 7)
                        continue;  // skip blank/malformed lines

                samples[i].timestamp = ts / 1e9;
                samples[i].gyro[0]  = w[0];
                samples[i].gyro[1]  = w[1];
                samples[i].gyro[2]  = w[2];
                samples[i].accel[0] = a[0];
                samples[i].accel[1] = a[1];
                samples[i].accel[2] = a[2];
                i++;
        }
        fclose(f);

        double duration = samples[i-1].timestamp - samples[0].timestamp;
        printf("%zu samples, %.1f s, %.0f Hz\n", i, duration, (double)i / duration);

        free(samples);

        return 0;
}

