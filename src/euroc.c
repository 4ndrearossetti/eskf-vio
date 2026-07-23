#include <stdio.h>
#include <stdlib.h>
#include "euroc.h"

size_t euroc_load_imu(const char *path, imu_sample_t **out) {
        FILE *f = fopen(path, "r");
        if (!f) { perror(path); return 0; }

        char line[256];
        size_t n = 0;  // lines counter
        while (fgets(line, sizeof line, f))
                if (line[0] != '#') n++;
        rewind(f);

        imu_sample_t *s = malloc(n * sizeof *s);
        if (!s) { perror("malloc"); fclose(f); return 0; }

        size_t i = 0; // index for samples
        while (fgets(line, sizeof line, f)) {
                if (line[0] == '#') continue;  // skip header

                long long ts;
                double w[3], a[3];
                if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf",
                                &ts, &w[0], &w[1], &w[2], &a[0], &a[1], &a[2]) != 7)
                        continue;  // skip blank/malformed lines

                s[i].timestamp = ts / 1e9;
                s[i].gyro[0]   = w[0];
                s[i].gyro[1]   = w[1];
                s[i].gyro[2]   = w[2];
                s[i].accel[0]  = a[0];
                s[i].accel[1]  = a[1];
                s[i].accel[2]  = a[2];
                i++;
        }
        fclose(f);

        *out = s;

        return i;
}

