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
                vector_3d_t w, a;
                if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf",
                                &ts, &w.x, &w.y, &w.z, &a.x, &a.y, &a.z) != 7)
                        continue;  // skip blank/malformed lines

                s[i].timestamp = ts / 1e9;
                s[i].gyro.x    = w.x;
                s[i].gyro.y    = w.y;
                s[i].gyro.z    = w.z;
                s[i].accel.x   = a.x;
                s[i].accel.y   = a.y;
                s[i].accel.z   = a.z;
                i++;
        }
        fclose(f);

        *out = s;

        return i;
}

