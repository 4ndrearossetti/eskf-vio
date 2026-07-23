#include <stdio.h>
#include <stdlib.h>
#include "euroc.h"

size_t euroc_load_imu(const char *path, imu_sample_t **out) {
        FILE *f = fopen(path, "r");
        if (!f) { perror(path); return 0; }

        char line[256];
        size_t n = 0;
        while (fgets(line, sizeof line, f))
                if (line[0] != '#') n++;
        rewind(f);

        imu_sample_t *s = malloc(n * sizeof *s);
        if (!s) { perror("malloc"); fclose(f); return 0; }

        size_t i = 0;
        while (fgets(line, sizeof line, f)) {
                if (line[0] == '#') continue;

                long long ts;
                imu_sample_t *m = &s[i];
                if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf",
                                &ts,
                                &m->gyro.x,  &m->gyro.y,  &m->gyro.z,
                                &m->accel.x, &m->accel.y, &m->accel.z) != 7)
                        continue;

                m->timestamp = ts / 1e9;
                i++;
        }
        fclose(f);

        *out = s;

        return i;
}

size_t euroc_load_gt(const char *path, gt_sample_t **out) {
        FILE *f = fopen(path, "r");
        if (!f) { perror(path); return 0; }

        char line[512];
        size_t n = 0;
        while (fgets(line, sizeof line, f))
                if (line[0] != '#') n++;
        rewind(f);

        gt_sample_t *s = malloc(n * sizeof *s);
        if (!s) { perror("malloc"); fclose(f); return 0; }

        size_t i = 0;
        while (fgets(line, sizeof line, f)) {
                if (line[0] == '#') continue;

                long long ts;
                gt_sample_t *g = &s[i];
                if (sscanf(line,
                        "%lld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                        &ts,
                        &g->pos.x, &g->pos.y, &g->pos.z,
                        &g->q.w, &g->q.x, &g->q.y, &g->q.z,
                        &g->vel.x, &g->vel.y, &g->vel.z,
                        &g->gyro_bias.x,  &g->gyro_bias.y,  &g->gyro_bias.z,
                        &g->accel_bias.x, &g->accel_bias.y, &g->accel_bias.z) != 17)
                        continue;

                g->timestamp = ts / 1e9;
                i++;
        }
        fclose(f);

        *out = s;

        return i;
}

