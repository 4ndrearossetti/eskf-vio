#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "euroc.h"
#include "quat.h"
#include "eskf.h"

static size_t gt_nearest(const gt_sample_t *gt, size_t n, double t) {
        for (size_t i=0; i<n; i++)
                if (gt[i].timestamp >= t)
                        return i;
        return n-1;
}

static double gauss(void) {
        double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
        double u2 =  rand()        / (RAND_MAX + 1.0);
        return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

int main(int argc, char *argv[]) {
        if (argc < 4) { fprintf(stderr, "usage: %s <imu.csv> <gt.csv>\n", argv[0]); return 1; }

        imu_sample_t *imu;
        size_t n = euroc_load_imu(argv[1], &imu);
        if (n == 0) return 1;

        gt_sample_t *gt;
        size_t m = euroc_load_gt(argv[2], &gt);
        if (m == 0) return 1;

        cam_frame_t *cam;
        size_t nc = euroc_load_cam(argv[3], &cam);
        if (nc == 0) return 1;
        printf("%zu frames, %.1f s, %.0f Hz, first: %s\n", nc,
               cam[nc-1].timestamp - cam[0].timestamp,
               (double)nc / (cam[nc-1].timestamp - cam[0].timestamp),
               cam[0].filename);
        free(cam);

        size_t i0 = 0;
        while (i0 < n && imu[i0].timestamp < gt[0].timestamp)
                i0++;

        size_t j0 = gt_nearest(gt, m, imu[i0].timestamp);

        eskf_t f;
        eskf_init(&f, gt[j0].q, gt[j0].pos, gt[j0].vel, gt[0].accel_bias, gt[0].gyro_bias);

        printf("%-8s %-12s %-12s %-10s\n", "t [s]", "pos err [m]", "pred +- [m]", "att err [deg]");

        srand(42);
        for (size_t k = i0; k < n-1; k++) {
                double dt = imu[k+1].timestamp - imu[k].timestamp;
                eskf_predict(&f, imu[k], dt);

                const double sigma_z = 0.05;
                if ((k - i0) % 10 == 0) {
                        gt_sample_t *g = &gt[gt_nearest(gt, m, imu[k].timestamp)];
                        vector_3d_t z = { g->pos.x + sigma_z*gauss(),
                                          g->pos.y + sigma_z*gauss(),
                                          g->pos.z + sigma_z*gauss() };
                        eskf_update_pos(&f, z, sigma_z);
                }

                if ((k - i0) % 4000 == 0) {
                        gt_sample_t *g = &gt[gt_nearest(gt, m, imu[k].timestamp)];
                        double dx = f.pos.x - g->pos.x, dy = f.pos.y - g->pos.y, dz = f.pos.z - g->pos.z;
                        double dot = f.q.w*g->q.w + f.q.x*g->q.x + f.q.y*g->q.y + f.q.z*g->q.z;
                        printf("%-8.1f %-12.2f %-12.2f %-10.2f\n",
                               imu[k].timestamp - imu[i0].timestamp,
                               sqrt(dx*dx + dy*dy + dz*dz),
                               sqrt(mat_get(f.P, 0, 0)),
                               2.0 * acos(fabs(dot)) * 180.0 / M_PI);
                }
        }

        gt_sample_t *g = &gt[gt_nearest(gt, m, imu[n-1].timestamp)];
        double dx = f.pos.x - g->pos.x, dy = f.pos.y - g->pos.y, dz = f.pos.z - g->pos.z;
        printf("final: measured %.1f m, predicted +-%.1f m\n",
               sqrt(dx*dx + dy*dy + dz*dz),
               sqrt(mat_get(f.P, 0, 0)));
        printf("att 1-sigma: %.3f deg\n", sqrt(mat_get(f.P, 6, 6)) * 180.0 / M_PI);
        gt_sample_t *ge = &gt[m-1];
        printf("gyro bias est %.5f %.5f %.5f | true %.5f %.5f %.5f\n",
               f.bg.x, f.bg.y, f.bg.z, ge->gyro_bias.x, ge->gyro_bias.y, ge->gyro_bias.z);

        free(imu);
        free(gt);
        return 0;
}

