#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "euroc.h"
#include "quat.h"
#include "eskf.h"
#include "image.h"
#include "frontend.h"
#include "msckf.h"

static size_t gt_nearest(const gt_sample_t *gt, size_t n, double t) {
        for (size_t i = 0; i < n; i++)
                if (gt[i].timestamp >= t)
                        return i;
        return n - 1;
}

int main(int argc, char *argv[]) {
        if (argc < 4) {
                fprintf(stderr, "usage: %s <imu.csv> <gt.csv> <cam0 dir>\n", argv[0]);
                return 1;
        }

        imu_sample_t *imu;
        size_t n = euroc_load_imu(argv[1], &imu);
        if (n == 0) return 1;

        gt_sample_t *gt;
        size_t m = euroc_load_gt(argv[2], &gt);
        if (m == 0) return 1;

        const char *cam_dir = argv[3];
        char path[512];
        snprintf(path, sizeof path, "%s/data.csv", cam_dir);
        cam_frame_t *cam;
        size_t ncam = euroc_load_cam(path, &cam);
        if (ncam == 0) return 1;

        size_t i0 = 0;
        while (i0 < n && imu[i0].timestamp < gt[0].timestamp)
                i0++;

        size_t j0 = gt_nearest(gt, m, imu[i0].timestamp);

        eskf_t f;
        eskf_init(&f, gt[j0].q, gt[j0].pos, gt[j0].vel, gt[0].accel_bias, gt[0].gyro_bias);

        frontend_t fe;
        frontend_init(&fe);
        size_t ic = 0;
        while (ic < ncam && cam[ic].timestamp < imu[i0].timestamp)
                ic++;

        int updates_ok = 0, updates_rej = 0;

        printf("%-8s %-12s %-12s %-10s\n", "t [s]", "pos err [m]", "pred +- [m]", "att err [deg]");

        for (size_t k = i0; k < n-1; k++) {
                double dt = imu[k+1].timestamp - imu[k].timestamp;
                eskf_predict(&f, imu[k], dt);

                if (ic < ncam && cam[ic].timestamp <= imu[k+1].timestamp) {
                        snprintf(path, sizeof path, "%s/data/%s", cam_dir, cam[ic].filename);
                        image_t img;
                        if (image_load(path, &img) == 0) {
                                eskf_augment(&f, cam[ic].timestamp);
                                frontend_process(&fe, &img);

                                for (int d = 0; d < fe.n_dead; d++) {
                                        dead_track_t *tk = &fe.dead[d];
                                        int kk = tk->nobs;
                                        if (kk > f.n_clones) continue;
                                        int ci[FE_HIST];
                                        for (int j = 0; j < kk; j++)
                                                ci[j] = f.n_clones - kk + j;
                                        if (msckf_update_track(&f, ci, tk->obs, kk, 3.0/458.0))
                                                updates_ok++;
                                        else
                                                updates_rej++;
                                }
                                image_free(&img);
                        }
                        ic++;
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
        printf("updates: %d ok, %d rejected\n", updates_ok, updates_rej);
        printf("gyro bias est %.5f %.5f %.5f | true %.5f %.5f %.5f\n",
               f.bg.x, f.bg.y, f.bg.z,
               gt[m-1].gyro_bias.x, gt[m-1].gyro_bias.y, gt[m-1].gyro_bias.z);

        frontend_free(&fe);
        free(imu);
        free(gt);
        free(cam);
        return 0;
}

