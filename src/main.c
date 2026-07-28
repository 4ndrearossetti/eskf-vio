#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "euroc.h"
#include "ins.h"
#include "quat.h"

static size_t gt_nearest(const gt_sample_t *gt, size_t n, double t) {
        for (size_t i=0; i<n; i++)
                if (gt[i].timestamp >= t)
                        return i;
        return n-1;
}

int main(int argc, char *argv[]) {
        if (argc < 3) { fprintf(stderr, "usage: %s <file>\n", argv[0]); return 1; }

        imu_sample_t *samples_imu;
        size_t n = euroc_load_imu(argv[1], &samples_imu);
        if (n == 0) return 1;

        gt_sample_t *samples_gt;
        size_t m = euroc_load_gt(argv[2], &samples_gt);
        if (m == 0) return 1;

        size_t i0 = 0;
        while (i0 < n && samples_imu[i0].timestamp < samples_gt[0].timestamp)
                i0++;

        quaternion_t q = samples_gt[gt_nearest(samples_gt, m, samples_imu[i0].timestamp)].q;
        printf("q0 = %.4f %.4f %.4f %.4f\n", q.w, q.x, q.y, q.z);

        vector_3d_t b = samples_gt[0].gyro_bias;
        printf("gyro bias: %.4f %.4f %.4f  |b|=%.4f rad/s\n",
               b.x, b.y, b.z, sqrt(b.x*b.x + b.y*b.y + b.z*b.z));

        for (size_t k = i0; k < n-1; k++) {
                double dt = samples_imu[k+1].timestamp - samples_imu[k].timestamp;

                vector_3d_t w = samples_imu[k].gyro;
                w.x -= b.x;
                w.y -= b.y;
                w.z -= b.z;

                quaternion_t dq = gyro_to_q(w, dt);
                q = q_norm(q_mul_q(q, dq));

                if ((k - i0) % 4000 == 0) {
                        quaternion_t qk = samples_gt[gt_nearest(samples_gt, m, samples_imu[k].timestamp)].q;
                        double dk = q.w*qk.w + q.x*qk.x + q.y*qk.y + q.z*qk.z;
                        printf("t=%6.1f  err=%6.2f deg\n",
                               samples_imu[k].timestamp - samples_imu[i0].timestamp,
                               2.0 * acos(fabs(dk)) * 180.0 / M_PI);
                }
        }
        printf("q = %.4f %.4f %.4f %.4f\n", q.w, q.x, q.y, q.z);

        quaternion_t qt = samples_gt[gt_nearest(samples_gt, m, samples_imu[n-1].timestamp)].q;
        double dot = q.w*qt.w + q.x*qt.x + q.y*qt.y + q.z*qt.z;
        double err = 2.0 * acos(fabs(dot)) * 180.0 / M_PI;
        printf("gt    = %.4f %.4f %.4f %.4f\n", qt.w, qt.x, qt.y, qt.z);
        printf("error = %.2f deg\n", err);

        free(samples_imu);
        free(samples_gt);

        return 0;
}

