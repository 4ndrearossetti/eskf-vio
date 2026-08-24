#include "eskf.h"
#include "ins.h"

enum { POS = 0, VEL = 3, TH = 6, BA = 9, BG = 12 };

mat_t mat_skew(vector_3d_t a) {
        mat_t M = mat_zero(3, 3);
        mat_set(&M, 0, 1, -a.z);
        mat_set(&M, 0, 2, a.y);
        mat_set(&M, 1, 0, a.z);
        mat_set(&M, 1, 2, -a.x);
        mat_set(&M, 2, 0, -a.y);
        mat_set(&M, 2, 1, a.x);
        return M;
}

mat_t quat_to_R(quaternion_t q) {
        mat_t R = mat_zero(3, 3);
        mat_set(&R, 0, 0, 1 - 2*(q.y*q.y + q.z*q.z));
        mat_set(&R, 0, 1, 2*(q.x*q.y - q.z*q.w));
        mat_set(&R, 0, 2, 2*(q.x*q.z + q.y*q.w));
        mat_set(&R, 1, 0, 2*(q.x*q.y + q.z*q.w));
        mat_set(&R, 1, 1, 1 - 2*(q.x*q.x + q.z*q.z));
        mat_set(&R, 1, 2, 2*(q.y*q.z - q.x*q.w));
        mat_set(&R, 2, 0, 2*(q.x*q.z - q.y*q.w));
        mat_set(&R, 2, 1, 2*(q.y*q.z + q.x*q.w));
        mat_set(&R, 2, 2, 1 - 2*(q.x*q.x + q.y*q.y));
        return R;
}

mat_t build_F(quaternion_t q, vector_3d_t a, vector_3d_t w, double dt) {
        mat_t F = mat_eye(15);
        mat_t R = quat_to_R(q);

        // dp += dv * dt  (cell 0,1)
        mat_set(&F, POS+0, VEL+0, dt);
        mat_set(&F, POS+1, VEL+1, dt);
        mat_set(&F, POS+2, VEL+2, dt);

        // dv += -R * [a]_x * dt  (cell 1,2)
        mat_t RA = mat_mul(R, mat_skew(a));
        for (size_t i = 0; i < 3; i++)
                for (size_t j = 0; j < 3; j++)
                        mat_set(&F, VEL+i, TH+j, -dt * mat_get(RA, i, j));

        // dv += -R * da_b * dt  (cell 1,3)
        for (size_t i = 0; i < 3; i++)
                for (size_t j = 0; j < 3; j++)
                        mat_set(&F, VEL+i, BA+j, -dt * mat_get(R, i, j));

        // dw += R^T * dw * dt (cell 2, 2)
        mat_t Rw = mat_transpose(quat_to_R(gyro_to_q(w, dt)));
        for (size_t i = 0; i < 3; i++)
                for (size_t j = 0; j < 3; j++)
                        mat_set(&F, TH+i, TH+j, mat_get(Rw, i, j));

        // dw += -dw_b * dt  (cell 2,4)
        mat_set(&F, TH+0, BG+0, -dt);
        mat_set(&F, TH+1, BG+1, -dt);
        mat_set(&F, TH+2, BG+2, -dt);

        return F;
}

// IMU noise params — EuRoC imu0/sensor.yaml (ADIS16448)
#define SIG_A   2.0000e-3    // accel noise density   [m/s²/√Hz]
#define SIG_G   1.6968e-4    // gyro  noise density   [rad/s/√Hz]
#define SIG_BA  3.0000e-3    // accel bias random walk [m/s³/√Hz]
#define SIG_BG  1.9393e-5    // gyro  bias random walk [rad/s²/√Hz]

mat_t build_Q(double dt) {
        mat_t Q = mat_zero(15, 15);

        double qv  = SIG_A * SIG_A * dt;
        double qth = SIG_G * SIG_G * dt;
        double qba = SIG_BA * SIG_BA * dt;
        double qbg = SIG_BG * SIG_BG * dt;

        for (size_t i = 0; i < 3; i++) {
                mat_set(&Q, VEL+i, VEL+i, qv);
                mat_set(&Q, TH+i,  TH+i,  qth);
                mat_set(&Q, BA+i,  BA+i,  qba);
                mat_set(&Q, BG+i,  BG+i,  qbg);
        }
        return Q;
}

void eskf_init(eskf_t *filter) {
        filter->P = mat_zero(15, 15);
        for (size_t i = BA; i < BA+3; i++)  mat_set(&filter->P, i, i, 1e-6);
        for (size_t i = BG; i < 15;   i++)  mat_set(&filter->P, i, i, 1e-8);
}

void eskf_predict(eskf_t *filter, quaternion_t q, vector_3d_t a, vector_3d_t w, double dt) {
        mat_t F = build_F(q, a, w, dt);
        mat_t Ft = mat_transpose(F);
        mat_t Q = build_Q(dt);
        filter->P = mat_add(mat_mul(mat_mul(F, filter->P), Ft), Q);
}

