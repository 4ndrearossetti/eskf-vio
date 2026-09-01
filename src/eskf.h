#ifndef ESKF_H
#define ESKF_H
#include "imu.h"
#include "mat.h"
#include "quat.h"

typedef struct {
        quaternion_t q;
        vector_3d_t pos, vel;
        vector_3d_t ba, bg;
        mat_t P;
} eskf_t;

void eskf_init(eskf_t *f, quaternion_t q, vector_3d_t pos, vector_3d_t vel,
               vector_3d_t ba, vector_3d_t bg);
void eskf_predict(eskf_t *f, imu_sample_t s, double dt);

#endif

