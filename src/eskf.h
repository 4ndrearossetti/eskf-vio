#ifndef ESKF_H
#define ESKF_H
#include "mat.h"
#include "quat.h"

typedef struct {
        mat_t P;        // 15x15 error covariance
} eskf_t;

void eskf_init(eskf_t *filter);
void eskf_predict(eskf_t *filter, quaternion_t q, vector_3d_t a, vector_3d_t w, double dt);

#endif

