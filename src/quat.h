#ifndef QUAT_H
#define QUAT_H

typedef struct {
        double x;
        double y;
        double z;
} vector_3d_t;

typedef struct {
        double w;
        double x;
        double y;
        double z;
} quaternion_t;

quaternion_t v_to_q(vector_3d_t v);
quaternion_t q_conj(quaternion_t q);
quaternion_t q_mul_q(quaternion_t a, quaternion_t b);
void rotate_vector(vector_3d_t *v, quaternion_t q);

#endif

