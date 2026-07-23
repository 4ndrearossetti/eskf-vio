#include "quat.h"

quaternion_t v_to_q(vector_3d_t v) {
        quaternion_t v_as_q;
        v_as_q.w = 0;
        v_as_q.x = v.x;
        v_as_q.y = v.y;
        v_as_q.z = v.z;
        return v_as_q;
}

quaternion_t q_conj(quaternion_t q) {
        quaternion_t inverse_q;
        inverse_q.w =  q.w;
        inverse_q.x = -q.x;
        inverse_q.y = -q.y;
        inverse_q.z = -q.z;
        return inverse_q;
}

quaternion_t q_mul_q(quaternion_t a, quaternion_t b) {
        quaternion_t product_q;
        product_q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
        product_q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
        product_q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
        product_q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
        return product_q;
}

void rotate_vector(vector_3d_t *v, quaternion_t q) {
        quaternion_t v_as_q = v_to_q(*v);
        quaternion_t rotate_q;
        // v' = Q ∘ v ∘ Q^-1
        rotate_q = q_mul_q(q, v_as_q);
        rotate_q = q_mul_q(rotate_q, q_inv(q));

        v->x = rotate_q.x;
        v->y = rotate_q.y;
        v->z = rotate_q.z;
}

