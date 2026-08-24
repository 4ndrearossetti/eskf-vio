#include <stdio.h>
#include <math.h>
#include "quat.h"

static int nfail = 0;

static void check(const char *name, int cond) {
        printf("%-28s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

static int qeq(quaternion_t a, double w, double x, double y, double z) {
        return fabs(a.w-w) < 1e-12 && fabs(a.x-x) < 1e-12 &&
               fabs(a.y-y) < 1e-12 && fabs(a.z-z) < 1e-12;
}

static int veq(vector_3d_t a, double x, double y, double z) {
        return fabs(a.x-x) < 1e-12 && fabs(a.y-y) < 1e-12 && fabs(a.z-z) < 1e-12;
}

int main(void) {
        const double s = 0.7071067811865476;  // cos45 = sin45
        quaternion_t q90z = { s, 0, 0, s };  // 90deg yaw about z
        quaternion_t id   = { 1, 0, 0, 0 };

        check("mul: 90z*90z = 180z",
              qeq(q_mul_q(q90z, q90z), 0, 0, 0, 1));
        check("mul: identity is neutral",
              qeq(q_mul_q(id, q90z), s, 0, 0, s) &&
              qeq(q_mul_q(q90z, id), s, 0, 0, s));
        check("mul: q * conj(q) = identity",
              qeq(q_mul_q(q90z, q_conj(q90z)), 1, 0, 0, 0));

        quaternion_t q90x = { s, s, 0, 0 };
        quaternion_t xz = q_mul_q(q90x, q90z);
        quaternion_t zx = q_mul_q(q90z, q90x);
        check("mul: xz != zx",
              fabs(xz.y - zx.y) > 1e-3);

        quaternion_t denorm = { 2, 0, 0, 0 };
        check("norm: (2,0,0,0) -> identity", qeq(q_norm(denorm), 1, 0, 0, 0));
        check("norm: unit stays unchanged",  qeq(q_norm(q90z), s, 0, 0, s));

        vector_3d_t v = { 1, 0, 0 };
        rotate_vector(&v, q90z);
        check("rot: 90z takes x to y", veq(v, 0, 1, 0));

        vector_3d_t g = { 0, 0, -9.81 };
        rotate_vector(&g, q90z);
        check("rot: yaw leaves z alone", veq(g, 0, 0, -9.81));

        vector_3d_t vx = { 1, 0, 0 };
        rotate_vector(&vx, q90x);
        check("rot: axis vector invariant", veq(vx, 1, 0, 0));

        quaternion_t q90y = { s, 0, s, 0 };

        check("mul: 90x*90x = 180x",
              qeq(q_mul_q(q90x, q90x), 0, 1, 0, 0));
        check("mul: 90y*90y = 180y",
              qeq(q_mul_q(q90y, q90y), 0, 0, 1, 0));

        vector_3d_t vy = { 0, 1, 0 };
        rotate_vector(&vy, q90x);
        check("rot: 90x takes y to z", veq(vy, 0, 0, 1));

        vector_3d_t vz = { 0, 0, 1 };
        rotate_vector(&vz, q90y);
        check("rot: 90y takes z to x", veq(vz, 1, 0, 0));

        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

