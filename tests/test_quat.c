#include <stdio.h>
#include <math.h>
#include "../src/quat.h"

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
        const double s = 0.7071067811865476;    // cos45 = sin45
        quaternion_t q90z = { s, 0, 0, s };     // 90deg yaw about z
        quaternion_t id   = { 1, 0, 0, 0 };

        // --- q_mul_q ---
        check("mul: 90z*90z = 180z",
              qeq(q_mul_q(q90z, q90z), 0, 0, 0, 1));
        check("mul: identity is neutral",
              qeq(q_mul_q(id, q90z), s, 0, 0, s) &&
              qeq(q_mul_q(q90z, id), s, 0, 0, s));
        check("mul: q * conj(q) = identity",
              qeq(q_mul_q(q90z, q_conj(q90z)), 1, 0, 0, 0));

        // --- non-commutativity: 90° about x then z vs z then x differ ---
        quaternion_t q90x = { s, s, 0, 0 };
        quaternion_t xz = q_mul_q(q90x, q90z);
        quaternion_t zx = q_mul_q(q90z, q90x);
        check("mul: xz != zx",
              fabs(xz.y - zx.y) > 1e-3);        // they differ in the y component

        // --- q_norm ---
        quaternion_t denorm = { 2, 0, 0, 0 };
        check("norm: (2,0,0,0) -> identity", qeq(q_norm(denorm), 1, 0, 0, 0));
        check("norm: unit stays unchanged",  qeq(q_norm(q90z), s, 0, 0, s));

        // --- rotate_vector ---
        vector_3d_t v = { 1, 0, 0 };
        rotate_vector(&v, q90z);                // x-axis yawed 90° -> +y
        check("rot: 90z takes x to y", veq(v, 0, 1, 0));

        vector_3d_t g = { 0, 0, -9.81 };
        rotate_vector(&g, q90z);                // yaw must not touch vertical
        check("rot: yaw leaves z alone", veq(g, 0, 0, -9.81));

        vector_3d_t vx = { 1, 0, 0 };
        rotate_vector(&vx, q90x);               // rotation about own axis: no-op
        check("rot: axis vector invariant", veq(vx, 1, 0, 0));

        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

