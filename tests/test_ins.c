#include <stdio.h>
#include <math.h>
#include "ins.h"

static int nfail = 0;

static void check(const char *name, int cond) {
        printf("%-28s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

static int qeq(quaternion_t a, double w, double x, double y, double z) {
        return fabs(a.w-w) < 1e-12 && fabs(a.x-x) < 1e-12 &&
               fabs(a.y-y) < 1e-12 && fabs(a.z-z) < 1e-12;
}

int main(void) {
        const double s = 0.7071067811865476;    // cos45 = sin45

        vector_3d_t wz = { 0, 0, M_PI / 2.0 };
        check("gyro: pi/2 about z, 1s -> 90z", qeq(gyro_to_q(wz, 1.0), s, 0, 0, s));

        vector_3d_t wx = { M_PI / 2.0, 0, 0 };
        check("gyro: pi/2 about x, 1s -> 90x", qeq(gyro_to_q(wx, 1.0), s, s, 0, 0));

        vector_3d_t wy = { 0, M_PI / 2.0, 0 };
        check("gyro: pi/2 about y, 1s -> 90y", qeq(gyro_to_q(wy, 1.0), s, 0, s, 0));

        vector_3d_t w0 = { 0, 0, 0 };
        check("gyro: zero rate -> identity",   qeq(gyro_to_q(w0, 0.005), 1, 0, 0, 0));

        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

