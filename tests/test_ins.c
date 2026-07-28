#include <stdio.h>
#include <math.h>
#include "ins.h"

static int close_to(double a, double b) { return fabs(a - b) < 1e-12; }

int main(void) {
        int ok = 1;

        // main path: pi/2 rad/s about z for 1 s = 90° yaw
        // expect (cos 45°, 0, 0, sin 45°)
        vector_3d_t wz = { 0, 0, M_PI / 2.0 };
        quaternion_t q = gyro_to_q(wz, 1.0);
        printf("main path:  %f %f %f %f\n", q.w, q.x, q.y, q.z);
        ok &= close_to(q.w, 0.7071067811865476) && close_to(q.x, 0)
           && close_to(q.y, 0)                  && close_to(q.z, 0.7071067811865476);

        // guard path: zero gyro = identity, no NaN
        vector_3d_t w0 = { 0, 0, 0 };
        quaternion_t qi = gyro_to_q(w0, 0.005);
        printf("guard path: %f %f %f %f\n", qi.w, qi.x, qi.y, qi.z);
        ok &= close_to(qi.w, 1.0) && close_to(qi.x, 0)
           && close_to(qi.y, 0)   && close_to(qi.z, 0);

        // pi/2 rad/s about x for 1 s = 90° roll -> (cos45, sin45, 0, 0)
        vector_3d_t wx = { M_PI / 2.0, 0, 0 };
        quaternion_t qx = gyro_to_q(wx, 1.0);
        printf("x axis:     %f %f %f %f\n", qx.w, qx.x, qx.y, qx.z);
        ok &= close_to(qx.w, 0.7071067811865476) && close_to(qx.x, 0.7071067811865476)
           && close_to(qx.y, 0)                  && close_to(qx.z, 0);

        // pi/2 rad/s about y for 1 s = 90° pitch -> (cos45, 0, sin45, 0)
        vector_3d_t wy = { 0, M_PI / 2.0, 0 };
        quaternion_t qy = gyro_to_q(wy, 1.0);
        printf("y axis:     %f %f %f %f\n", qy.w, qy.x, qy.y, qy.z);
        ok &= close_to(qy.w, 0.7071067811865476) && close_to(qy.x, 0)
           && close_to(qy.y, 0.7071067811865476) && close_to(qy.z, 0);

        printf(ok ? "PASS\n" : "FAIL\n");
        return !ok;
}

