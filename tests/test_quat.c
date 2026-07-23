#include <stdio.h>
#include <math.h>
#include "../src/quat.h"

int main(void) {
        quaternion_t q90 = { 0.7071067811865476, 0, 0, 0.7071067811865476 };
        quaternion_t r = q_mul_q(q90, q90);

        printf("%f %f %f %f\n", r.w, r.x, r.y, r.z);

        int ok = fabs(r.w) < 1e-12 && fabs(r.x) < 1e-12 &&
                 fabs(r.y) < 1e-12 && fabs(r.z - 1.0) < 1e-12;
        printf(ok ? "PASS\n" : "FAIL\n");
        return !ok;
}

