#include <stdio.h>
#include <math.h>
#include "cam.h"

static int nfail = 0;

static void check(const char *name, int cond) {
        printf("%-40s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

int main(void) {
        const cam_t *c = &EUROC_CAM0;
        double u, v;

        double pts[][2] = {{367.215, 248.375}, {50, 40}, {700, 450}, {376, 10}};
        int ok = 1;
        for (int k = 0; k < 4; k++) {
                vector_3d_t r = cam_unproject(c, pts[k][0], pts[k][1]);
                cam_project(c, r, &u, &v);
                ok &= fabs(u - pts[k][0]) < 1e-6 && fabs(v - pts[k][1]) < 1e-6;
                printf("  (%g,%g) -> residual %.2e px\n", pts[k][0], pts[k][1],
                sqrt((u-pts[k][0])*(u-pts[k][0]) + (v-pts[k][1])*(v-pts[k][1])));
        }
        check("cam: project(unproject(px)) = px", ok);

        cam_project(c, (vector_3d_t){0, 0, 1}, &u, &v);
        check("cam: optical axis -> principal point",
              fabs(u - c->cu) < 1e-12 && fabs(v - c->cv) < 1e-12);

        check("cam: behind camera rejected",
              cam_project(c, (vector_3d_t){0, 0, -1}, &u, &v) == 0);

        vector_3d_t r = cam_unproject(c, 100, 100);
        check("cam: unproject gives z = 1", r.z == 1.0);

        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

