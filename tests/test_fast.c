#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "fast.h"

static int nfail = 0;

static void check(const char *name, int cond) {
        printf("%-40s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

int main(void) {
        image_t img = { 64, 64, calloc(64*64, 1) };
        for (int y = 22; y < 42; y++)
                for (int x = 22; x < 42; x++)
                        img.data[y*64 + x] = 255;

        corner_t c[256];
        int n = fast_detect(&img, 50, c, 256);
        check("fast: finds corners", n > 0 && n < 40);

        /* every detection must be within 3 px of one of the four true corners */
        int true_c[4][2] = {{22,22},{41,22},{22,41},{41,41}};
        int all_near = 1;
        for (int i = 0; i < n; i++) {
                int near = 0;
                for (int k = 0; k < 4; k++)
                        if (abs(c[i].x - true_c[k][0]) <= 3 && abs(c[i].y - true_c[k][1]) <= 3) near = 1;
                all_near &= near;
        }
        check("fast: all detections near a true corner", all_near);
        free(img.data);

        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;

}

