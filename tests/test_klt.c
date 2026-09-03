#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "klt.h"

static int nfail = 0;
static void check(const char *name, int cond) {
        printf("%-44s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

static uint32_t rng = 12345u;
static uint32_t xr(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static image_t make_texture(int w, int h) {
        image_t a = { w, h, malloc((size_t)w*h) }, b = { w, h, malloc((size_t)w*h) };
        for (int k = 0; k < w*h; k++) a.data[k] = (uint8_t)(xr() & 255);
        for (int pass = 0; pass < 3; pass++) {
                memcpy(b.data, a.data, (size_t)w*h);
                for (int y = 1; y < h-1; y++)
                        for (int x = 1; x < w-1; x++) {
                                int s = 0;
                                for (int j = -1; j <= 1; j++)
                                        for (int i = -1; i <= 1; i++)
                                                s += a.data[(y+j)*w + x+i];
                                b.data[y*w + x] = (uint8_t)(s / 9);
                        }
                memcpy(a.data, b.data, (size_t)w*h);
        }
        free(b.data);
        return a;
}

static image_t shift(const image_t *I, double dx, double dy) {
        image_t J = { I->w, I->h, calloc((size_t)I->w * I->h, 1) };
        for (int y = 0; y < I->h; y++)
                for (int x = 0; x < I->w; x++) {
                        double sx = x - dx, sy = y - dy;
                        if (sx < 0 || sy < 0 || sx >= I->w - 1 || sy >= I->h - 1) continue;
                        int x0 = (int)sx, y0 = (int)sy;
                        double fx = sx - x0, fy = sy - y0;
                        const uint8_t *p = I->data + y0*I->w + x0;
                        double v = (p[0]*(1-fx) + p[1]*fx)*(1-fy) + (p[I->w]*(1-fx) + p[I->w+1]*fx)*fy;
                        J.data[y*I->w + x] = (uint8_t)(v + 0.5);
                }
        return J;
}

static void run(const image_t *I, const image_t *J, double dx, double dy,
                int win, int levels, double *frac, double *err) {
        pt2_t p0[64] = {{0,0}}, p1[64] = {{0,0}}; unsigned char st[64];
        int n = 0;
        for (int y = 80; y <= I->h - 80; y += 20)
                for (int x = 80; x <= I->w - 80 && n < 64; x += 20) {
                        p0[n] = (pt2_t){ x, y };  p1[n] = p0[n];  n++;
                }
        int tr = klt_track(I, J, p0, p1, st, n, win, levels);
        double e = 0; int c = 0;
        for (int i = 0; i < n; i++)
                if (st[i]) { e += fabs(p1[i].x - p0[i].x - dx) + fabs(p1[i].y - p0[i].y - dy); c++; }
        *frac = (double)tr / n;
        *err  = c ? e / (2*c) : 1e9;
}

int main(void) {
        image_t I = make_texture(480, 400);
        double frac, err;

        image_t J1 = shift(&I, 1.3, -0.8);
        run(&I, &J1, 1.3, -0.8, 7, 1, &frac, &err);
        printf("  small shift, 1 level:  tracked %.0f%%, mean err %.3f px\n", 100*frac, err);
        check("klt: small shift, single level, all tracked", frac > 0.95);
        check("klt: small shift recovered to < 0.05 px",   err < 0.05);

        image_t J2 = shift(&I, 12.4, -7.6);
        run(&I, &J2, 12.4, -7.6, 7, 1, &frac, &err);
        printf("  large shift, 1 level:  tracked %.0f%%, mean err %.3f px  (expected to struggle)\n", 100*frac, err);
        run(&I, &J2, 12.4, -7.6, 7, 4, &frac, &err);
        printf("  large shift, 4 levels: tracked %.0f%%, mean err %.3f px\n", 100*frac, err);
        check("klt: large shift with pyramid, most tracked", frac > 0.85);
        check("klt: large shift recovered to < 0.1 px",     err < 0.1);

        free(I.data); free(J1.data); free(J2.data);
        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

