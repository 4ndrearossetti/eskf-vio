#include <stdlib.h>
#include "klt.h"

#define MAX_LEVELS 5
#define MAX_WIN    15
#define MAX_ITERS  20

static image_t downsample(const image_t *s) {
        image_t d = { s->w / 2, s->h / 2, NULL };
        d.data = malloc((size_t)d.w * d.h);
        for (int y = 0; y < d.h; y++)
                for (int x = 0; x < d.w; x++) {
                        const uint8_t *r0 = s->data + (2*y) * s->w + 2*x;
                        const uint8_t *r1 = r0 + s->w;
                        d.data[y*d.w + x] = (uint8_t)((r0[0] + r0[1] + r1[0] + r1[1] + 2) / 4);
                }
        return d;
}

static double sample(const image_t *im, double x, double y) {
        int x0 = (int)x, y0 = (int)y;
        double fx = x - x0, fy = y - y0;
        const uint8_t *p = im->data + y0 * im->w + x0;
        double top = p[0]      * (1 - fx) + p[1]        * fx;
        double bot = p[im->w]  * (1 - fx) + p[im->w+1]  * fx;
        return top * (1 - fy) + bot * fy;
}

static int inside(const image_t *im, double x, double y, int margin) {
        return x >= margin && y >= margin &&
               x < im->w - 1 - margin && y < im->h - 1 - margin;
}

static int lk_level(const image_t *I, const image_t *J,
                    double px, double py, double *dx, double *dy, int win) {
        if (!inside(I, px, py, win + 1)) return 0;

        const int W = 2*win + 1;
        double T[(2*MAX_WIN+1)*(2*MAX_WIN+1)];
        double Gx[(2*MAX_WIN+1)*(2*MAX_WIN+1)];
        double Gy[(2*MAX_WIN+1)*(2*MAX_WIN+1)];

        double gxx = 0, gxy = 0, gyy = 0;
        int k = 0;
        for (int j = -win; j <= win; j++)
                for (int i = -win; i <= win; i++, k++) {
                        double x = px + i, y = py + j;
                        T[k]  = sample(I, x, y);
                        Gx[k] = 0.5 * (sample(I, x + 1, y) - sample(I, x - 1, y));
                        Gy[k] = 0.5 * (sample(I, x, y + 1) - sample(I, x, y - 1));
                        gxx += Gx[k]*Gx[k];  gxy += Gx[k]*Gy[k];  gyy += Gy[k]*Gy[k];
                }
        double det = gxx*gyy - gxy*gxy;
        if (det < 1e-6) return 0;

        for (int it = 0; it < MAX_ITERS; it++) {
                if (!inside(J, px + *dx, py + *dy, win + 1)) return 0;
                double bx = 0, by = 0;
                k = 0;
                for (int j = -win; j <= win; j++)
                        for (int i = -win; i <= win; i++, k++) {
                                double e = T[k] - sample(J, px + *dx + i, py + *dy + j);
                                bx += Gx[k] * e;  by += Gy[k] * e;
                        }
                double ux = ( gyy*bx - gxy*by) / det;
                double uy = (-gxy*bx + gxx*by) / det;
                *dx += ux;  *dy += uy;
                if (ux*ux + uy*uy < 1e-4) break;
        }
        (void)W;
        return 1;
}

int klt_track(const image_t *prev, const image_t *next,
              const pt2_t *p0, pt2_t *p1, unsigned char *status, int n,
              int win, int levels) {
        if (levels > MAX_LEVELS) levels = MAX_LEVELS;
        if (levels < 1) levels = 1;
        if (win > MAX_WIN) win = MAX_WIN;

        image_t pI[MAX_LEVELS], pJ[MAX_LEVELS];
        pI[0] = *prev;  pJ[0] = *next;
        for (int l = 1; l < levels; l++) {
                pI[l] = downsample(&pI[l-1]);
                pJ[l] = downsample(&pJ[l-1]);
        }

        int tracked = 0;
        for (int i = 0; i < n; i++) {
                double s  = 1.0 / (1 << (levels - 1));
                double dx = (p1[i].x - p0[i].x) * s;
                double dy = (p1[i].y - p0[i].y) * s;
                int ok = 0;
                for (int l = levels - 1; l >= 0; l--) {
                        double sc = 1.0 / (1 << l);
                        int r = lk_level(&pI[l], &pJ[l],
                                         p0[i].x * sc, p0[i].y * sc, &dx, &dy, win);
                        if (l == 0) ok = r;
                        if (l > 0) { dx *= 2; dy *= 2; }
                }
                if (ok) { p1[i].x = p0[i].x + dx;  p1[i].y = p0[i].y + dy; }
                status[i] = (unsigned char)ok;
                tracked += ok;
        }

        for (int l = 1; l < levels; l++) { free(pI[l].data); free(pJ[l].data); }
        return tracked;
}

