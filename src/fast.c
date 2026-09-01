#include <stdlib.h>
#include <string.h>
#include "fast.h"

static const int RING[16][2] = {
        { 0,-3},{ 1,-3},{ 2,-2},{ 3,-1},{ 3, 0},{ 3, 1},{ 2, 2},{ 1, 3},
        { 0, 3},{-1, 3},{-2, 2},{-3, 1},{-3, 0},{-3,-1},{-2,-2},{-1,-3},
};

static int corner_score(const image_t *img, int x, int y, int t) {
        int c = img->data[y*img->w + x];
        int diff[16];
        for (int i = 0; i < 16; i++)
                diff[i] = (int)img->data[(y + RING[i][1])*img->w + (x + RING[i][0])] - c;

        int best = 0;
        for (int start = 0; start < 16; start++) {
                int sb = 0, sd = 0, ok_b = 1, ok_d = 1;
                for (int k = 0; k < 9; k++) {
                        int d = diff[(start + k) & 15];
                        if (d >  t) sb += d - t;  else ok_b = 0;
                        if (d < -t) sd += -d - t; else ok_d = 0;
                }
                if (ok_b && sb > best) best = sb;
                if (ok_d && sd > best) best = sd;
        }
        return best;
}

int fast_detect(const image_t *img, int threshold, corner_t *out, int max_out) {
        int n = 0;
        for (int y = 3; y < img->h - 3 && n < max_out; y++)
                for (int x = 3; x < img->w - 3 && n < max_out; x++) {
                        int s = corner_score(img, x, y, threshold);
                        if (s > 0)
                                out[n++] = (corner_t){ x, y, s };
                }
        return n;
}

static int cmp_score_desc(const void *a, const void *b) {
        return ((const corner_t *)b)->score - ((const corner_t *)a)->score;
}

int fast_select(const corner_t *in, int n, int w, int h,
                int nms_r, int grid_x, int grid_y, int per_cell,
                corner_t *out, int max_out) {
        corner_t *s = malloc((size_t)n * sizeof *s);
        memcpy(s, in, (size_t)n * sizeof *s);
        qsort(s, (size_t)n, sizeof *s, cmp_score_desc);

        unsigned char *occ = calloc((size_t)w * h, 1);
        int *cell_count = calloc((size_t)grid_x * grid_y, sizeof *cell_count);
        int cw = w / grid_x, ch = h / grid_y;

        int m = 0;
        for (int i = 0; i < n && m < max_out; i++) {
                int x = s[i].x, y = s[i].y;
                if (occ[y*w + x]) continue;

                int cx = x / cw, cy = y / ch;
                if (cx >= grid_x) cx = grid_x - 1;
                if (cy >= grid_y) cy = grid_y - 1;
                int cell = cy * grid_x + cx;
                if (cell_count[cell] >= per_cell) continue;

                out[m++] = s[i];
                cell_count[cell]++;
                for (int dy = -nms_r; dy <= nms_r; dy++)
                        for (int dx = -nms_r; dx <= nms_r; dx++) {
                                int xx = x + dx, yy = y + dy;
                                if (xx >= 0 && xx < w && yy >= 0 && yy < h)
                                        occ[yy*w + xx] = 1;
                        }
        }

        free(s);
        free(occ);
        free(cell_count);
        return m;
}

