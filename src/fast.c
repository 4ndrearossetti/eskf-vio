#include "fast.h"

static const int RING[16][2] = {
        { 0,-3},{ 1,-3},{ 2,-2},{ 3,-1},{ 3, 0},{ 3, 1},{ 2, 2},{ 1, 3},
        { 0, 3},{-1, 3},{-2, 2},{-3, 1},{-3, 0},{-3,-1},{-2,-2},{-1,-3},
};

static int is_corner(const image_t *img, int x, int y, int t) {
        int c = img->data[y*img->w + x];
        int bright[16], dark[16];
        for (int i = 0; i < 16; i++) {
                int p = img->data[(y + RING[i][1])*img->w + (x + RING[i][0])];
                bright[i] = p > c + t;
                dark[i]   = p < c - t;
        }
        // longest run of consecutive 1s, treating the ring as circular
        for (int start = 0; start < 16; start++) {
                int rb = 0, rd = 0;
                for (int k = 0; k < 9; k++) {
                        int i = (start + k) & 15;
                        rb += bright[i];  rd += dark[i];
                }
                if (rb == 9 || rd == 9) return 1;
        }
        return 0;
}

int fast_detect(const image_t *img, int threshold, corner_t *out, int max_out) {
        int n = 0;
        for (int y = 3; y < img->h - 3 && n < max_out; y++)
                for (int x = 3; x < img->w - 3 && n < max_out; x++)
                        if (is_corner(img, x, y, threshold))
                                out[n++] = (corner_t){ x, y, 0 };
        return n;
}

