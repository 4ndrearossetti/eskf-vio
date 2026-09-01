#ifndef FAST_H
#define FAST_H
#include "image.h"

typedef struct {
        int x, y;
        int score;
} corner_t;

int fast_detect(const image_t *img, int threshold, corner_t *out, int max_out);

#endif

