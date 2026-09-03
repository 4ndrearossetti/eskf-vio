#ifndef KLT_H
#define KLT_H
#include "image.h"

typedef struct {
        double x, y;
} pt2_t;

int klt_track(const image_t *prev, const image_t *next,
              const pt2_t *p0, pt2_t *p1, unsigned char *status, int n,
              int win, int levels);

#endif
