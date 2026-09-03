#ifndef FRONTEND_H
#define FRONTEND_H
#include "image.h"
#include "klt.h"

#define FE_MAX 256

typedef struct {
        int id;
        pt2_t pt;
        int age;
} feature_t;

typedef struct {
        image_t prev;
        int has_prev;
        feature_t f[FE_MAX];
        int n;
        int next_id;
} frontend_t;

void frontend_init(frontend_t *fe);
void frontend_process(frontend_t *fe, const image_t *frame);
void frontend_free(frontend_t *fe);

#endif
