#ifndef TRI_H
#define TRI_H
#include "eskf.h"
#include "klt.h"

int triangulate(const clone_t *cl, const pt2_t *obs, int n, vector_3d_t *out);

#endif

