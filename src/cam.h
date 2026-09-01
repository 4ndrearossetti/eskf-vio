#ifndef CAM_H
#define CAM_H
#include "quat.h"

typedef struct {
        double fu, fv, cu, cv;  // intrinsics
        double k1, k2, p1, p2;  // radial-tangential distortion
} cam_t;

extern const cam_t EUROC_CAM0;

int cam_project(const cam_t *c, vector_3d_t p, double *u, double *v);
vector_3d_t cam_unproject(const cam_t *c, double u, double v);

#endif

