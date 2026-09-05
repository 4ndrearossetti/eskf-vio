#ifndef MSCKF_H
#define MSCKF_H
#include "eskf.h"
#include "klt.h"

typedef struct {
        double Hp[6];   // 2x3 row-major: dz/d clone-pos error
        double Hth[6];  // 2x3: dz/d clone-attitude error
        double Hf[6];   // 2x3: dz/d feature-pos error
        pt2_t  z;       // predicted normalised observation
        int    valid;   // 0 if point behind camera
} obs_jac_t;

obs_jac_t obs_jacobian(const clone_t *cl, vector_3d_t pf);

#endif

