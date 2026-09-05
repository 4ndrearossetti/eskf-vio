#ifndef MSCKF_H
#define MSCKF_H
#include "eskf.h"
#include "klt.h"

typedef struct {
        double Hp[6];
        double Hth[6];
        double Hf[6];
        pt2_t  z;
        int    valid;
} obs_jac_t;

obs_jac_t obs_jacobian(const clone_t *cl, vector_3d_t pf);
mat_t msckf_nullspace(mat_t Hf);
int msckf_update_track(eskf_t *f, const int *ci, const pt2_t *obs, int k, double sigma);

#endif

