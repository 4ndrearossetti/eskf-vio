#include "msckf.h"
#include "cam.h"

extern mat_t quat_to_R(quaternion_t q);
extern mat_t mat_skew(vector_3d_t v);

obs_jac_t obs_jacobian(const clone_t *cl, vector_3d_t pf) {
        obs_jac_t o = { .valid = 0 };

        mat_t Rwb = quat_to_R(cl->q);
        mat_t Rbs = mat_zero(3, 3);
        for (int k = 0; k < 9; k++) Rbs.d[k] = EUROC_T_BS.R[k];
        mat_t Rwc = mat_mul(Rwb, Rbs);
        mat_t Rcw = mat_transpose(Rwc);

        vector_3d_t t = EUROC_T_BS.t;
        vector_3d_t c = {
                cl->pos.x + Rwb.d[0]*t.x + Rwb.d[1]*t.y + Rwb.d[2]*t.z,
                cl->pos.y + Rwb.d[3]*t.x + Rwb.d[4]*t.y + Rwb.d[5]*t.z,
                cl->pos.z + Rwb.d[6]*t.x + Rwb.d[7]*t.y + Rwb.d[8]*t.z,
        };

        vector_3d_t d = { pf.x - c.x, pf.y - c.y, pf.z - c.z };
        double x = Rcw.d[0]*d.x + Rcw.d[1]*d.y + Rcw.d[2]*d.z;
        double y = Rcw.d[3]*d.x + Rcw.d[4]*d.y + Rcw.d[5]*d.z;
        double z = Rcw.d[6]*d.x + Rcw.d[7]*d.y + Rcw.d[8]*d.z;
        if (z <= 0.1) return o;

        o.z.x = x / z;
        o.z.y = y / z;

        // Jpi = (1/z) [1 0 -u; 0 1 -v]  (2x3)
        double Jpi[6] = { 1/z, 0, -o.z.x/z,  0, 1/z, -o.z.y/z };

        /* Hf = Jpi * Rcw ; Hp = -Hf */
        for (int r = 0; r < 2; r++)
                for (int cc = 0; cc < 3; cc++) {
                        double s = 0;
                        for (int k = 0; k < 3; k++)
                                s += Jpi[r*3 + k] * Rcw.d[k*3 + cc];
                        o.Hf[r*3 + cc] = s;
                        o.Hp[r*3 + cc] = -s;
                }

        // Hth = Jpi * Rbs^T * [Rwb^T (pf - p)]x
        vector_3d_t pb = { pf.x - cl->pos.x, pf.y - cl->pos.y, pf.z - cl->pos.z };
        vector_3d_t lb = {
                Rwb.d[0]*pb.x + Rwb.d[3]*pb.y + Rwb.d[6]*pb.z,
                Rwb.d[1]*pb.x + Rwb.d[4]*pb.y + Rwb.d[7]*pb.z,
                Rwb.d[2]*pb.x + Rwb.d[5]*pb.y + Rwb.d[8]*pb.z,
        };
        mat_t A = mat_mul(mat_transpose(Rbs), mat_skew(lb));
        for (int r = 0; r < 2; r++)
                for (int cc = 0; cc < 3; cc++) {
                        double s = 0;
                        for (int k = 0; k < 3; k++)
                                s += Jpi[r*3 + k] * A.d[k*3 + cc];
                        o.Hth[r*3 + cc] = s;
                }

        o.valid = 1;
        return o;
}

