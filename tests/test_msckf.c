#include <stdio.h>
#include <math.h>
#include "msckf.h"
#include "ins.h"

static int nfail = 0;
static void check(const char *name, int cond) {
        printf("%-44s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

int main(void) {
        const double s2 = 0.7071067811865476;
        quaternion_t qc = q_norm((quaternion_t){ s2, 0.1, -0.05, s2 });
        clone_t cl = { qc, { 0.4, -0.2, 0.15 }, 0.0 };
        vector_3d_t pf = { 1.5, 0.8, 4.0 };
        const double eps = 1e-7;

        obs_jac_t o = obs_jacobian(&cl, pf);
        check("jac: valid", o.valid);

        double maxerr = 0;

        for (int k = 0; k < 3; k++) {
                clone_t p = cl;
                if (k == 0) p.pos.x += eps;
                if (k == 1) p.pos.y += eps;
                if (k == 2) p.pos.z += eps;
                obs_jac_t op = obs_jacobian(&p, pf);
                double n0 = (op.z.x - o.z.x) / eps, n1 = (op.z.y - o.z.y) / eps;
                double e = fmax(fabs(n0 - o.Hp[k]), fabs(n1 - o.Hp[3+k]));
                if (e > maxerr) maxerr = e;
        }
        printf("  Hp  max err %.2e\n", maxerr);
        check("jac: Hp matches finite differences", maxerr < 1e-5);

        maxerr = 0;
        for (int k = 0; k < 3; k++) {
                vector_3d_t dth = { 0, 0, 0 };
                if (k == 0) dth.x = eps;
                if (k == 1) dth.y = eps;
                if (k == 2) dth.z = eps;
                clone_t p = cl;
                p.q = q_norm(q_mul_q(cl.q, gyro_to_q(dth, 1.0)));
                obs_jac_t op = obs_jacobian(&p, pf);
                double n0 = (op.z.x - o.z.x) / eps, n1 = (op.z.y - o.z.y) / eps;
                double e = fmax(fabs(n0 - o.Hth[k]), fabs(n1 - o.Hth[3+k]));
                if (e > maxerr) maxerr = e;
        }
        printf("  Hth max err %.2e\n", maxerr);
        check("jac: Hth matches finite differences", maxerr < 1e-5);

        maxerr = 0;
        for (int k = 0; k < 3; k++) {
                vector_3d_t p = pf;
                if (k == 0) p.x += eps;
                if (k == 1) p.y += eps;
                if (k == 2) p.z += eps;
                obs_jac_t op = obs_jacobian(&cl, p);
                double n0 = (op.z.x - o.z.x) / eps, n1 = (op.z.y - o.z.y) / eps;
                double e = fmax(fabs(n0 - o.Hf[k]), fabs(n1 - o.Hf[3+k]));
                if (e > maxerr) maxerr = e;
        }
        printf("  Hf  max err %.2e\n", maxerr);
        check("jac: Hf matches finite differences", maxerr < 1e-5);

        check("jac: Hp = -Hf", fabs(o.Hp[0] + o.Hf[0]) < 1e-15 &&
                               fabs(o.Hp[5] + o.Hf[5]) < 1e-15);

        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

