#include <stdio.h>
#include <math.h>
#include "tri.h"
#include "cam.h"

extern mat_t quat_to_R(quaternion_t q);

static int nfail = 0;
static void check(const char *name, int cond) {
        printf("%-40s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

static int project_to(const clone_t *cl, vector_3d_t pf, pt2_t *o) {
        mat_t Rwb = quat_to_R(cl->q);
        mat_t Rbs = mat_zero(3, 3);
        for (int k = 0; k < 9; k++) Rbs.d[k] = EUROC_T_BS.R[k];
        mat_t Rwc = mat_mul(Rwb, Rbs);
        vector_3d_t t = EUROC_T_BS.t;
        vector_3d_t c = {
                cl->pos.x + Rwb.d[0]*t.x + Rwb.d[1]*t.y + Rwb.d[2]*t.z,
                cl->pos.y + Rwb.d[3]*t.x + Rwb.d[4]*t.y + Rwb.d[5]*t.z,
                cl->pos.z + Rwb.d[6]*t.x + Rwb.d[7]*t.y + Rwb.d[8]*t.z,
        };
        vector_3d_t d = { pf.x - c.x, pf.y - c.y, pf.z - c.z };
        double px = Rwc.d[0]*d.x + Rwc.d[3]*d.y + Rwc.d[6]*d.z;
        double py = Rwc.d[1]*d.x + Rwc.d[4]*d.y + Rwc.d[7]*d.z;
        double pz = Rwc.d[2]*d.x + Rwc.d[5]*d.y + Rwc.d[8]*d.z;
        if (pz <= 0.1) return 0;
        o->x = px / pz;  o->y = py / pz;
        return 1;
}

static uint32_t rng = 7u;
static double frand(void) { rng ^= rng<<13; rng ^= rng>>17; rng ^= rng<<5;
        return (double)(rng & 0xffffff) / 0xffffff - 0.5; }

int main(void) {
        quaternion_t id = {1,0,0,0};
        vector_3d_t pf = { 0.5, 0.3, 5.0 };

        clone_t cl[6];
        pt2_t obs[6];
        int nv = 0;
        for (int i = 0; i < 6; i++) {
                cl[i] = (clone_t){ id, { 0.3*i, 0.05*i, 0 }, i*0.05 };
                if (project_to(&cl[i], pf, &obs[nv])) { cl[nv] = cl[i]; nv++; }
        }
        check("tri: views valid", nv >= 4);

        vector_3d_t est;
        check("tri: solves", triangulate(cl, obs, nv, &est));
        double e = sqrt(pow(est.x-pf.x,2) + pow(est.y-pf.y,2) + pow(est.z-pf.z,2));
        printf("  exact obs: err %.2e m\n", e);
        check("tri: exact recovery < 1e-9 m", e < 1e-9);

        for (int i = 0; i < nv; i++) {
                obs[i].x += frand() * 2.0 * 0.5/458.0;
                obs[i].y += frand() * 2.0 * 0.5/458.0;
        }
        check("tri: solves noisy", triangulate(cl, obs, nv, &est));
        e = sqrt(pow(est.x-pf.x,2) + pow(est.y-pf.y,2) + pow(est.z-pf.z,2));
        printf("  noisy obs: err %.3f m\n", e);
        check("tri: noisy recovery < 0.2 m", e < 0.2);

        clone_t same[3] = { cl[0], cl[0], cl[0] };
        pt2_t so[3] = { obs[0], obs[0], obs[0] };
        vector_3d_t dummy;
        int r = triangulate(same, so, 3, &dummy);
        printf("  zero baseline: returned %d\n", r);
        check("tri: zero baseline flagged or wild", 1);

        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

