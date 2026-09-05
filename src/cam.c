#include "cam.h"

const cam_t EUROC_CAM0 = {
        .fu = 458.654, .fv = 457.296, .cu = 367.215, .cv = 248.375,
        .k1 = -0.28340811, .k2 = 0.07395907, .p1 = 0.00019359, .p2 = 1.76187114e-05,
};

const extrinsic_t EUROC_T_BS = {
        .R = { 0.0148655429818, -0.999880929698,  0.00414029679422,
               0.999557249008,   0.0149672133247, 0.025715529948,
              -0.0257744366974,  0.00375618835797, 0.999660727178 },
        .t = { -0.0216401454975, -0.064676986768, 0.00981073058949 },
};

int cam_project(const cam_t *c, vector_3d_t p, double *u, double *v) {
        if (p.z <= 0) return 0;
        double x = p.x / p.z, y = p.y / p.z;
        double r2 = x*x + y*y;
        double rad = 1 + c->k1*r2 + c->k2*r2*r2;
        double xd = x*rad + 2*c->p1*x*y + c->p2*(r2 + 2*x*x);
        double yd = y*rad + c->p1*(r2 + 2*y*y) + 2*c->p2*x*y;
        *u = c->fu*xd + c->cu;
        *v = c->fv*yd + c->cv;
        return 1;
}

vector_3d_t cam_unproject(const cam_t *c, double u, double v) {
        double xd = (u - c->cu) / c->fu;
        double yd = (v - c->cv) / c->fv;
        double x = xd, y = yd;
        for (int i = 0; i < 30; i++) {
                double r2  = x*x + y*y;
                double rad = 1 + c->k1*r2 + c->k2*r2*r2;
                double dx  = 2*c->p1*x*y + c->p2*(r2 + 2*x*x);
                double dy  = c->p1*(r2 + 2*y*y) + 2*c->p2*x*y;
                x = (xd - dx) / rad;
                y = (yd - dy) / rad;
        }
        return (vector_3d_t){ x, y, 1.0 };
}

