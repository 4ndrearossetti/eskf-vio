#include <math.h>
#include "ins.h"

quaternion_t gyro_to_q(vector_3d_t w, double dt) {
        double norm = sqrt(w.x*w.x + w.y*w.y + w.z*w.z);
        double theta = norm * dt;

        if (theta < 1e-8) {
                quaternion_t dq = { 1.0, w.x*dt/2, w.y*dt/2, w.z*dt/2 };
                return dq;
        }

        vector_3d_t u = { w.x/norm, w.y/norm, w.z/norm };

        double h = theta / 2.0;
        quaternion_t dq = { cos(h), u.x*sin(h), u.y*sin(h), u.z*sin(h) };

        return dq;
}

