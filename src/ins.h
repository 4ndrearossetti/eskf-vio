#ifndef INS_H
#define INS_H
#include "quat.h"

// Convert one gyro reading (rad/s, body frame) over timestep dt (s)
// into the incremental rotation quaternion for that tick.
quaternion_t gyro_to_q(vector_3d_t w, double dt);

#endif

