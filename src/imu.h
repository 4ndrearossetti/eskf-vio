#ifndef IMU_H
#define IMU_H
#include <stddef.h>
#include "quat.h"

typedef struct {
        double timestamp;
        vector_3d_t gyro;
        vector_3d_t accel;
} imu_sample_t;

typedef struct {
        double timestamp;       // s
        vector_3d_t pos;        // m, world frame
        quaternion_t q;         // body→world, stored (w,x,y,z), matches csv column order
        vector_3d_t vel;        // m/s, world frame
        vector_3d_t gyro_bias;  // rad/s
        vector_3d_t accel_bias; // m/s^2
} gt_sample_t;

#endif

