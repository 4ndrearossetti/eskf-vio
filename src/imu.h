#ifndef IMU_H
#define IMU_H
#include <stddef.h>
#include "quat.h"

typedef struct {
        double timestamp;
        vector_3d_t gyro;
        vector_3d_t accel;
} imu_sample_t;

#endif

