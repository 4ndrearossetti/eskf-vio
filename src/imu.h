#ifndef IMU_H
#define IMU_H
#include <stddef.h>

typedef struct {
        double timestamp;
        double gyro[3];
        double accel[3];
} imu_sample_t;

#endif

