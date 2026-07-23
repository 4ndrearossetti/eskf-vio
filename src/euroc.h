#ifndef EUROC_H
#define EUROC_H
#include "imu.h"

size_t euroc_load_imu(const char *path, imu_sample_t **out);
size_t euroc_load_gt(const char *path, gt_sample_t **out);

#endif

