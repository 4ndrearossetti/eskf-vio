#ifndef EUROC_H
#define EUROC_H
#include "imu.h"

// Loads EuRoC imu0 CSV. Returns sample count, or 0 on error.
// Caller owns *out and frees it. On error, *out is untouched.
size_t euroc_load_imu(const char *path, imu_sample_t **out);

#endif

