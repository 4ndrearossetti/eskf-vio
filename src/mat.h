#ifndef MAT_H
#define MAT_H
#include <stddef.h>

#define MAT_MAX 80

typedef struct {
        size_t rows, cols;
        double d[MAT_MAX*MAT_MAX];
} mat_t;

mat_t mat_zero(size_t r, size_t c);
mat_t mat_eye(size_t n);
mat_t mat_add(mat_t A, mat_t B);
mat_t mat_mul(mat_t A, mat_t B);
mat_t mat_transpose(mat_t A);
mat_t mat_scale(mat_t A, double s);
double mat_get(mat_t A, size_t i, size_t j);
void mat_set(mat_t *A, size_t i, size_t j, double v);
mat_t mat3_inv(mat_t A);
int mat_chol_solve(mat_t S, mat_t B, mat_t *X);

#endif

