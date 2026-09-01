#include <assert.h>
#include <math.h>
#include "mat.h"

mat_t mat_zero(size_t r, size_t c) {
        mat_t M = { .rows = r, .cols = c};
        return M;
}

mat_t mat_eye(size_t n) {
        mat_t M = { .rows = n, .cols = n};

        for (size_t i = 0; i < n; i++) {
                M.d[i*n + i] = 1;
        }
        return M;
}

mat_t mat_add(mat_t A, mat_t B) {
        assert(A.rows == B.rows && A.cols == B.cols);

        mat_t M = { .rows = A.rows, A.cols};

        for (size_t i = 0; i < M.rows; i++) {
                for (size_t j = 0; j < M.cols; j++) {
                        M.d[i*M.cols + j] = A.d[i*M.cols + j] + B.d[i*M.cols + j];
                }
        }
        return M;
}

mat_t mat_mul(mat_t A, mat_t B) {
        assert(A.cols == B.rows);

        mat_t M = { .rows = A.rows, .cols = B.cols};

        for (size_t i = 0; i < M.rows; i++) {
                for (size_t j = 0; j < M.cols; j++) {
                        for (size_t k = 0; k < A.cols; k++) {
                                M.d[i*M.cols + j] += A.d[i*A.cols + k] * B.d[k*B.cols + j];
                        }
                }
        }
        return M;
}

mat_t mat_transpose(mat_t A) {
        mat_t M = { .rows = A.cols, .cols = A.rows};

        for (size_t i = 0; i < M.rows; i++) {
                for (size_t j = 0; j < M.cols; j++) {
                        M.d[i*M.cols + j] = A.d[j*A.cols + i];
                }
        }
        return M;
}

mat_t mat_scale(mat_t A, double s) {
        mat_t M = { .rows = A.rows, .cols = A.cols};

        for (size_t i = 0; i < M.rows; i++) {
                for (size_t j = 0; j < M.cols; j++) {
                        M.d[i*M.cols + j] = A.d[i*A.cols + j] * s;
                }
        }
        return M;
}

double mat_get(mat_t A, size_t i, size_t j) {
        assert(i < A.rows && j < A.cols);

        return A.d[i*A.cols + j];
}

void mat_set(mat_t *A, size_t i, size_t j, double v) {
        assert(i < A->rows && j < A->cols);
        A->d[i*A->cols + j] = v;
}

mat_t mat3_inv(mat_t A) {
        assert(A.rows == 3 && A.cols == 3);
        double a = A.d[0], b = A.d[1], c = A.d[2],
               d = A.d[3], e = A.d[4], f = A.d[5],
               g = A.d[6], h = A.d[7], i = A.d[8];
        double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
        assert(fabs(det) > 1e-300);
        double s = 1.0 / det;
        mat_t M = mat_zero(3, 3);
        M.d[0] =  (e*i - f*h)*s;  M.d[1] = -(b*i - c*h)*s;  M.d[2] =  (b*f - c*e)*s;
        M.d[3] = -(d*i - f*g)*s;  M.d[4] =  (a*i - c*g)*s;  M.d[5] = -(a*f - c*d)*s;
        M.d[6] =  (d*h - e*g)*s;  M.d[7] = -(a*h - b*g)*s;  M.d[8] =  (a*e - b*d)*s;
        return M;
}

