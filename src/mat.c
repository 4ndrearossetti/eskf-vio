#include <assert.h>
#include "mat.h"

mat_t mat_zero(size_t r, size_t c) {
        mat_t M = {r, c};
        return M;
}

mat_t mat_eye(size_t n) {
        mat_t M = {n, n};

        for (size_t i = 0; i < n; i++) {
                M.d[i*n + i] = 1;
        }
        return M;
}

mat_t mat_add(mat_t A, mat_t B) {
        assert(A.rows == B.rows && A.cols == B.cols);

        mat_t M = {A.rows, A.cols};

        for (size_t i = 0; i < M.rows; i++) {
                for (size_t j = 0; j < M.cols; j++) {
                        M.d[i*M.cols + j] = A.d[i*M.cols + j] + B.d[i*M.cols + j];
                }
        }
        return M;
}

mat_t mat_mul(mat_t A, mat_t B) {
        assert(A.cols == B.rows);

        mat_t M = {A.rows, B.cols};

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
        mat_t M = {A.cols, A.rows};

        for (size_t i = 0; i < M.rows; i++) {
                for (size_t j = 0; j < M.cols; j++) {
                        M.d[i*M.cols + j] = A.d[j*A.cols + i];
                }
        }
        return M;
}

mat_t mat_scale(mat_t A, double s) {
        mat_t M = {A.rows, A.cols};

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

