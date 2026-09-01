#include <stdio.h>
#include <math.h>
#include "mat.h"

static int nfail = 0;

static void check(const char *name, int cond) {
        printf("%-32s %s\n", name, cond ? "ok" : "FAIL");
        if (!cond) nfail++;
}

static int meq(mat_t A, mat_t B) {
        if (A.rows != B.rows || A.cols != B.cols) return 0;
        for (size_t i = 0; i < A.rows; i++)
                for (size_t j = 0; j < A.cols; j++)
                        if (fabs(mat_get(A,i,j) - mat_get(B,i,j)) > 1e-12) return 0;
        return 1;
}

int main(void) {
        mat_t A = mat_zero(2,3), B = mat_zero(3,2);
        double av[] = {1,2,3,4,5,6}, bv[] = {7,8,9,10,11,12};
        for (int k = 0; k < 6; k++) { A.d[k] = av[k]; B.d[k] = bv[k]; }

        mat_t AB = mat_mul(A, B);
        mat_t AB_expect = mat_zero(2,2);
        double ev[] = {58,64,139,154};
        for (int k = 0; k < 4; k++) AB_expect.d[k] = ev[k];
        check("mul: hand-computed 2x3 * 3x2", meq(AB, AB_expect));

        check("mul: A*I = A",            meq(mat_mul(A, mat_eye(3)), A));
        check("mul: I*A = A",            meq(mat_mul(mat_eye(2), A), A));
        check("add: A + 0 = A",          meq(mat_add(A, mat_zero(2,3)), A));
        check("transpose: (A^T)^T = A",  meq(mat_transpose(mat_transpose(A)), A));
        check("transpose: (AB)^T = B^T A^T",
              meq(mat_transpose(AB),
                  mat_mul(mat_transpose(B), mat_transpose(A))));
        check("scale: 2 then 0.5 = A",   meq(mat_scale(mat_scale(A, 2.0), 0.5), A));
        check("get/set roundtrip",       (mat_set(&A, 1, 2, 42.0), mat_get(A, 1, 2) == 42.0));

        mat_t A3 = mat_zero(3,3);
        double a3[] = {4,7,2, 3,6,1, 2,5,3};
        for (int k = 0; k < 9; k++) A3.d[k] = a3[k];
        check("inv3: A * inv(A) = I", meq(mat_mul(A3, mat3_inv(A3)), mat_eye(3)));

        printf(nfail ? "FAIL (%d)\n" : "PASS\n", nfail);
        return nfail != 0;
}

