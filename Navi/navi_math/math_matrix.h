
#ifndef MATH_MATRIX_H
#define MATH_MATRIX_H

void vect3d_askew(double v[3], double mat[3][3]);
void vect3d_cross_vect3d(double v1[3], double v2[3], double v3[3]);
void vect3d_assign(double a0, double a1, double a2, double v[3]);
void mat_multi(double *a, double *b, double *c, int m, int n, int p);
void mat_multi_f(const float *__restrict a, const float *__restrict b, float *__restrict c, int m, int n, int p);
void mat_plus_f(const float *__restrict a, const float *__restrict b, float *__restrict c, int m, int n);
void mat_minus_f(const float *__restrict a, const float *__restrict b, float *__restrict c, int m, int n);
void mat_assign_f(const float *__restrict a, float *__restrict b, int m, int n);
void mat_transpose_f(const float *__restrict a, float *__restrict b, int m, int n);
void mat_unit_f(float *__restrict a, int m);
void mat_multi_num(double *a, double b, double *c, int m, int n);
void mat_plus(double *a, double *b, double *c, int m, int n);
void mat_minus(double *a, double *b, double *c, int m, int n);
void mat_assign(double *a, double *b, int m, int n);
void mat_transpose(double *a, double *b, int m, int n);
void mat_unit(double *a, int m);
void mat3d_inverse(double mat1[3][3], double mat2[3][3]);
void mat2d_inverse(double mat1[2][2], double mat2[2][2]);
int mat_inverse(int n, float *A, float *A_inv);

#endif
