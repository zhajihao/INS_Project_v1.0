#pragma once
#ifndef NAVI_MATRIX_H
#define NAVI_MATRIX_H

void mat_multi(double *a, double *b, double *c, int m, int n, int p);
void mat_multi_num(double *a, double b, double *c, int m, int n);
void mat_plus(double *a, double *b, double *c, int m, int n);
void mat_minus(double *a, double *b, double *c, int m, int n);
void mat_assign(double *a, double *b, int m, int n);
void mat_transpose(double *a, double *b, int m, int n);
void mat_unit(double *a, int m);
void mat3d_inverse(double mat1[3][3], double mat2[3][3]);
void mat2d_inverse(double mat1[2][2], double mat2[2][2]);

#endif /* NAVI_MATRIX_H */
