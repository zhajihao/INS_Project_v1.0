#pragma once
#ifndef NAVI_VECTOR_H
#define NAVI_VECTOR_H

void vect3d_askew(double v[3], double mat[3][3]);
void vect3d_cross_vect3d(double v1[3], double v2[3], double v3[3]);
void vect3d_assign(double a0, double a1, double a2, double v[3]);
double norm_3d(double v[3]);

#endif /* NAVI_VECTOR_H */
