#pragma once
#ifndef NAVI_ATTITUDE_H
#define NAVI_ATTITUDE_H

void a2mat(double att[3], double cnb[3][3]);
void m2att(double cnb[3][3], double att[3]);
void rv2mat(double v[3], double mat[3][3]);

#endif /* NAVI_ATTITUDE_H */
