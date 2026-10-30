#ifndef  KALMAN15d_SSMat_H
#define   KALMAN15d_SSMat_H

struct Kalman15d_SSMat
{
	float Phikk_1[15][15];
	float Phik[15][15];
	float Tauk[15][6];
	float Pkk_1[15][15];
	float Pk[15][15];
	float xkk_1[15];
	float xk[15];
	float qk[6][6];
	//double rk[6][6];
	//double Hk[6][19];
};

#endif
