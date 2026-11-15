#ifndef CORE_CMAT_LMAT_H
#define CORE_CMAT_LMAT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
class lMat
{
public:
	lMat();
	lMat(int, int);
	lMat(lMat&);
	lMat(const lMat&);
	~lMat();
public:
	float* v;
	int r,c;
	bool temp;
	float err;
public:
	void Initialization();
	void Init(int r, int c, bool zero = true);
	void Init(const lMat&);
	void Delete();
public:
	bool isTemp() {
		return temp;
	}
	void Temp() {
		temp = true;
	}
	void NoTemp() {
		temp = false;
	}
	void TempRemove() {
		if (temp) {
			Delete();
		}
		NoTemp();
	}
public:
	void E();
	void transpose();
	void diag(float*,int);
	lMat T();
	lMat Inv();
	void Inverse();
public:
	void SVD(lMat&, lMat&, lMat&);
public:
	lMat operator*(lMat);
	lMat operator*(float);
	lMat operator/(float);
	lMat operator+(lMat);
	lMat operator-(lMat);
	void operator=(lMat);
	float& In(int, int);
	float Out(int, int);
};
#endif  // CORE_CMAT_LMAT_H
