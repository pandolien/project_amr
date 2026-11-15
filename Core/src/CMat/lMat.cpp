#include "CMat/lMat.h"

#include <vector>
#define LAPACK_ROW_MAJOR               101
#define LAPACK_COL_MAJOR               102
extern "C" {
	// 헤더(lapacke.h) 없이 직접 선언한다. 라이브러리(LAPACKE 또는 OpenBLAS)만 링크하면 된다.
	int LAPACKE_dgesvd_work(int matrix_order, char jobu, char jobvt,
		int m, int n, double* a,
		int lda, double* s, double* u,
		int ldu, double* vt, int ldvt,
		double* work, int lwork);
}

namespace {

// float 행렬(열 우선, r x c)을 double 로 올려 LAPACK dgesvd 로 특이값 분해한다 (입력은 바뀌지 않는다).
//   full = true : U (r x r), Vt (c x c)      full = false : U (r x p), Vt (p x c), p = min(r, c)
// 결과는 모두 열 우선 double 배열이다. 실패(수렴 실패, 크기 0 등)하면 false.
bool SvdDouble(const float* a, int r, int c, bool full,
	std::vector<double>* U, std::vector<double>* s, std::vector<double>* Vt)
{
	if (a == NULL || r <= 0 || c <= 0) return false;
	const int p = r < c ? r : c;
	const int ucols = full ? r : p;
	const int vrows = full ? c : p;
	const char job = full ? 'A' : 'S';
	const size_t n = (size_t)r * (size_t)c;
	std::vector<double> A(n);
	for (size_t i = 0; i < n; i++) A[i] = (double)a[i];
	s->assign((size_t)p, 0.0);
	U->assign((size_t)r * (size_t)ucols, 0.0);
	Vt->assign((size_t)vrows * (size_t)c, 0.0);

	double query = 0.0;
	int info = LAPACKE_dgesvd_work(LAPACK_COL_MAJOR, job, job, r, c, &A[0], r, &(*s)[0],
		&(*U)[0], r, &(*Vt)[0], vrows, &query, -1);
	if (info != 0) return false;
	int lwork = (int)query;
	if (lwork < 1) lwork = 1;
	std::vector<double> work((size_t)lwork);
	info = LAPACKE_dgesvd_work(LAPACK_COL_MAJOR, job, job, r, c, &A[0], r, &(*s)[0],
		&(*U)[0], r, &(*Vt)[0], vrows, &work[0], lwork);
	return info == 0;
}

}  // namespace
lMat::lMat()
{
	Initialization();
}

lMat::lMat(int r, int c)
{
	Initialization();
	Init(r, c);
}

lMat::lMat(lMat& v)
{
	Initialization();
	if (v.temp) {
		// 임시 객체: 메모리 소유권을 넘겨받는다
		this->v = v.v;
		this->r = v.r;
		this->c = v.c;
		Temp();

		// 원본의 포인터를 NULL로 비워 이중 delete[]를 방지한다
		v.v = NULL;
		v.NoTemp();
	}
	else {
		// 일반 객체: 깊은 복사
		Init((const lMat&)v);
	}
}
lMat::lMat(const lMat& v) { 
	Initialization();
	Init(v);
}

lMat::~lMat()
{
	if (temp)return;
	Delete();
}

void lMat::Initialization()
{
	v = NULL;
	r = 0;
	c = 0;
	err = 0;
	NoTemp();
}

void lMat::Init(int r, int c, bool zero)
{
	if (r == 0 || c == 0) { Delete(); return; }
	if (v && this->r == r && this->c == c) {
		// 크기가 같으면 버퍼를 그대로 재사용한다 (1MB 새로 할당하면 290us)
		if (zero) memset(v, 0, sizeof(float) * r * c);
		return;
	}
	Delete();
	v = new float[r*c];
	if (zero) memset(v, 0, sizeof(float) * r * c);
	this->r = r;
	this->c = c;
}

void lMat::Init(const lMat& v)
{
	Delete();
	if (v.temp) {
		this->v = v.v;
		this->r = v.r;
		this->c = v.c;
		NoTemp();
		return;
	}
	else {
		Init(v.r, v.c, false);
		if (this->v && v.v)
			memmove(this->v, v.v, sizeof(float) * r * c);
		NoTemp();
	}
}

void lMat::Delete()
{
	if (v)delete[](v);
	v = NULL;
	r = 0;
	c = 0;
}

void lMat::E()
{
	int i,n = r;
	if (r > c) n = c;
	for (i = 0; i < n; i++) {
		v[i *r+ i] = 1;
	}
}

void lMat::transpose()
{
	int i, j;
	int nr = this->c, nc = this->r;
	float* nv = new float[nr * nc];
	for (i = 0; i < nc; i++) {
		for (j = 0; j < nr; j++) {
			nv[i * nr + j] = this->v[j * nc + i];
		}
	}
	Delete();
	this->v = nv;
	this->r = nr;
	this->c = nc;
}

void lMat::diag(float*v, int n)
{
	int i;
	Init(n, n);
	for (i = 0; i < n; i++) {
		this->v[i * n + i] = v[i];
	}
}

lMat lMat::T()
{
	lMat out;
	int i, j;
	int nr = this->c, nc = this->r;
	float* v = new float[nr * nc];
	for (i = 0; i < nc; i++) {
		for (j = 0; j < nr; j++) {
			v[i * nr + j] = this->v[j * nc + i];
		}
	}
	out.v = v;
	out.r = nr;
	out.c = nc;
	out.Temp();
	NoTemp();
	return out;
}

// 특이값 분해 A = U * S * Vt. 내부 계산은 double(dgesvd)로 하고 결과를 float 로 돌려준다.
// 입력 A(this)는 바뀌지 않는다. U: r x r, S: p x p 대각, Vt: c x c (p = min(r, c)).
void lMat::SVD(lMat& U, lMat& S, lMat& Vt) {
	std::vector<double> Ud, sd, Vtd;
	const int p = r < c ? r : c;
	if (!SvdDouble(v, r, c, true, &Ud, &sd, &Vtd)) {
		U.Delete();
		Vt.Delete();
		S.Delete();
		NoTemp();
		return;
	}
	U.Init(r, r);
	Vt.Init(c, c);
	for (size_t i = 0; i < Ud.size(); i++) U.v[i] = (float)Ud[i];
	for (size_t i = 0; i < Vtd.size(); i++) Vt.v[i] = (float)Vtd[i];
	std::vector<float> sf((size_t)p);
	for (int i = 0; i < p; i++) sf[(size_t)i] = (float)sd[(size_t)i];
	S.diag(&sf[0], p);
	NoTemp();
}


// 무어-펜로즈 의사역행렬 (정방 가역 행렬이면 보통의 역행렬). 결과는 c x r.
// 분해와 곱셈을 모두 double 로 하고, 마지막에 float 로 한 번만 반올림한다.
// 작은 특이값은 max(r,c) * eps(float) * 최대 특이값 이하이면 0 으로 취급한다
// (입력이 float 라서 그 이하는 데이터의 잡음이다).
lMat lMat::Inv()
{
	lMat out;
	const int r0 = r, c0 = c;
	std::vector<double> U, s, Vt;
	if (!SvdDouble(v, r0, c0, false, &U, &s, &Vt)) { NoTemp(); return out; }	// SVD 실패
	const int p = r0 < c0 ? r0 : c0;
	double smax = 0.0;
	for (int k = 0; k < p; k++) if (s[(size_t)k] > smax) smax = s[(size_t)k];
	const double tol = (double)(r0 > c0 ? r0 : c0) * 1.192092896e-7 * smax;

	out.Init(c0, r0);
	// out(i, j) = sum_k Vt(k, i) / s_k * U(j, k).   Vt: p x c (ld = p),  U: r x p (ld = r)
	for (int j = 0; j < r0; j++) {
		for (int i = 0; i < c0; i++) {
			double sum = 0.0;
			for (int k = 0; k < p; k++) {
				const double sv = s[(size_t)k];
				if (sv > tol) sum += Vt[(size_t)k + (size_t)i * (size_t)p] / sv * U[(size_t)j + (size_t)k * (size_t)r0];
			}
			out.v[(size_t)i + (size_t)j * (size_t)c0] = (float)sum;
		}
	}
	out.Temp();
	NoTemp();
	return out;
}

void lMat::Inverse()
{
	*this = Inv();
	NoTemp();
}

lMat lMat::operator*(lMat v)
{
	lMat out;
	int i, j, k;
	float sum;
	if (this->c != v.r) {
		v.TempRemove();
		NoTemp();
		return out;
	}
	const int m = this->r, K = this->c, n = v.c;
	out.Init(m, n, m >= 8);
	if (m >= 8) {
		for (j = 0; j < n; j++) {
			float* cj = out.v + (size_t)j * m;
			const float* bj = v.v + (size_t)j * K;
			for (k = 0; k + 3 < K; k += 4) {
				const float b0 = bj[k], b1 = bj[k + 1], b2 = bj[k + 2], b3 = bj[k + 3];
				const float* a0 = this->v + (size_t)k * m;
				const float* a1 = a0 + m;
				const float* a2 = a1 + m;
				const float* a3 = a2 + m;
				for (i = 0; i < m; i++)
					cj[i] += a0[i] * b0 + a1[i] * b1 + a2[i] * b2 + a3[i] * b3;
			}
			for (; k < K; k++) {
				const float b = bj[k];
				const float* a = this->v + (size_t)k * m;
				for (i = 0; i < m; i++)
					cj[i] += a[i] * b;
			}
		}
	}
	else {
		for (i = 0; i < this->r; i++) {
			for (j = 0; j < v.c; j++) {
				sum = 0.0;
				for (k = 0; k < this->c; k++) {
					sum += this->v[k * this->r + i] * v.v[j * v.r + k];
				}
				out.v[j * this->r + i] = sum;
			}
		}
	}
	out.Temp();
	v.TempRemove();
	NoTemp();
	return out;
}

lMat lMat::operator*(float other)
{
	lMat out;
	int i;
	out.Init(r, c, false);
	for (i = 0; i < r * c; i++)
		out.v[i] = v[i] * other;
	out.Temp();
	NoTemp();
	return out;
}

lMat lMat::operator/(float other)
{
	lMat out;
	int i;
	out.Init(r, c, false);
	for (i = 0; i < r * c; i++)
		out.v[i] = v[i] / other;
	out.Temp();
	NoTemp();
	return out;
}

lMat lMat::operator+(lMat v)
{
	lMat out;
	int i;
	if (this->r != v.r || this->c != v.c) {
		v.TempRemove();
		NoTemp();
		return out;
	}
	out.Init(r, c, false);
	for (i = 0; i < r * c; i++)
		out.v[i] = this->v[i] + v.v[i];
	out.Temp();
	v.TempRemove();
	NoTemp();
	return out;
}

lMat lMat::operator-(lMat v)
{
	lMat out;
	int i;
	if (this->r != v.r || this->c != v.c) {
		v.TempRemove();
		NoTemp();
		return out;
	}
	out.Init(r, c, false);
	for (i = 0; i < r * c; i++)
		out.v[i] = this->v[i] - v.v[i];
	out.Temp();
	v.TempRemove();
	NoTemp();
	return out;
}

void lMat::operator=(lMat v)
{
	Init(v.r, v.c, false);
	if (this->v && v.v)
		memmove(this->v, v.v, sizeof(float) * r * c);
	NoTemp();
	v.TempRemove();
}

float& lMat::In(int r, int c)
{
	if (this->r <= r) return err;
	if (this->c <= c) return err;
	NoTemp();
	return v[c * this->r + r];
}

float lMat::Out(int r, int c)
{
	if (this->r <= r) return 0.0;
	if (this->c <= c) return 0.0;
	NoTemp();
	return v[c * this->r + r];
}