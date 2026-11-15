// CMat(lMat) 테스트: 곱셈/전치/복사/SVD/Inv 의 정확성 확인
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <vector>

#include "CMat/lMat.h"

namespace {

int g_fail = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

bool nearly(double a, double b, double eps = 1e-3) { return std::fabs(a - b) <= eps; }

// 결정적 의사난수 [-1, 1)
float rnd(unsigned& s) {
    s = s * 1664525u + 1013904223u;
    return static_cast<float>((s >> 8) & 0xFFFF) / 32768.0f - 1.0f;
}

void randomize(lMat& m, int r, int c, unsigned seed) {
    m.Init(r, c);
    unsigned s = seed;
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++) m.In(i, j) = rnd(s);
}

// 행 우선 배열로 채운다 (lMat 내부는 열 우선)
void fill(lMat& m, int r, int c, const float* a) {
    m.Init(r, c);
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++) m.In(i, j) = a[i * c + j];
}

bool nearlyMat(lMat& a, lMat& b, double eps = 1e-3) {
    if (!a.v || !b.v || a.r != b.r || a.c != b.c) return false;
    for (int i = 0; i < a.r; i++)
        for (int j = 0; j < a.c; j++)
            if (!nearly(a.Out(i, j), b.Out(i, j), eps)) return false;
    return true;
}

bool isIdentity(lMat& m, double eps = 1e-3) {
    if (!m.v || m.r != m.c) return false;
    for (int i = 0; i < m.r; i++)
        for (int j = 0; j < m.c; j++)
            if (!nearly(m.Out(i, j), i == j ? 1.0 : 0.0, eps)) return false;
    return true;
}

// 기준 곱셈 (3중 루프, double 누적)
void naiveMul(lMat& A, lMat& B, lMat& C) {
    C.Init(A.r, B.c);
    for (int i = 0; i < A.r; i++) {
        for (int j = 0; j < B.c; j++) {
            double s = 0;
            for (int k = 0; k < A.c; k++) s += static_cast<double>(A.Out(i, k)) * B.Out(k, j);
            C.In(i, j) = static_cast<float>(s);
        }
    }
}

void testBasic() {
    lMat m(2, 3);
    check(m.r == 2 && m.c == 3, "생성: 크기");
    check(nearly(m.Out(1, 2), 0), "생성: 0으로 초기화");
    m.In(1, 2) = 7;
    check(nearly(m.Out(1, 2), 7), "In/Out");
    check(nearly(m.Out(5, 5), 0), "Out: 범위 밖은 0");

    lMat I;
    I.Init(3, 3);
    I.E();
    check(isIdentity(I), "E(): 단위행렬");

    const float d[3] = {1, 2, 3};
    lMat D;
    D.diag(const_cast<float*>(d), 3);
    check(nearly(D.Out(0, 0), 1) && nearly(D.Out(1, 1), 2) && nearly(D.Out(2, 2), 3) && nearly(D.Out(0, 1), 0),
          "diag()");

    const float a[6] = {1, 2, 3, 4, 5, 6};
    lMat A, At, B;
    fill(A, 2, 3, a);
    At = A.T();
    const float t[6] = {1, 4, 2, 5, 3, 6};
    fill(B, 3, 2, t);
    check(nearlyMat(At, B), "T(): 전치");
    check(nearlyMat(A, A), "T() 호출 후 원본 유지");
    A.transpose();
    check(nearlyMat(A, B), "transpose(): 제자리 전치");
}

void testMul() {
    // m < 8 (단순 경로) 와 m >= 8 (4열 블록 경로, K 나머지 포함) 모두 검증
    const int shapes[][3] = {{2, 3, 2}, {3, 3, 3}, {7, 5, 4}, {8, 8, 8}, {9, 5, 7}, {13, 4, 1}, {16, 16, 16}, {10, 9, 3}};
    for (unsigned i = 0; i < sizeof(shapes) / sizeof(shapes[0]); i++) {
        int m = shapes[i][0], k = shapes[i][1], n = shapes[i][2];
        lMat A, B, C, R;
        randomize(A, m, k, 100 + i);
        randomize(B, k, n, 200 + i);
        C = A * B;
        naiveMul(A, B, R);
        char name[64];
        std::snprintf(name, sizeof(name), "곱셈 (%dx%d)*(%dx%d)", m, k, k, n);
        check(nearlyMat(C, R), name);
    }
    lMat X(2, 3), Y(2, 3), Z;
    Z = X * Y;
    check(Z.v == NULL && Z.r == 0 && Z.c == 0, "곱셈: 크기 불일치는 빈 행렬");
}

void testAddSubScalar() {
    lMat A, B, C, R;
    randomize(A, 3, 4, 1);
    randomize(B, 3, 4, 2);
    C = A + B;
    R.Init(3, 4);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++) R.In(i, j) = A.Out(i, j) + B.Out(i, j);
    check(nearlyMat(C, R), "덧셈");
    C = A - B;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++) R.In(i, j) = A.Out(i, j) - B.Out(i, j);
    check(nearlyMat(C, R), "뺄셈");
    C = A * 2.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++) R.In(i, j) = A.Out(i, j) * 2.0f;
    check(nearlyMat(C, R), "스칼라 곱");
    C = A / 4.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++) R.In(i, j) = A.Out(i, j) / 4.0f;
    check(nearlyMat(C, R), "스칼라 나눗셈");

    lMat X(2, 2), Y(3, 3), Z;
    Z = X + Y;
    check(Z.v == NULL, "덧셈: 크기 불일치는 빈 행렬");
}

void testCopy() {
    lMat A, B;
    randomize(A, 3, 3, 5);
    B = A;
    check(nearlyMat(A, B), "대입: 값 복사");
    B.In(0, 0) += 10;
    check(!nearly(A.Out(0, 0), B.Out(0, 0)), "대입: 깊은 복사 (원본 불변)");

    lMat C(A);
    check(nearlyMat(A, C), "복사 생성자: 값 복사");
    C.In(1, 1) += 10;
    check(!nearly(A.Out(1, 1), C.Out(1, 1)), "복사 생성자: 깊은 복사");

    lMat S;
    randomize(S, 3, 3, 7);
    lMat S0;
    S0 = S;
    S = S;
    check(nearlyMat(S, S0), "자기 대입");
}

void testSVDShape(int r, int c, unsigned seed) {
    char name[80];
    lMat A, A0, U, S, Vt;
    randomize(A, r, c, seed);
    A0 = A;  // SVD는 A를 덮어쓰므로 사본을 보관
    A.SVD(U, S, Vt);

    const int p = r < c ? r : c;
    std::snprintf(name, sizeof(name), "SVD %dx%d: 결과 크기 (U rxr, S pxp, Vt cxc)", r, c);
    check(U.r == r && U.c == r && Vt.r == c && Vt.c == c && S.r == p && S.c == p, name);
    if (!U.v || !S.v || !Vt.v) {
        check(false, "SVD: 결과가 비어 있음");
        return;
    }

    lMat R(r, c);
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            double sum = 0;
            for (int k = 0; k < p; k++) sum += static_cast<double>(U.Out(i, k)) * S.Out(k, k) * Vt.Out(k, j);
            R.In(i, j) = static_cast<float>(sum);
        }
    }
    std::snprintf(name, sizeof(name), "SVD %dx%d: U*S*Vt 재구성", r, c);
    check(nearlyMat(R, A0), name);

    bool sorted = true;
    for (int k = 0; k < p; k++) {
        if (S.Out(k, k) < 0) sorted = false;
        if (k > 0 && S.Out(k, k) > S.Out(k - 1, k - 1) + 1e-5f) sorted = false;
    }
    std::snprintf(name, sizeof(name), "SVD %dx%d: 특이값 비음수/내림차순", r, c);
    check(sorted, name);

    bool orth = true;
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < r; j++) {
            double d = 0;
            for (int k = 0; k < r; k++) d += static_cast<double>(U.Out(k, i)) * U.Out(k, j);
            if (!nearly(d, i == j ? 1.0 : 0.0)) orth = false;
        }
    }
    for (int i = 0; i < c; i++) {
        for (int j = 0; j < c; j++) {
            double d = 0;
            for (int k = 0; k < c; k++) d += static_cast<double>(Vt.Out(i, k)) * Vt.Out(j, k);
            if (!nearly(d, i == j ? 1.0 : 0.0)) orth = false;
        }
    }
    std::snprintf(name, sizeof(name), "SVD %dx%d: U, Vt 직교성", r, c);
    check(orth, name);
}

// ---- 정밀도: 조건수가 큰 행렬(약 1e5)의 역행렬이 double 기준 참값에 가까운가 ----
// A = Q1 * D * Q2 (Givens 회전 곱, 계산은 double) 를 float 로 반올림한 행렬의 "정확한" 역행렬을
// double 가우스-조던으로 구해 비교한다. float 로 SVD 를 하면 오차가 1e-2 수준까지 커진다.
void givens(std::vector<double>& M, int n, int i, int j, double th) {  // 왼쪽에서 곱함 (열 우선 n x n)
    const double c = std::cos(th), s = std::sin(th);
    for (int col = 0; col < n; col++) {
        const double a = M[(size_t)i + (size_t)col * n], b = M[(size_t)j + (size_t)col * n];
        M[(size_t)i + (size_t)col * n] = c * a - s * b;
        M[(size_t)j + (size_t)col * n] = s * a + c * b;
    }
}

void testInvPrecision() {
    const int n = 6;
    const double d[n] = {1.0, 0.3, 0.1, 1e-2, 1e-3, 1e-5};
    std::vector<double> A((size_t)n * n, 0.0);
    for (int i = 0; i < n; i++) A[(size_t)i + (size_t)i * n] = d[i];
    // 서로 다른 회전을 왼쪽에서 여러 번 곱하고, 전치를 이용해 오른쪽에서도 섞는다
    for (int k = 0; k < 12; k++) givens(A, n, k % n, (k * 5 + 1) % n == k % n ? (k + 1) % n : (k * 5 + 1) % n, 0.4 + 0.37 * k);
    std::vector<double> At((size_t)n * n);
    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) At[(size_t)j + (size_t)i * n] = A[(size_t)i + (size_t)j * n];
    for (int k = 0; k < 12; k++) givens(At, n, (k + 2) % n, (k * 3 + 4) % n == (k + 2) % n ? (k + 3) % n : (k * 3 + 4) % n, 0.9 + 0.23 * k);
    for (int i = 0; i < n; i++) for (int j = 0; j < n; j++) A[(size_t)i + (size_t)j * n] = At[(size_t)j + (size_t)i * n];

    lMat M;
    M.Init(n, n);
    std::vector<double> Af((size_t)n * n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            M.In(i, j) = (float)A[(size_t)i + (size_t)j * n];
            Af[(size_t)i + (size_t)j * n] = (double)M.In(i, j);  // float 로 반올림된 값이 "정확한 입력"
        }

    // 참값: double 가우스-조던 (부분 피벗)
    std::vector<double> W = Af, X((size_t)n * n, 0.0);
    for (int i = 0; i < n; i++) X[(size_t)i + (size_t)i * n] = 1.0;
    for (int col = 0; col < n; col++) {
        int piv = col;
        for (int r = col + 1; r < n; r++) if (std::fabs(W[(size_t)r + (size_t)col * n]) > std::fabs(W[(size_t)piv + (size_t)col * n])) piv = r;
        for (int c2 = 0; c2 < n; c2++) {
            std::swap(W[(size_t)col + (size_t)c2 * n], W[(size_t)piv + (size_t)c2 * n]);
            std::swap(X[(size_t)col + (size_t)c2 * n], X[(size_t)piv + (size_t)c2 * n]);
        }
        const double pv = W[(size_t)col + (size_t)col * n];
        for (int c2 = 0; c2 < n; c2++) { W[(size_t)col + (size_t)c2 * n] /= pv; X[(size_t)col + (size_t)c2 * n] /= pv; }
        for (int r = 0; r < n; r++) {
            if (r == col) continue;
            const double f = W[(size_t)r + (size_t)col * n];
            for (int c2 = 0; c2 < n; c2++) {
                W[(size_t)r + (size_t)c2 * n] -= f * W[(size_t)col + (size_t)c2 * n];
                X[(size_t)r + (size_t)c2 * n] -= f * X[(size_t)col + (size_t)c2 * n];
            }
        }
    }
    double xmax = 0;
    for (size_t i = 0; i < X.size(); i++) xmax = std::max(xmax, std::fabs(X[i]));

    lMat Inv = M.Inv();
    check(Inv.r == n && Inv.c == n && Inv.v != NULL, "Inv 정밀도: 결과 크기");
    if (Inv.v == NULL) return;
    double err = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) err = std::max(err, std::fabs((double)Inv.Out(i, j) - X[(size_t)i + (size_t)j * n]));
    std::printf("  Inv 정밀도(조건수 ~1e5): 최대 오차 / 최대 원소 = %.3e\n", err / xmax);
    check(err / xmax < 1e-5, "Inv 정밀도: 조건수 1e5 행렬의 역행렬 상대오차 < 1e-5");

    // SVD() 는 입력을 바꾸지 않는다
    lMat Mcopy;
    Mcopy = M;
    lMat U, S, Vt;
    M.SVD(U, S, Vt);
    bool same = true;
    for (int i = 0; i < n * n; i++) if (M.v[i] != Mcopy.v[i]) same = false;
    check(same, "SVD 는 입력을 덮어쓰지 않는다");
}

void testSVD() {
    testSVDShape(4, 4, 11);
    testSVDShape(5, 5, 12);
    testSVDShape(6, 3, 13);
    testSVDShape(3, 6, 14);
    testSVDShape(10, 7, 15);
}

void testInv() {
    // 정방 (대각 우세로 조건수 양호)
    lMat A, A0, Ai, P;
    randomize(A, 4, 4, 21);
    for (int i = 0; i < 4; i++) A.In(i, i) += 3.0f;
    A0 = A;
    Ai = A.Inv();
    check(Ai.v != NULL && Ai.r == 4 && Ai.c == 4, "Inv 4x4: 결과 크기");
    P = A * Ai;
    check(isIdentity(P), "Inv 4x4: A * Inv(A) = I");
    P = Ai * A;
    check(isIdentity(P), "Inv 4x4: Inv(A) * A = I");
    check(nearlyMat(A, A0), "Inv: 원본 행렬 불변");

    // 열 전체 계수 5x3: 좌역원 (pinv * A = I3)
    lMat T, Tp;
    randomize(T, 5, 3, 22);
    Tp = T.Inv();
    check(Tp.v != NULL && Tp.r == 3 && Tp.c == 5, "Inv 5x3: 결과 크기 3x5");
    P = Tp * T;
    check(isIdentity(P), "Inv 5x3: pinv * A = I");

    // 행 전체 계수 3x5: 우역원 (A * pinv = I3)
    lMat W, Wp;
    randomize(W, 3, 5, 23);
    Wp = W.Inv();
    check(Wp.v != NULL && Wp.r == 5 && Wp.c == 3, "Inv 3x5: 결과 크기 5x3");
    P = W * Wp;
    check(isIdentity(P), "Inv 3x5: A * pinv = I");

    // 특이행렬: [[1,2],[2,4]] 의 유사역행렬은 A/25
    const float s[4] = {1, 2, 2, 4};
    const float e[4] = {0.04f, 0.08f, 0.08f, 0.16f};
    lMat Sg, Sp, E;
    fill(Sg, 2, 2, s);
    fill(E, 2, 2, e);
    Sp = Sg.Inv();
    check(nearlyMat(Sp, E), "Inv 특이행렬: 유사역행렬 A/25");

    // Inverse(): 제자리 역행렬
    lMat M;
    M = A0;
    M.Inverse();
    P = M * A0;
    check(isIdentity(P), "Inverse(): 제자리 역행렬");
}

}  // namespace

int main() {
    testBasic();
    testMul();
    testAddSubScalar();
    testCopy();
    testSVD();
    testInvPrecision();
    testInv();

    if (g_fail) {
        std::printf("test_cmat: %d 실패\n", g_fail);
        return 1;
    }
    std::printf("test_cmat: 모두 통과\n");
    return 0;
}
