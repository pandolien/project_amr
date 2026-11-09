// 3d 모듈 회귀 테스트: 이식 과정에서 발견된 쿼터니언/const 참조 문제가 재발하지 않는지 확인
#include <cmath>
#include <cstdio>

#include "3d/dhMat.h"
#include "3d/dhVector.h"
#include "3d/dhVertex.h"

namespace {

int g_fail = 0;
const float kEps = 1e-4f;
const float kPi = 3.14159265f;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

bool nearly(float a, float b) { return std::fabs(a - b) <= kEps; }

bool nearlyVec(const dhVector& a, const dhVector& b) {
    return nearly(a.x, b.x) && nearly(a.y, b.y) && nearly(a.z, b.z);
}

bool nearlyMat(const dhMat& a, const dhMat& b) {
    for (int i = 0; i < 16; i++) {
        if (!nearly(a.v[i], b.v[i])) return false;
    }
    return true;
}

void testVector() {
    dhVector a(1, 2, 3), b(1, 1, 1);
    // 임시 객체 연산 (const 참조가 아니면 컴파일 불가)
    check(nearlyVec(a + (b * 2.0f), dhVector(3, 4, 5)), "vector: a + (b*2)");
    check(nearlyVec(a - b, dhVector(0, 1, 2)), "vector: a - b");
    check(nearlyVec(dhVector(1, 0, 0) * dhVector(0, 1, 0), dhVector(0, 0, 1)), "vector: cross x*y=z");
    check(nearly(dhVector(3, 4, 0).Norm(), 5.0f), "vector: Norm");
    check(nearly(dhVector(0, 5, 0).Unit().y, 1.0f), "vector: Unit");
    check(nearly(a.Dot(b), 6.0f), "vector: Dot");
    check(nearly(dhVector(1, 1, 1).Norm(dhVector(1, 1, 4)), 3.0f), "vector: Norm(other)");
}

void testVertex() {
    dhVertex v;
    v.SetPoint(dhVector(1, 2, 3));
    v.AddNrom(dhVector(0, 0, 1));
    v.AddNrom(dhVector(0, 0, 1));
    check(nearlyVec(v.GetPoint(), dhVector(1, 2, 3)), "vertex: point");
    check(nearlyVec(v.GetNormalVector(), dhVector(0, 0, 2)), "vertex: AddNrom");
}

void testBasicTransforms() {
    const float H = kPi / 2;
    dhVector ex(1, 0, 0), ey(0, 1, 0);
    check(nearlyVec(dhMat::RotZ(H) * ex, dhVector(0, 1, 0)), "RotZ(90) * x = y");
    check(nearlyVec(dhMat::RotX(H) * ey, dhVector(0, 0, 1)), "RotX(90) * y = z");
    check(nearlyVec(dhMat::RotY(H) * dhVector(0, 0, 1), dhVector(1, 0, 0)), "RotY(90) * z = x");
    check(nearlyVec(dhMat::Trans(1, 2, 3) * dhVector(1, 1, 1), dhVector(2, 3, 4)), "Trans");
    check(nearlyVec(dhMat::Scale(2, 3, 4) * dhVector(1, 1, 1), dhVector(2, 3, 4)), "Scale");

    // 강체변환의 역행렬
    dhMat m = dhMat::RotZ(0.5236f) * dhMat::Trans(1, 2, 3);
    check(nearlyMat(m * m.Inv(), dhMat()), "Inv: M * Inv(M) = I");
    check(nearlyMat(m.Inv() * m, dhMat()), "Inv: Inv(M) * M = I");
}

void testQuatAxisAngle() {
    // Quat(각도, 축)이 RotX/Y/Z와 같은 행렬이어야 한다 (반각 계산 버그 회귀)
    const float angles[] = {0.3f, 1.2f, 2.0f, 3.0f, -2.5f};
    for (int i = 0; i < 5; i++) {
        float a = angles[i];
        check(nearlyMat(dhMat::Quat(a, dhVector(1, 0, 0)), dhMat::RotX(a)), "Quat(a, x) == RotX(a)");
        check(nearlyMat(dhMat::Quat(a, dhVector(0, 1, 0)), dhMat::RotY(a)), "Quat(a, y) == RotY(a)");
        check(nearlyMat(dhMat::Quat(a, dhVector(0, 0, 1)), dhMat::RotZ(a)), "Quat(a, z) == RotZ(a)");
    }
    // 축은 정규화되지 않아도 된다
    check(nearlyMat(dhMat::Quat(1.2f, dhVector(0, 0, 5)), dhMat::RotZ(1.2f)), "Quat: 비정규화 축");
}

void testQuatFromQuaternion() {
    // Quat(dhQuat)이 전치되어 반대로 회전하던 버그 회귀
    const float H = kPi / 2;
    dhQuat q;
    q.w = std::cos(H / 2);
    q.x = 0;
    q.y = 0;
    q.z = std::sin(H / 2);  // Z축 90도
    check(nearlyVec(dhMat::Quat(q) * dhVector(1, 0, 0), dhVector(0, 1, 0)), "Quat(q_z90) * x = y");
    check(nearlyMat(dhMat::Quat(q), dhMat::RotZ(H)), "Quat(q_z90) == RotZ(90)");
}

void testGetQuatRoundTrip() {
    // 행렬 -> 쿼터니언 -> 행렬. 모든 분기(w/x/y/z 최대)가 실행되도록 큰 각도 포함
    std::printf("  (GetQuat 왕복 케이스 실행)\n");
    const float angles[] = {0.3f, 1.2f, 2.0f, 3.0f, -2.5f};
    for (int i = 0; i < 5; i++) {
        float a = angles[i];
        dhMat rots[3] = {dhMat::RotX(a), dhMat::RotY(a), dhMat::RotZ(a)};
        for (int k = 0; k < 3; k++) {
            check(nearlyMat(dhMat::Quat(rots[k].GetQuat()), rots[k]), "GetQuat 왕복 (단일 축)");
        }
    }
    dhMat h;
    h.Hmatrix(0, 0, 0, 0.7f, -0.4f, 1.9f);
    check(nearlyMat(dhMat::Quat(h.GetQuat()), h), "GetQuat 왕복 (복합 1)");
    h.Hmatrix(0, 0, 0, 2.5f, 1.0f, -2.9f);
    check(nearlyMat(dhMat::Quat(h.GetQuat()), h), "GetQuat 왕복 (복합 2)");
    h.Hmatrix(0, 0, 0, -2.8f, -1.2f, 2.7f);
    check(nearlyMat(dhMat::Quat(h.GetQuat()), h), "GetQuat 왕복 (복합 3)");

    // RotY(60deg) -> w=cos30, y=sin30 (y 성분 인덱스 버그 회귀)
    dhQuat q = dhMat::RotY(1.0471976f).GetQuat();
    check(nearly(q.w, 0.8660254f) && nearly(q.x, 0) && nearly(q.y, 0.5f) && nearly(q.z, 0), "GetQuat(RotY(60))");
}

void testRPY() {
    dhMat h;
    h.Hmatrix(1, 2, 3, 0.5f, 0.3f, -0.8f);
    _RPY p = h.RPY();
    check(nearly(p.a, 0.5f) && nearly(p.b, 0.3f) && nearly(p.r, -0.8f), "RPY 왕복");
    check(nearlyVec(h.O(), dhVector(1, 2, 3)), "Hmatrix: 이동 성분");
}

void testAngleHelpers() {
    check(std::fabs(dhRad(180.0) - dhPI) < 1e-9, "dhRad(180) = pi");
    check(std::fabs(dhDeg(dhPI / 2) - 90.0) < 1e-9, "dhDeg(pi/2) = 90");
}

}  // namespace

int main() {
    testVector();
    testVertex();
    testBasicTransforms();
    testQuatAxisAngle();
    testQuatFromQuaternion();
    testGetQuatRoundTrip();
    testRPY();
    testAngleHelpers();

    if (g_fail) {
        std::printf("test_3d: %d 실패\n", g_fail);
        return 1;
    }
    std::printf("test_3d: 모두 통과\n");
    return 0;
}
