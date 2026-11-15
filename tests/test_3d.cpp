// 3d 모듈 회귀 테스트: 이식 과정에서 발견된 쿼터니언/const 참조 문제가 재발하지 않는지 확인
#include "CBase/Math.h"
#include <stdio.h>

#include "3d/Units.h"
#include "3d/dhEuler.h"
#include "3d/dhMat.h"
#include "3d/dhPose.h"
#include "3d/dhQuat.h"
#include "3d/dhVector.h"
#include "3d/dhVertex.h"

namespace {

int g_fail = 0;
const float kEps = 1e-4f;
const float kPi = 3.14159265f;

void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

bool nearly(float a, float b) { return amr::Abs(a - b) <= kEps; }

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
    q.w = amr::Cos(H / 2);
    q.x = 0;
    q.y = 0;
    q.z = amr::Sin(H / 2);  // Z축 90도
    check(nearlyVec(dhMat::Quat(q) * dhVector(1, 0, 0), dhVector(0, 1, 0)), "Quat(q_z90) * x = y");
    check(nearlyMat(dhMat::Quat(q), dhMat::RotZ(H)), "Quat(q_z90) == RotZ(90)");
}

void testGetQuatRoundTrip() {
    // 행렬 -> 쿼터니언 -> 행렬. 모든 분기(w/x/y/z 최대)가 실행되도록 큰 각도 포함
    printf("  (GetQuat 왕복 케이스 실행)\n");
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
    dhEuler p = h.RPY();
    check(nearly(p.a, 0.5f) && nearly(p.b, 0.3f) && nearly(p.r, -0.8f), "RPY 왕복");
    check(nearlyVec(h.O(), dhVector(1, 2, 3)), "Hmatrix: 이동 성분");
}

void testAngleHelpers() {
    check(amr::Abs(dhRad(180.0) - dhPI) < 1e-9, "dhRad(180) = pi");
    check(amr::Abs(dhDeg(dhPI / 2) - 90.0) < 1e-9, "dhDeg(pi/2) = 90");
}

void testVertexExtras() {
    dhVertex a, b;
    a.SetPoint(1, 2, 3);
    b.SetPoint(4, 6, 3);
    check(nearly(a.distance(&b), 5.0f), "vertex: distance");
    check(nearly(a.at(0), 1) && nearly(a.at(1), 2) && nearly(a.at(2), 3), "vertex: at()");
    check(nearly(a.at(3), 0) && nearly(a.at(-1), 0), "vertex: at() 범위 밖은 0");
    check(nearly(a[1], 2), "vertex: operator[]");
}

void testQuatClass() {
    const float H = kPi / 2;
    dhQuat id;
    check(nearly(id.w, 1) && nearly(id.x, 0) && nearly(id.y, 0) && nearly(id.z, 0), "dhQuat: 기본 생성자는 항등");
    check(nearlyMat(id.GetMat(), dhMat()), "dhQuat: 항등 -> 단위행렬");

    // 생성자 인자 순서는 (x, y, z, w)
    dhQuat q(0, 0, amr::Sin(H / 2), amr::Cos(H / 2));
    check(nearly(q.x, 0) && nearly(q.z, amr::Sin(H / 2)) && nearly(q.w, amr::Cos(H / 2)), "dhQuat: 생성자 인자 순서 (x,y,z,w)");
    check(nearlyVec(q.GetMat() * dhVector(1, 0, 0), dhVector(0, 1, 0)), "dhQuat::GetMat: Z축 90도 (역방향 아님)");
    check(nearlyMat(q.GetMat(), dhMat::RotZ(H)), "dhQuat::GetMat == RotZ(90)");
    dhEuler e = q.GetEuler();
    check(nearly(e.a, H) && nearly(e.b, 0) && nearly(e.r, 0), "dhQuat::GetEuler: Z축 90도");
}

void testEulerClass() {
    dhEuler z;
    check(nearly(z.a, 0) && nearly(z.b, 0) && nearly(z.r, 0), "dhEuler: 기본 생성자");
    check(nearlyMat(z.GetMat(), dhMat()), "dhEuler: 0 -> 단위행렬");

    // b 는 gimbal lock(±90도) 근처를 피한다
    const float abr[][3] = {{0.5f, 0.3f, -0.8f}, {-1.2f, 0.9f, 2.0f}, {2.8f, -1.2f, -2.7f}, {0.1f, 1.3f, 0.4f}};
    for (int i = 0; i < 4; i++) {
        dhEuler e(abr[i][0], abr[i][1], abr[i][2]);
        dhMat h;
        h.Hmatrix(0, 0, 0, e.a, e.b, e.r);
        check(nearlyMat(e.GetMat(), h), "dhEuler::GetMat == Hmatrix");
        check(nearlyMat(e.GetMat(), dhMat::RotZ(e.a) * dhMat::RotY(e.b) * dhMat::RotX(e.r)), "dhEuler::GetMat == Rz*Ry*Rx");
        check(nearlyMat(e.GetQuat().GetMat(), e.GetMat()), "dhEuler::GetQuat 왕복 (행렬)");
        dhEuler back = e.GetQuat().GetEuler();
        check(nearly(back.a, e.a) && nearly(back.b, e.b) && nearly(back.r, e.r), "dhEuler -> dhQuat -> dhEuler 왕복");
    }
}

}  // namespace

// ---- 쿼터니언 연산, Pose, 단위
void testQuatOps() {
    const dhVector ax(1, 2, 3);
    const dhQuat q = dhQuat::FromAxisAngle(0.8f, ax);
    check(nearly(q.Norm(), 1.0f), "FromAxisAngle gives a unit quaternion");
    check(nearlyMat(q.GetMat(), dhMat::Quat(0.8f, ax)), "FromAxisAngle == dhMat::Quat");
    check(nearly(dhQuat::FromAxisAngle(1.0f, dhVector(0, 0, 0)).w, 1.0f), "zero axis -> identity");

    const dhVector v(0.3f, -1.2f, 2.0f);
    check(nearlyVec(q.Rotate(v), q.GetMat() * dhVector(0, 0, 0) + dhVector(0, 0, 0) + (q.GetMat() * v)), "Rotate == matrix * v (rotation only)");
    const dhQuat a = dhQuat::FromAxisAngle(0.5f, dhVector(1, 0, 0)), b = dhQuat::FromAxisAngle(1.1f, dhVector(0, 1, 1));
    check(nearlyVec((a * b).Rotate(v), a.Rotate(b.Rotate(v))), "(a*b) applies b first, then a");
    check(nearlyMat((a * b).GetMat(), a.GetMat() * b.GetMat()), "quaternion product == matrix product");
    check(nearlyVec(q.Conjugate().Rotate(q.Rotate(v)), v), "Conjugate is the inverse rotation");
    check(nearly(dhQuat(0, 0, 0, 2).Normalized().w, 1.0f), "Normalized");
    check(nearly(dhQuat(0, 0, 0, 0).Normalized().w, 1.0f), "zero quaternion normalizes to identity");
    check(nearly(dhQuat(NAN, 0, 0, 1).Normalized().w, 1.0f), "NaN quaternion normalizes to identity");
}

void testPose() {
    const dhPose p(dhVector(1, 2, 3), dhQuat::FromAxisAngle(0.7f, dhVector(0, 0, 1)));
    check(nearlyMat(p.ToMat(), dhMat::Trans(1, 2, 3) * dhMat::RotZ(0.7f)), "Pose::ToMat == Trans * Rot");
    const dhPose back = dhPose::FromMat(p.ToMat());
    check(nearlyVec(back.position, p.position) && nearly(amr::Abs(back.orientation.Dot(p.orientation)), 1.0f), "FromMat round trip");

    const dhPose q(dhVector(-0.5f, 0.2f, 1.0f), dhQuat::FromAxisAngle(1.3f, dhVector(1, 1, 0)));
    check(nearlyMat((p * q).ToMat(), p.ToMat() * q.ToMat()), "Pose composition == matrix product");
    const dhVector pt(0.4f, 0.1f, -0.7f);
    check(nearlyVec(p * pt, p.ToMat() * pt), "Pose * point");
    check(nearlyMat((p * p.Inv()).ToMat(), dhMat()), "pose * inverse = identity");
    check(nearlyVec((p * q).Inv() * ((p * q) * pt), pt), "inverse of a composition");

    // 전송 형식: x y z qx qy qz qw, 정규화, w >= 0
    double w[7];
    dhPose(dhVector(1, 2, 3), dhQuat(0, 0, 0.5f * 2, -2)).ToArray(w);  // (0,0,1,-2) 를 정규화하면 w < 0 -> 부호 반전
    check(w[0] == 1 && w[1] == 2 && w[2] == 3, "ToArray: position (m) first");
    check(w[6] > 0 && nearly(static_cast<float>(w[3] * w[3] + w[4] * w[4] + w[5] * w[5] + w[6] * w[6]), 1.0f), "ToArray: unit quaternion with w >= 0");
    check(nearly(static_cast<float>(w[5]), -0.4472136f) && nearly(static_cast<float>(w[6]), 0.8944272f), "ToArray: order is x y z w");
    dhPose r;
    check(dhPose::FromArray(w, &r) && nearlyVec(r.position, dhVector(1, 2, 3)), "FromArray");
    check(nearly(r.orientation.z, -0.4472136f) && nearly(r.orientation.w, 0.8944272f), "FromArray: x y z w order");
    double bad[7] = {0, 0, 0, 0, 0, 0, 0};
    check(!dhPose::FromArray(bad, &r), "FromArray rejects a zero quaternion");
    double nanv[7] = {0, 0, NAN, 0, 0, 0, 1};
    check(!dhPose::FromArray(nanv, &r), "FromArray rejects NaN");
    double unnorm[7] = {0, 0, 0, 0, 0, 3, 4};
    check(dhPose::FromArray(unnorm, &r) && nearly(r.orientation.Norm(), 1.0f), "FromArray normalizes");
}

void testUnits() {
    check(amr::Abs(amr::units::DegToRad(180.0) - 3.14159265358979323846) < 1e-12, "DegToRad(180) = pi");
    check(amr::Abs(amr::units::RadToDeg(3.14159265358979323846 / 2) - 90.0) < 1e-12, "RadToDeg(pi/2) = 90");
    // 표시용 rpy(deg) 왕복. 짐벌 락에서 먼 값만 쓴다.
    const amr::units::RpyDeg d(20.0, -35.0, 110.0);
    const dhQuat q = amr::units::QuatFromRpyDeg(d);
    check(nearly(q.Norm(), 1.0f), "QuatFromRpyDeg is a unit quaternion");
    const amr::units::RpyDeg e = amr::units::RpyDegFromQuat(q);
    check(amr::Abs(e.roll - 20.0) < 1e-2 && amr::Abs(e.pitch + 35.0) < 1e-2 && amr::Abs(e.yaw - 110.0) < 1e-2, "rpy(deg) round trip");
    // URDF rpy 규약과 같다: yaw 90 도이면 x 축이 y 축이 된다
    check(nearlyVec(amr::units::QuatFromRpyDeg(amr::units::RpyDeg(0, 0, 90)).Rotate(dhVector(1, 0, 0)), dhVector(0, 1, 0)), "yaw 90 deg maps x to y");
    // -q 도 같은 회전이므로 같은 표시값이 나온다
    const dhQuat nq(-q.x, -q.y, -q.z, -q.w);
    const amr::units::RpyDeg f = amr::units::RpyDegFromQuat(nq);
    check(amr::Abs(f.roll - e.roll) < 1e-2 && amr::Abs(f.pitch - e.pitch) < 1e-2 && amr::Abs(f.yaw - e.yaw) < 1e-2, "q and -q display the same angles");
}


int main() {
    testVector();
    testVertex();
    testBasicTransforms();
    testQuatAxisAngle();
    testQuatFromQuaternion();
    testGetQuatRoundTrip();
    testRPY();
    testAngleHelpers();
    testVertexExtras();
    testQuatClass();
    testEulerClass();
    testQuatOps();
    testPose();
    testUnits();

    if (g_fail) {
        printf("test_3d: %d 실패\n", g_fail);
        return 1;
    }
    printf("test_3d: 모두 통과\n");
    return 0;
}
