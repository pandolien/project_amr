#ifndef CORE_CCOLLISION_OBB_H
#define CORE_CCOLLISION_OBB_H

#include "3d/dhMat.h"
#include "3d/dhVector.h"
#include "CCollision/Aabb.h"

// 방향 경계 상자 (Oriented Bounding Box): 중심 + 서로 직교하는 세 단위 축 + 축 방향 반크기.
// 물체와 함께 회전하므로 기울어진 물체에도 딱 맞는다 (AABB 는 축에 고정이라 헐렁해진다).
//
// 약속
//   - axis[0..2] 는 단위벡터이고 서로 직교한다. half 는 각 축 방향의 반크기(>= 0).
//   - 기본 생성은 "빈 상자" 이다 (half 가 음수). 빈 상자는 어떤 것과도 겹치지 않는다.
//   - 면이 닿기만 해도 겹친 것으로 본다. 판정은 double 로 계산하고, 반올림 오차 때문에
//     겹침을 놓치지 않도록 아주 작은 여유(1e-6 상대값)를 둔다.
namespace simul {

struct Obb {
    dhVector center;
    dhVector axis[3];
    dhVector half;

    Obb();  // 빈 상자
    Obb(const dhVector& c, const dhVector& ax0, const dhVector& ax1, const dhVector& ax2, const dhVector& h);

    static Obb FromAabb(const Aabb& box);  // 빈 AABB 는 빈 OBB 가 된다

    bool IsEmpty() const;  // half 의 어느 성분이 음수이거나 NaN 이면 true
    float Volume() const;

    void Corners(dhVector out[8]) const;  // 꼭짓점 8개. i 의 비트 0,1,2 가 각 축의 +/- 를 정한다
    Aabb ToAabb() const;                  // 이 OBB 를 감싸는 가장 작은 AABB
    bool Contains(const dhVector& p) const;
    Obb Inflated(float margin) const;     // 모든 방향으로 margin 만큼 키운 새 상자 (음수면 줄어들며, 너무 줄면 빈 상자)

    // m 으로 변환한 OBB. m 은 회전 + 이동이고, 균일 스케일 또는 이 상자의 축 방향 스케일은 허용한다.
    // (축이 서로 직교하지 않게 되는 전단(shear) 변환은 지원하지 않는다.)
    Obb Transformed(const dhMat& m) const;

    // 분리축 정리(SAT)로 겹침을 판정한다: 면 법선 6개 + 모서리 외적 9개 = 15개 축.
    bool Overlaps(const Obb& o) const;
};

}  // namespace simul

#endif  // CORE_CCOLLISION_OBB_H
