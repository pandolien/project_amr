#ifndef CORE_CCOLLISION_AABB_H
#define CORE_CCOLLISION_AABB_H

#include "3d/dhMat.h"
#include "3d/dhVector.h"

// 축 정렬 경계 상자 (Axis-Aligned Bounding Box). 좌표계는 호출하는 쪽이 정한다 (보통 월드).
// 멤버 이름이 lo / hi 인 이유: Windows 헤더의 min / max 매크로와 충돌하지 않게 하기 위해서다.
//
// 약속
//   - 기본 생성은 "빈 상자" 이다. 빈 상자는 어떤 것과도 겹치지 않고 아무것도 포함하지 않는다.
//   - 면이 닿기만 해도 겹친 것으로 본다 (경계 포함, 닫힌 구간).
//   - float 이므로 닿는 경우의 판정은 반올림 오차에 영향을 받을 수 있다. 여유가 필요하면 Inflated() 를 쓴다.
namespace amr {

struct Aabb {
    dhVector lo;  // 각 축의 최솟값
    dhVector hi;  // 각 축의 최댓값

    Aabb();                                     // 빈 상자
    Aabb(const dhVector& l, const dhVector& h);  // 그대로 저장한다 (l > h 인 축이 있으면 빈 상자)

    static Aabb FromCenterHalfExtents(const dhVector& center, const dhVector& half);

    bool IsEmpty() const;  // 어느 한 축이라도 lo > hi 이거나 NaN 이면 true

    void Expand(const dhVector& p);  // 점 하나를 포함하도록 넓힌다
    void Merge(const Aabb& o);       // 다른 상자를 포함하도록 넓힌다 (빈 상자는 무시)
    // 모든 방향으로 margin 만큼 넓힌 새 상자. margin 이 음수이면 줄어들며, 너무 줄면 빈 상자가 된다.
    Aabb Inflated(float margin) const;

    dhVector Center() const;       // 빈 상자이면 원점
    dhVector Size() const;         // 각 축의 길이 (빈 상자이면 0)
    dhVector HalfExtents() const;  // Size() / 2
    float Volume() const;

    bool Contains(const dhVector& p) const;
    bool Contains(const Aabb& o) const;  // o 가 빈 상자이면 false
    bool Overlaps(const Aabb& o) const;

    // 두 상자 사이의 최단 거리 (겹치면 0). 하나라도 비어 있으면 -1.
    float Distance(const Aabb& o) const;

    // 겹친 정도: 겹치면 각 축에서 겹친 길이 중 가장 짧은 값(=밀어내야 하는 최소 거리)을, 아니면 0 을 돌려준다.
    float PenetrationDepth(const Aabb& o) const;

    // 이 상자를 m 으로 변환한 뒤의 상자를 감싸는 새 AABB (회전하면 원래보다 커질 수 있다).
    // m 은 회전/이동뿐 아니라 축별 스케일이 섞여 있어도 된다.
    Aabb Transformed(const dhMat& m) const;

    // 광선 origin + t * dir (0 <= t <= tMax) 이 상자와 만나는가. 만나면 처음 닿는 t 를 *tHit 로 준다
    // (광선 시작점이 상자 안이면 0). dir 은 단위벡터가 아니어도 된다. 영벡터이면 false.
    bool Raycast(const dhVector& origin, const dhVector& dir, float tMax, float* tHit) const;
};

}  // namespace amr

#endif  // CORE_CCOLLISION_AABB_H
