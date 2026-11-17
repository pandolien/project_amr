#ifndef CORE_CCOLLISION_BROADPHASE_H
#define CORE_CCOLLISION_BROADPHASE_H

#include "CBase/Array.h"

#include "CCollision/Aabb.h"

// 브로드페이즈: AABB 들 중 서로 겹치는 쌍을 빠르게 찾는다 (Sweep and Prune, x 축 기준).
// 상자가 n 개이고 겹치는 쌍이 k 개이면 O(n log n + 겹침 후보 수). 빈 상자는 건너뛴다.
// 결과는 (a, b) 오름차순으로 정렬되어 있어 항상 같은 순서로 나온다.
namespace simul {

struct CollisionPair {
    int a;
    int b;
    CollisionPair() : a(-1), b(-1) {}
    CollisionPair(int pa, int pb) : a(pa), b(pb) {}
    bool operator==(const CollisionPair& o) const { return a == o.a && b == o.b; }
    bool operator<(const CollisionPair& o) const { return a < o.a || (a == o.a && b < o.b); }
};

// 한 집합 안에서 겹치는 쌍. 항상 a < b 이다.
simul::Array<CollisionPair> FindOverlappingPairs(const simul::Array<Aabb>& boxes);

// 두 집합 사이에서 겹치는 쌍. a 는 setA 의 인덱스, b 는 setB 의 인덱스다. (같은 집합 안의 쌍은 찾지 않는다)
simul::Array<CollisionPair> FindOverlappingPairs(const simul::Array<Aabb>& setA, const simul::Array<Aabb>& setB);

// 비교용 O(n^2) 구현. 테스트에서 위 함수의 결과를 검증하는 데 쓴다.
simul::Array<CollisionPair> FindOverlappingPairsBruteForce(const simul::Array<Aabb>& boxes);

}  // namespace simul

#endif  // CORE_CCOLLISION_BROADPHASE_H
