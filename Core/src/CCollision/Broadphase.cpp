#include "CCollision/Broadphase.h"

#include <algorithm>

namespace amr {

namespace {

struct Item {
    const Aabb* box;
    int set;  // 0 또는 1
    int idx;  // 원래 집합 안의 인덱스
};

bool ItemLess(const Item& p, const Item& q) {
    if (p.box->lo.x != q.box->lo.x) return p.box->lo.x < q.box->lo.x;
    if (p.set != q.set) return p.set < q.set;
    return p.idx < q.idx;
}

// y, z 축까지 겹치는지 (x 는 sweep 이 이미 걸러냈다)
bool OverlapsYZ(const Aabb& p, const Aabb& q) {
    return p.lo.y <= q.hi.y && q.lo.y <= p.hi.y && p.lo.z <= q.hi.z && q.lo.z <= p.hi.z;
}

// items 를 lo.x 순으로 훑으며 겹치는 쌍을 모은다. crossOnly 이면 서로 다른 집합끼리만 쌍으로 인정한다.
std::vector<CollisionPair> Sweep(std::vector<Item>* items, bool crossOnly) {
    std::vector<CollisionPair> out;
    std::sort(items->begin(), items->end(), ItemLess);
    const size_t n = items->size();
    for (size_t i = 0; i < n; i++) {
        const Item& p = (*items)[i];
        for (size_t j = i + 1; j < n; j++) {
            const Item& q = (*items)[j];
            if (q.box->lo.x > p.box->hi.x) break;  // 이후 상자는 모두 x 에서 떨어져 있다
            if (crossOnly && p.set == q.set) continue;
            if (!OverlapsYZ(*p.box, *q.box)) continue;
            if (crossOnly) {
                // 항상 (setA 의 인덱스, setB 의 인덱스) 로 돌려준다
                out.push_back(p.set == 0 ? CollisionPair(p.idx, q.idx) : CollisionPair(q.idx, p.idx));
            } else {
                out.push_back(p.idx < q.idx ? CollisionPair(p.idx, q.idx) : CollisionPair(q.idx, p.idx));
            }
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace

std::vector<CollisionPair> FindOverlappingPairs(const std::vector<Aabb>& boxes) {
    std::vector<Item> items;
    items.reserve(boxes.size());
    for (size_t i = 0; i < boxes.size(); i++) {
        if (boxes[i].IsEmpty()) continue;
        Item it = {&boxes[i], 0, static_cast<int>(i)};
        items.push_back(it);
    }
    return Sweep(&items, false);
}

std::vector<CollisionPair> FindOverlappingPairs(const std::vector<Aabb>& setA, const std::vector<Aabb>& setB) {
    std::vector<Item> items;
    items.reserve(setA.size() + setB.size());
    for (size_t i = 0; i < setA.size(); i++) {
        if (setA[i].IsEmpty()) continue;
        Item it = {&setA[i], 0, static_cast<int>(i)};
        items.push_back(it);
    }
    for (size_t i = 0; i < setB.size(); i++) {
        if (setB[i].IsEmpty()) continue;
        Item it = {&setB[i], 1, static_cast<int>(i)};
        items.push_back(it);
    }
    return Sweep(&items, true);
}

std::vector<CollisionPair> FindOverlappingPairsBruteForce(const std::vector<Aabb>& boxes) {
    std::vector<CollisionPair> out;
    for (size_t i = 0; i < boxes.size(); i++)
        for (size_t j = i + 1; j < boxes.size(); j++)
            if (boxes[i].Overlaps(boxes[j])) out.push_back(CollisionPair(static_cast<int>(i), static_cast<int>(j)));
    return out;
}

}  // namespace amr
