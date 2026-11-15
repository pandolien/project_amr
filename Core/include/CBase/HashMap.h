#ifndef CORE_CBASE_HASHMAP_H
#define CORE_CBASE_HASHMAP_H

#include <stddef.h>

#include "CBase/Array.h"

namespace amr {

// 해시 맵: 정점 합치기처럼 항목이 수십만 개가 될 수 있는 조회용 (평균 O(1)). 순서는 없다.
// K 는 == 비교와 `size_t Hash() const` 를 가져야 한다. 지우기는 지원하지 않는다.
// Find 가 돌려준 포인터는 다음 Insert 전까지만 유효하다.
template <class K, class V>
class HashMap {
public:
    HashMap() : count_(0) {}
    size_t size() const { return count_; }
    void clear() {
        slots_.clear();
        count_ = 0;
    }
    V* Find(const K& k) {
        if (slots_.empty()) return nullptr;
        const size_t mask = slots_.size() - 1;
        for (size_t i = k.Hash() & mask;; i = (i + 1) & mask) {
            Slot& s = slots_[i];
            if (!s.used) return nullptr;
            if (s.key == k) return &s.value;
        }
    }
    // 없으면 v 로 새로 넣는다. 이미 있으면 기존 값을 그대로 두고 그 값의 주소를 돌려준다. *inserted 는 새로 넣었는지.
    V* Insert(const K& k, const V& v, bool* inserted = nullptr) {
        if ((count_ + 1) * 2 > slots_.size()) grow();
        const size_t mask = slots_.size() - 1;
        for (size_t i = k.Hash() & mask;; i = (i + 1) & mask) {
            Slot& s = slots_[i];
            if (!s.used) {
                s.used = true;
                s.key = k;
                s.value = v;
                count_++;
                if (inserted) *inserted = true;
                return &s.value;
            }
            if (s.key == k) {
                if (inserted) *inserted = false;
                return &s.value;
            }
        }
    }

private:
    struct Slot {
        bool used;
        K key;
        V value;
        Slot() : used(false), key(), value() {}
    };
    void grow() {
        Array<Slot> old;
        old.swap(slots_);
        size_t n = old.size() < 8 ? 16 : old.size() * 2;
        slots_.resize(n);
        const size_t mask = n - 1;
        for (size_t j = 0; j < old.size(); j++) {
            if (!old[j].used) continue;
            size_t i = old[j].key.Hash() & mask;
            while (slots_[i].used) i = (i + 1) & mask;
            slots_[i] = old[j];
        }
    }
    Array<Slot> slots_;
    size_t count_;
};

}  // namespace amr

#endif  // CORE_CBASE_HASHMAP_H
