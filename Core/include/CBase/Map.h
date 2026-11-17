#ifndef CORE_CBASE_MAP_H
#define CORE_CBASE_MAP_H

#include <stdio.h>
#include <stdlib.h>

#include "CBase/Array.h"
#include "CBase/Util.h"

namespace simul {

// 키로 정렬된 맵 (map 대체). 정렬된 배열에 이진 탐색으로 찾는다: 찾기는 O(log n), 넣기/지우기는 O(n).
// 원소가 수천 개 이하인 이름표/설정 용도이고, 수만 개가 넘는 조회에는 HashMap 을 쓴다.
// 주의: 넣거나 지우면 이전에 얻은 반복자(포인터)는 무효가 된다 (map 과 다른 점). 반복은 키 순서이다.
template <class K, class V>
class Map {
public:
    typedef Pair<K, V> value_type;
    typedef value_type* iterator;
    typedef const value_type* const_iterator;

    size_t size() const { return a_.size(); }
    bool empty() const { return a_.empty(); }
    void clear() { a_.clear(); }
    iterator begin() { return a_.begin(); }
    iterator end() { return a_.end(); }
    const_iterator begin() const { return a_.begin(); }
    const_iterator end() const { return a_.end(); }

    iterator find(const K& k) {
        const size_t i = lower(k);
        return (i < a_.size() && !(k < a_[i].first)) ? a_.begin() + i : a_.end();
    }
    const_iterator find(const K& k) const {
        const size_t i = lower(k);
        return (i < a_.size() && !(k < a_[i].first)) ? a_.begin() + i : a_.end();
    }
    size_t count(const K& k) const { return find(k) != end() ? 1 : 0; }

    V& operator[](const K& k) {
        const size_t i = lower(k);
        if (i < a_.size() && !(k < a_[i].first)) return a_[i].second;
        return a_.insert(a_.begin() + i, value_type(k, V()))->second;
    }
    // 없으면 프로그램을 멈춘다 (map::at 은 예외를 던지지만 이 프로젝트는 예외를 쓰지 않는다)
    V& at(const K& k) {
        iterator it = find(k);
        if (it == end()) {
            fputs("Map::at: key not found\n", stderr);
            abort();
        }
        return it->second;
    }
    const V& at(const K& k) const {
        const_iterator it = find(k);
        if (it == end()) {
            fputs("Map::at: key not found\n", stderr);
            abort();
        }
        return it->second;
    }
    // 이미 있으면 넣지 않는다. 결과의 second 는 새로 넣었는지.
    Pair<iterator, bool> insert(const value_type& v) {
        const size_t i = lower(v.first);
        if (i < a_.size() && !(v.first < a_[i].first)) return Pair<iterator, bool>(a_.begin() + i, false);
        return Pair<iterator, bool>(a_.insert(a_.begin() + i, v), true);
    }
    size_t erase(const K& k) {
        iterator it = find(k);
        if (it == end()) return 0;
        a_.erase(it);
        return 1;
    }

private:
    size_t lower(const K& k) const {  // k 이상인 첫 위치
        size_t lo = 0, hi = a_.size();
        while (lo < hi) {
            const size_t mid = lo + (hi - lo) / 2;
            if (a_[mid].first < k) lo = mid + 1;
            else hi = mid;
        }
        return lo;
    }
    Array<value_type> a_;
};

// 정렬된 집합 (set 대체). 같은 성질: 작은 집합용.
template <class T>
class Set {
public:
    typedef T* iterator;
    typedef const T* const_iterator;
    size_t size() const { return a_.size(); }
    bool empty() const { return a_.empty(); }
    void clear() { a_.clear(); }
    iterator begin() { return a_.begin(); }
    iterator end() { return a_.end(); }
    const_iterator begin() const { return a_.begin(); }
    const_iterator end() const { return a_.end(); }
    const_iterator find(const T& v) const {
        const size_t i = lower(v);
        return (i < a_.size() && !(v < a_[i])) ? a_.begin() + i : a_.end();
    }
    size_t count(const T& v) const { return find(v) != end() ? 1 : 0; }
    Pair<iterator, bool> insert(const T& v) {
        const size_t i = lower(v);
        if (i < a_.size() && !(v < a_[i])) return Pair<iterator, bool>(a_.begin() + i, false);
        return Pair<iterator, bool>(a_.insert(a_.begin() + i, v), true);
    }
    size_t erase(const T& v) {
        const_iterator it = find(v);
        if (it == end()) return 0;
        a_.erase(a_.begin() + (it - a_.begin()));
        return 1;
    }

private:
    size_t lower(const T& v) const {
        size_t lo = 0, hi = a_.size();
        while (lo < hi) {
            const size_t mid = lo + (hi - lo) / 2;
            if (a_[mid] < v) lo = mid + 1;
            else hi = mid;
        }
        return lo;
    }
    Array<T> a_;
};

}  // namespace simul

#endif  // CORE_CBASE_MAP_H
