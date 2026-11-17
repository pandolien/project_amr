#ifndef CORE_CBASE_ARRAY_H
#define CORE_CBASE_ARRAY_H

#include <stddef.h>
#include <stdlib.h>

#include "CBase/Util.h"

namespace simul {

// 크기가 변하는 배열 (vector 대체). 원소 타입은 기본 생성과 복사/이동 대입이 가능해야 한다.
// 반복자는 포인터이고, 원소가 옮겨지는 연산(push_back 등)을 하면 이전 포인터는 무효가 된다.
template <class T>
class Array {
public:
    typedef T* iterator;
    typedef const T* const_iterator;
    typedef T value_type;

    Array() : d_(nullptr), n_(0), cap_(0) {}
    explicit Array(size_t n) : d_(nullptr), n_(0), cap_(0) { resize(n); }
    Array(size_t n, const T& v) : d_(nullptr), n_(0), cap_(0) { assign(n, v); }
    Array(const Array& o) : d_(nullptr), n_(0), cap_(0) { assign(o.begin(), o.end()); }
    Array(Array&& o) : d_(o.d_), n_(o.n_), cap_(o.cap_) {
        o.d_ = nullptr;
        o.n_ = o.cap_ = 0;
    }
    // 포인터 범위로 만든다. (정수 두 개를 (개수, 값) 으로 읽어야 할 때 헷갈리지 않도록 포인터만 받는다)
    template <class U>
    Array(const U* first, const U* last) : d_(nullptr), n_(0), cap_(0) { assign(first, last); }
    ~Array() { delete[] d_; }

    Array& operator=(const Array& o) {
        if (this != &o) assign(o.begin(), o.end());
        return *this;
    }
    Array& operator=(Array&& o) {
        if (this != &o) {
            delete[] d_;
            d_ = o.d_;
            n_ = o.n_;
            cap_ = o.cap_;
            o.d_ = nullptr;
            o.n_ = o.cap_ = 0;
        }
        return *this;
    }

    size_t size() const { return n_; }
    bool empty() const { return n_ == 0; }
    size_t capacity() const { return cap_; }
    T* data() { return d_; }
    const T* data() const { return d_; }
    T& operator[](size_t i) { return d_[i]; }
    const T& operator[](size_t i) const { return d_[i]; }
    T& front() { return d_[0]; }
    const T& front() const { return d_[0]; }
    T& back() { return d_[n_ - 1]; }
    const T& back() const { return d_[n_ - 1]; }
    iterator begin() { return d_; }
    iterator end() { return d_ + n_; }
    const_iterator begin() const { return d_; }
    const_iterator end() const { return d_ + n_; }

    void reserve(size_t c) {
        if (c <= cap_) return;
        if (c > (~static_cast<size_t>(0)) / 2 / sizeof(T)) abort();  // 크기 계산이 넘치는 터무니없는 요청 (음수를 size_t 로 넘긴 경우 등)
        T* nd = new T[c]();
        for (size_t i = 0; i < n_; i++) nd[i] = Move(d_[i]);
        delete[] d_;
        d_ = nd;
        cap_ = c;
    }
    void clear() {  // 용량은 그대로 둔다. 버려지는 원소가 자원을 쥐고 있지 않도록 비운 값으로 덮는다
        for (size_t i = 0; i < n_; i++) d_[i] = T();
        n_ = 0;
    }
    void push_back(const T& v) {
        if (n_ == cap_) {
            T copy(v);  // v 가 이 배열 안의 원소일 수 있으므로 옮기기 전에 복사해 둔다
            reserve(grow());
            d_[n_++] = Move(copy);
        } else {
            d_[n_++] = v;
        }
    }
    void push_back(T&& v) {
        if (n_ == cap_) {
            T moved(Move(v));
            reserve(grow());
            d_[n_++] = Move(moved);
        } else {
            d_[n_++] = Move(v);
        }
    }
    void pop_back() {
        d_[n_ - 1] = T();
        n_--;
    }
    void resize(size_t n) {
        if (n > cap_) reserve(n);
        for (size_t i = n; i < n_; i++) d_[i] = T();
        for (size_t i = n_; i < n; i++) d_[i] = T();  // 늘어난 칸은 값 초기화 (vector 와 같다)
        n_ = n;
    }
    void resize(size_t n, const T& v) {
        for (size_t i = n; i < n_; i++) d_[i] = T();
        if (n > cap_) {
            T copy(v);
            reserve(n);
            for (size_t i = n_; i < n; i++) d_[i] = copy;
        } else {
            for (size_t i = n_; i < n; i++) d_[i] = v;
        }
        n_ = n;
    }
    void assign(size_t n, const T& v) {
        T copy(v);
        n_ = 0;
        reserve(n);
        for (size_t i = 0; i < n; i++) d_[i] = copy;
        n_ = n;
    }
    template <class U>
    void assign(const U* first, const U* last) {
        Array tmp;  // 자기 자신의 일부를 넣는 경우에도 안전하도록 새 배열에 먼저 만든다
        tmp.reserve(static_cast<size_t>(last - first));
        for (const U* p = first; p != last; ++p) tmp.push_back(T(*p));
        swap(tmp);
    }
    // pos 앞에 넣는다. 돌려주는 반복자는 넣은 첫 원소.
    iterator insert(iterator pos, const T& v) {
        const size_t i = static_cast<size_t>(pos - d_);
        T copy(v);
        openGap(i, 1);
        d_[i] = Move(copy);
        return d_ + i;
    }
    template <class U>
    iterator insert(iterator pos, const U* first, const U* last) {
        const size_t i = static_cast<size_t>(pos - d_);
        const size_t k = static_cast<size_t>(last - first);
        Array tmp(first, last);  // 원본이 이 배열 안일 수 있어서 먼저 복사
        openGap(i, k);
        for (size_t j = 0; j < k; j++) d_[i + j] = Move(tmp.d_[j]);
        return d_ + i;
    }
    iterator erase(iterator pos) { return erase(pos, pos + 1); }
    iterator erase(iterator first, iterator last) {
        const size_t i = static_cast<size_t>(first - d_), k = static_cast<size_t>(last - first);
        for (size_t j = i; j + k < n_; j++) d_[j] = Move(d_[j + k]);
        for (size_t j = n_ - k; j < n_; j++) d_[j] = T();
        n_ -= k;
        return d_ + i;
    }
    void swap(Array& o) {
        T* d = d_;
        d_ = o.d_;
        o.d_ = d;
        size_t t = n_;
        n_ = o.n_;
        o.n_ = t;
        t = cap_;
        cap_ = o.cap_;
        o.cap_ = t;
    }

private:
    size_t grow() const { return cap_ < 4 ? 4 : cap_ * 2; }
    void openGap(size_t i, size_t k) {
        if (n_ + k > cap_) {
            size_t c = grow();
            if (c < n_ + k) c = n_ + k;
            reserve(c);
        }
        for (size_t j = n_; j > i; j--) d_[j - 1 + k] = Move(d_[j - 1]);
        n_ += k;
    }
    T* d_;
    size_t n_, cap_;
};

template <class T>
bool operator==(const Array<T>& a, const Array<T>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++)
        if (!(a[i] == b[i])) return false;
    return true;
}
template <class T> bool operator!=(const Array<T>& a, const Array<T>& b) { return !(a == b); }

}  // namespace simul

#endif  // CORE_CBASE_ARRAY_H
