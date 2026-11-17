#ifndef CORE_CBASE_UTIL_H
#define CORE_CBASE_UTIL_H

#include <stddef.h>

// C++ 표준 라이브러리(std)를 쓰지 않는 이 프로젝트의 기본 도구: Pair, Min/Max/Swap/Move, Sort/Unique, UniquePtr.
// 이름과 동작은 std 의 같은 이름과 최대한 같게 맞췄다 (Min, Max, Swap, Move, Pair, MakePair, Sort, Unique).
namespace simul {

template <class T> struct RemoveRef { typedef T Type; };
template <class T> struct RemoveRef<T&> { typedef T Type; };
template <class T> struct RemoveRef<T&&> { typedef T Type; };

template <class T>
inline typename RemoveRef<T>::Type&& Move(T&& t) {
    return static_cast<typename RemoveRef<T>::Type&&>(t);
}

template <class T> inline const T& Min(const T& a, const T& b) { return b < a ? b : a; }
template <class T> inline const T& Max(const T& a, const T& b) { return a < b ? b : a; }

template <class T>
inline void Swap(T& a, T& b) {
    T t(Move(a));
    a = Move(b);
    b = Move(t);
}

template <class A, class B>
struct Pair {
    A first;
    B second;
    Pair() : first(), second() {}
    Pair(const A& a, const B& b) : first(a), second(b) {}
};

template <class A, class B> inline Pair<A, B> MakePair(const A& a, const B& b) { return Pair<A, B>(a, b); }
template <class A, class B> inline bool operator==(const Pair<A, B>& x, const Pair<A, B>& y) { return x.first == y.first && x.second == y.second; }
template <class A, class B> inline bool operator!=(const Pair<A, B>& x, const Pair<A, B>& y) { return !(x == y); }
template <class A, class B>
inline bool operator<(const Pair<A, B>& x, const Pair<A, B>& y) {
    return x.first < y.first || (!(y.first < x.first) && x.second < y.second);
}

// 안정 정렬 (같은 값의 순서가 유지된다). 병합 정렬이라 항상 O(n log n).
namespace detail {
template <class T, class Cmp>
void MergeSortRec(T* a, T* tmp, size_t n, Cmp cmp) {
    if (n < 2) return;
    if (n <= 12) {  // 작으면 삽입 정렬
        for (size_t i = 1; i < n; i++) {
            T v(Move(a[i]));
            size_t j = i;
            while (j > 0 && cmp(v, a[j - 1])) {
                a[j] = Move(a[j - 1]);
                j--;
            }
            a[j] = Move(v);
        }
        return;
    }
    const size_t h = n / 2;
    MergeSortRec(a, tmp, h, cmp);
    MergeSortRec(a + h, tmp, n - h, cmp);
    size_t i = 0, j = h, k = 0;
    while (i < h && j < n) tmp[k++] = cmp(a[j], a[i]) ? Move(a[j++]) : Move(a[i++]);
    while (i < h) tmp[k++] = Move(a[i++]);
    while (j < n) tmp[k++] = Move(a[j++]);
    for (size_t t = 0; t < n; t++) a[t] = Move(tmp[t]);
}
template <class T> struct Less {
    bool operator()(const T& a, const T& b) const { return a < b; }
};
}  // namespace detail

// 포인터 범위 [first, last) 를 cmp 로 정렬한다 (Array::begin()/end() 가 포인터라서 그대로 쓰면 된다)
template <class T, class Cmp>
void Sort(T* first, T* last, Cmp cmp) {
    const size_t n = static_cast<size_t>(last - first);
    if (n < 2) return;
    T* tmp = new T[n];
    detail::MergeSortRec(first, tmp, n, cmp);
    delete[] tmp;
}
template <class T>
void Sort(T* first, T* last) {
    Sort(first, last, detail::Less<T>());
}

// 정렬된 범위에서 이웃한 같은 값을 하나로 합치고 새 끝을 돌려준다 (erase 와 함께 쓴다)
template <class T>
T* Unique(T* first, T* last) {
    if (first == last) return last;
    T* out = first;
    for (T* p = first + 1; p != last; ++p)
        if (!(*out == *p)) *++out = Move(*p);
    return out + 1;
}

// 소유 포인터 (unique_ptr). 복사 불가, 이동만 가능.
template <class T>
class UniquePtr {
public:
    UniquePtr() : p_(nullptr) {}
    explicit UniquePtr(T* p) : p_(p) {}
    UniquePtr(UniquePtr&& o) : p_(o.p_) { o.p_ = nullptr; }
    UniquePtr& operator=(UniquePtr&& o) {
        if (this != &o) {
            delete p_;
            p_ = o.p_;
            o.p_ = nullptr;
        }
        return *this;
    }
    ~UniquePtr() { delete p_; }
    T* get() const { return p_; }
    T* operator->() const { return p_; }
    T& operator*() const { return *p_; }
    T* release() {
        T* p = p_;
        p_ = nullptr;
        return p;
    }
    void reset(T* p = nullptr) {
        if (p != p_) {
            delete p_;
            p_ = p;
        }
    }
    explicit operator bool() const { return p_ != nullptr; }

private:
    UniquePtr(const UniquePtr&);
    UniquePtr& operator=(const UniquePtr&);
    T* p_;
};

}  // namespace simul

#endif  // CORE_CBASE_UTIL_H
