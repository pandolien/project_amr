#include "CBase/String.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace simul {

const size_t String::npos;

String::String() : s_(new char[1]), n_(0), cap_(0) { s_[0] = '\0'; }
String::String(const char* s) : s_(nullptr), n_(0), cap_(0) {
    s_ = new char[1];
    s_[0] = '\0';
    if (s) assignBytes(s, strlen(s));
}
String::String(const char* s, size_t n) : s_(new char[1]), n_(0), cap_(0) {
    s_[0] = '\0';
    if (s && n) assignBytes(s, n);
}
String::String(const char* first, const char* last) : s_(new char[1]), n_(0), cap_(0) {
    s_[0] = '\0';
    if (first && last > first) assignBytes(first, static_cast<size_t>(last - first));
}
String::String(size_t n, char c) : s_(new char[1]), n_(0), cap_(0) {
    s_[0] = '\0';
    append(n, c);
}
String::String(const String& o) : s_(new char[1]), n_(0), cap_(0) {
    s_[0] = '\0';
    assignBytes(o.s_, o.n_);
}
String::String(String&& o) : s_(o.s_), n_(o.n_), cap_(o.cap_) {
    o.s_ = new char[1];
    o.s_[0] = '\0';
    o.n_ = o.cap_ = 0;
}
String::~String() { delete[] s_; }

String& String::operator=(const String& o) {
    if (this != &o) assignBytes(o.s_, o.n_);
    return *this;
}
String& String::operator=(String&& o) {
    if (this != &o) {
        delete[] s_;
        s_ = o.s_;
        n_ = o.n_;
        cap_ = o.cap_;
        o.s_ = new char[1];
        o.s_[0] = '\0';
        o.n_ = o.cap_ = 0;
    }
    return *this;
}
String& String::operator=(const char* s) {
    if (!s) s = "";
    assignBytes(s, strlen(s));  // s 가 자기 안을 가리켜도 assignBytes 가 안전하게 처리한다
    return *this;
}

void String::assignBytes(const char* s, size_t n) {
    if (n <= cap_) {
        memmove(s_, s, n);  // 겹쳐도 안전
    } else {
        char* ns = new char[n + 1];
        memcpy(ns, s, n);
        delete[] s_;
        s_ = ns;
        cap_ = n;
    }
    n_ = n;
    s_[n_] = '\0';
}

void String::clear() {
    n_ = 0;
    s_[0] = '\0';
}

void String::reserve(size_t c) {
    if (c <= cap_) return;
    char* ns = new char[c + 1];
    memcpy(ns, s_, n_ + 1);
    delete[] s_;
    s_ = ns;
    cap_ = c;
}

void String::resize(size_t n, char c) {
    if (n < n_) {
        n_ = n;
        s_[n_] = '\0';
    } else if (n > n_) {
        append(n - n_, c);
    }
}

void String::push_back(char c) { append(1, c); }

String& String::append(const char* s) { return s ? append(s, strlen(s)) : *this; }

String& String::append(const char* s, size_t n) {
    if (n == 0) return *this;
    if (n_ + n > cap_) {
        size_t c = cap_ * 2;
        if (c < n_ + n) c = n_ + n;
        if (c < 15) c = 15;
        char* ns = new char[c + 1];
        memcpy(ns, s_, n_);
        memcpy(ns + n_, s, n);  // s 가 옛 버퍼 안을 가리켜도 아직 지우지 않았으니 안전
        delete[] s_;
        s_ = ns;
        cap_ = c;
    } else {
        memmove(s_ + n_, s, n);
    }
    n_ += n;
    s_[n_] = '\0';
    return *this;
}

String& String::append(size_t n, char c) {
    if (n == 0) return *this;
    reserve(n_ + n);
    for (size_t i = 0; i < n; i++) s_[n_ + i] = c;
    n_ += n;
    s_[n_] = '\0';
    return *this;
}

String& String::erase(size_t pos, size_t len) {
    if (pos >= n_) return *this;
    if (len > n_ - pos) len = n_ - pos;
    memmove(s_ + pos, s_ + pos + len, n_ - pos - len + 1);
    n_ -= len;
    return *this;
}

String& String::insert(size_t pos, const String& o) {
    if (pos > n_) pos = n_;
    const String copy(o);  // o 가 자기 자신일 수 있다
    reserve(n_ + copy.n_);
    memmove(s_ + pos + copy.n_, s_ + pos, n_ - pos + 1);
    memcpy(s_ + pos, copy.s_, copy.n_);
    n_ += copy.n_;
    return *this;
}

String& String::replace(size_t pos, size_t len, const String& o) {
    if (pos > n_) pos = n_;
    const String copy(o);
    erase(pos, len);
    return insert(pos, copy);
}

String String::substr(size_t pos, size_t len) const {
    if (pos >= n_) return String();
    if (len > n_ - pos) len = n_ - pos;
    return String(s_ + pos, len);
}

int String::compare(const String& o) const {
    const size_t m = n_ < o.n_ ? n_ : o.n_;
    const int c = memcmp(s_, o.s_, m);  // 바이트 값(부호 없음) 순서
    if (c != 0) return c;
    return n_ < o.n_ ? -1 : (n_ > o.n_ ? 1 : 0);
}

int String::compare(const char* s) const { return compare(String(s)); }

size_t String::find(char c, size_t pos) const {
    for (size_t i = pos; i < n_; i++)
        if (s_[i] == c) return i;
    return npos;
}

size_t String::find(const char* s, size_t pos) const {
    const size_t m = s ? strlen(s) : 0;
    if (pos > n_) return npos;
    if (m == 0) return pos;
    if (m > n_) return npos;
    for (size_t i = pos; i + m <= n_; i++)
        if (memcmp(s_ + i, s, m) == 0) return i;
    return npos;
}

size_t String::find(const String& o, size_t pos) const { return find(o.s_, pos); }

size_t String::rfind(char c, size_t pos) const {
    if (n_ == 0) return npos;
    size_t i = pos >= n_ ? n_ - 1 : pos;
    for (;; i--) {
        if (s_[i] == c) return i;
        if (i == 0) break;
    }
    return npos;
}

size_t String::rfind(const String& o, size_t pos) const {
    if (o.n_ > n_) return npos;
    size_t i = n_ - o.n_;
    if (pos < i) i = pos;
    for (;; i--) {
        if (memcmp(s_ + i, o.s_, o.n_) == 0) return i;
        if (i == 0) break;
    }
    return npos;
}

size_t String::find_first_of(const char* set, size_t pos) const {
    for (size_t i = pos; i < n_; i++)
        if (strchr(set, s_[i]) && s_[i] != '\0') return i;
    return npos;
}

size_t String::find_first_not_of(const char* set, size_t pos) const {
    for (size_t i = pos; i < n_; i++)
        if (!(strchr(set, s_[i]) && s_[i] != '\0')) return i;
    return npos;
}

size_t String::find_last_of(const char* set, size_t pos) const {
    if (n_ == 0) return npos;
    size_t i = pos >= n_ ? n_ - 1 : pos;
    for (;; i--) {
        if (strchr(set, s_[i]) && s_[i] != '\0') return i;
        if (i == 0) break;
    }
    return npos;
}

void String::swap(String& o) {
    char* s = s_;
    s_ = o.s_;
    o.s_ = s;
    size_t t = n_;
    n_ = o.n_;
    o.n_ = t;
    t = cap_;
    cap_ = o.cap_;
    o.cap_ = t;
}

String operator+(const String& a, const String& b) {
    String r;
    r.reserve(a.size() + b.size());
    r.append(a);
    r.append(b);
    return r;
}
String operator+(const String& a, const char* b) {
    String r(a);
    r.append(b);
    return r;
}
String operator+(const char* a, const String& b) {
    String r(a);
    r.append(b);
    return r;
}
String operator+(const String& a, char b) {
    String r(a);
    r.push_back(b);
    return r;
}
String operator+(String&& a, const String& b) {
    String r(Move(a));
    r.append(b);
    return r;
}

String operator+(String&& a, const char* b) {
    String r(Move(a));
    r.append(b);
    return r;
}
String operator+(String&& a, char b) {
    String r(Move(a));
    r.push_back(b);
    return r;
}

namespace {
String FormatV(const char* fmt, va_list ap) {
    va_list ap2;
    va_copy(ap2, ap);
    char buf[256];
    const int n = vsnprintf(buf, sizeof buf, fmt, ap);
    String r;
    if (n < 0) {
        va_end(ap2);
        return r;
    }
    if (static_cast<size_t>(n) < sizeof buf) {
        r.append(buf, static_cast<size_t>(n));
    } else {
        char* big = new char[static_cast<size_t>(n) + 1];
        vsnprintf(big, static_cast<size_t>(n) + 1, fmt, ap2);
        r.append(big, static_cast<size_t>(n));
        delete[] big;
    }
    va_end(ap2);
    return r;
}
}  // namespace

String Format(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const String r = FormatV(fmt, ap);
    va_end(ap);
    return r;
}

String ToString(int v) { return Format("%d", v); }
String ToString(unsigned v) { return Format("%u", v); }
String ToString(long v) { return Format("%ld", v); }
String ToString(unsigned long v) { return Format("%lu", v); }
String ToString(long long v) { return Format("%lld", v); }
String ToString(unsigned long long v) { return Format("%llu", v); }
String ToString(double v) { return Format("%g", v); }

}  // namespace simul
