#ifndef CORE_CBASE_STRING_H
#define CORE_CBASE_STRING_H

#include <stddef.h>

#include "CBase/Util.h"

namespace simul {

// 문자열 (string 대체). 바이트 문자열이고 항상 널 문자로 끝난다. UTF-8 을 그대로 담는다.
class String {
public:
    static const size_t npos = static_cast<size_t>(-1);

    String();
    String(const char* s);  // NULL 은 빈 문자열
    String(const char* s, size_t n);
    String(const char* first, const char* last);  // [first, last)
    String(size_t n, char c);
    String(const String& o);
    String(String&& o);
    ~String();
    String& operator=(const String& o);
    String& operator=(String&& o);
    String& operator=(const char* s);

    size_t size() const { return n_; }
    size_t length() const { return n_; }
    bool empty() const { return n_ == 0; }
    const char* c_str() const { return s_; }
    const char* data() const { return s_; }
    char& operator[](size_t i) { return s_[i]; }
    const char& operator[](size_t i) const { return s_[i]; }
    char& back() { return s_[n_ - 1]; }
    const char& back() const { return s_[n_ - 1]; }
    char* begin() { return s_; }
    char* end() { return s_ + n_; }
    const char* begin() const { return s_; }
    const char* end() const { return s_ + n_; }

    void clear();
    void reserve(size_t c);
    void resize(size_t n, char c = '\0');
    void push_back(char c);
    String& append(const String& o) { return append(o.s_, o.n_); }
    String& append(const char* s);
    String& append(const char* s, size_t n);
    String& append(size_t n, char c);
    String& operator+=(const String& o) { return append(o.s_, o.n_); }
    String& operator+=(const char* s) { return append(s); }
    String& operator+=(char c) {
        push_back(c);
        return *this;
    }
    String& erase(size_t pos, size_t len = npos);
    String& insert(size_t pos, const String& o);
    String& replace(size_t pos, size_t len, const String& o);
    String substr(size_t pos = 0, size_t len = npos) const;
    int compare(const String& o) const;
    int compare(const char* s) const;
    int compare(size_t pos, size_t len, const String& o) const { return substr(pos, len).compare(o); }
    size_t find(char c, size_t pos = 0) const;
    size_t find(const String& o, size_t pos = 0) const;
    size_t find(const char* s, size_t pos = 0) const;
    size_t rfind(char c, size_t pos = npos) const;
    size_t rfind(const String& o, size_t pos = npos) const;
    size_t find_first_of(const char* set, size_t pos = 0) const;
    size_t find_first_not_of(const char* set, size_t pos = 0) const;
    size_t find_last_of(const char* set, size_t pos = npos) const;
    size_t find_last_of(char c, size_t pos = npos) const { return rfind(c, pos); }
    size_t find_first_of(char c, size_t pos = 0) const { return find(c, pos); }
    void swap(String& o);

private:
    void assignBytes(const char* s, size_t n);
    char* s_;
    size_t n_, cap_;  // cap_ 는 널 문자를 뺀 용량
};

String operator+(const String& a, const String& b);
String operator+(const String& a, const char* b);
String operator+(const char* a, const String& b);
String operator+(const String& a, char b);
String operator+(String&& a, const String& b);
String operator+(String&& a, const char* b);
String operator+(String&& a, char b);
inline bool operator==(const String& a, const String& b) { return a.compare(b) == 0; }
inline bool operator==(const String& a, const char* b) { return a.compare(b) == 0; }
inline bool operator==(const char* a, const String& b) { return b.compare(a) == 0; }
inline bool operator!=(const String& a, const String& b) { return a.compare(b) != 0; }
inline bool operator!=(const String& a, const char* b) { return a.compare(b) != 0; }
inline bool operator!=(const char* a, const String& b) { return b.compare(a) != 0; }
inline bool operator<(const String& a, const String& b) { return a.compare(b) < 0; }
inline bool operator>(const String& a, const String& b) { return a.compare(b) > 0; }
inline bool operator<=(const String& a, const String& b) { return a.compare(b) <= 0; }
inline bool operator>=(const String& a, const String& b) { return a.compare(b) >= 0; }

// 숫자 -> 문자열 (to_string 대체). 실수는 필요한 만큼의 자릿수로 (%.9g / %.17g 가 아니라 짧은 쪽 %g 의 확장)
String ToString(int v);
String ToString(unsigned v);
String ToString(long v);
String ToString(unsigned long v);
String ToString(long long v);
String ToString(unsigned long long v);
String ToString(double v);  // "%g" 형식 (유효숫자 6 자리). 더 필요하면 Format 을 쓴다
// printf 형식 (snprintf 로 만든다). 결과가 아주 길어도 잘리지 않는다.
String Format(const char* fmt, ...);

}  // namespace simul

#endif  // CORE_CBASE_STRING_H
