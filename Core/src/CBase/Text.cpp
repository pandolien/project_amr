#include "CBase/Text.h"

#include <math.h>
#include <stdint.h>

namespace amr {

namespace {

bool IsSpace(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v'; }
bool IsDigit(char c) { return c >= '0' && c <= '9'; }
char Lower(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }

bool MatchWord(const char* p, const char* word) {  // 대소문자 무시, 뒤가 영숫자가 아니어야 한다
    size_t i = 0;
    for (; word[i]; i++)
        if (Lower(p[i]) != word[i]) return false;
    return !((p[i] >= 'a' && p[i] <= 'z') || (p[i] >= 'A' && p[i] <= 'Z') || IsDigit(p[i]));
}

// 10 의 거듭제곱 (22 까지는 double 로 정확히 표현된다)
const double kPow10[23] = {1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};

}  // namespace

bool ParseDouble(const char* s, double* out, const char** end) {
    *end = s;
    const char* p = s;
    while (IsSpace(*p)) p++;
    bool neg = false;
    if (*p == '+' || *p == '-') {
        neg = *p == '-';
        p++;
    }
    if (MatchWord(p, "nan")) {
        *out = NAN;
        *end = p + 3;
        return true;
    }
    if (MatchWord(p, "inf")) {
        *out = neg ? -INFINITY : INFINITY;
        *end = p + 3;
        return true;
    }
    if (MatchWord(p, "infinity")) {
        *out = neg ? -INFINITY : INFINITY;
        *end = p + 8;
        return true;
    }
    uint64_t mant = 0;
    int digits = 0, exp10 = 0;
    bool any = false;
    for (; IsDigit(*p); p++) {
        any = true;
        if (digits < 19) {
            mant = mant * 10 + static_cast<uint64_t>(*p - '0');
            if (mant != 0) digits++;
        } else {
            exp10++;  // 자릿수가 너무 많으면 뒤는 버리고 지수만 올린다
        }
    }
    if (*p == '.') {
        p++;
        for (; IsDigit(*p); p++) {
            any = true;
            if (digits < 19) {
                mant = mant * 10 + static_cast<uint64_t>(*p - '0');
                if (mant != 0) digits++;
                exp10--;
            }
        }
    }
    if (!any) return false;
    if (*p == 'e' || *p == 'E') {
        const char* q = p + 1;
        bool eneg = false;
        if (*q == '+' || *q == '-') {
            eneg = *q == '-';
            q++;
        }
        if (IsDigit(*q)) {
            int e = 0;
            for (; IsDigit(*q); q++)
                if (e < 10000) e = e * 10 + (*q - '0');
            exp10 += eneg ? -e : e;
            p = q;
        }
    }
    double v = static_cast<double>(mant);
    if (mant != 0 && exp10 != 0) {
        if (exp10 > 0) v = exp10 <= 22 ? v * kPow10[exp10] : v * pow(10.0, exp10);
        else v = -exp10 <= 22 ? v / kPow10[-exp10] : v / pow(10.0, -exp10);
    }
    *out = neg ? -v : v;
    *end = p;
    return true;
}

bool ParseInt(const char* s, long* out, const char** end) {
    *end = s;
    const char* p = s;
    while (IsSpace(*p)) p++;
    bool neg = false;
    if (*p == '+' || *p == '-') {
        neg = *p == '-';
        p++;
    }
    if (!IsDigit(*p)) return false;
    long v = 0;
    for (; IsDigit(*p); p++) {
        if (v < 100000000000000000L) v = v * 10 + (*p - '0');  // 넘치지 않게 멈춘다
    }
    *out = neg ? -v : v;
    *end = p;
    return true;
}

bool ToDouble(const String& s, double* out) {
    const char* end;
    if (!ParseDouble(s.c_str(), out, &end)) return false;
    while (IsSpace(*end)) end++;
    return *end == '\0';
}

bool ToInt(const String& s, long* out) {
    const char* end;
    if (!ParseInt(s.c_str(), out, &end)) return false;
    while (IsSpace(*end)) end++;
    return *end == '\0';
}

String Trim(const String& s) {
    size_t a = 0, b = s.size();
    while (a < b && IsSpace(s[a])) a++;
    while (b > a && IsSpace(s[b - 1])) b--;
    return s.substr(a, b - a);
}

String ToLower(const String& s) {
    String r(s);
    for (size_t i = 0; i < r.size(); i++) r[i] = Lower(r[i]);
    return r;
}

bool StartsWith(const String& s, const char* prefix) {
    size_t i = 0;
    for (; prefix[i]; i++)
        if (i >= s.size() || s[i] != prefix[i]) return false;
    return true;
}

bool Tokens::Next(String* tok) {
    while (pos_ < s_.size() && IsSpace(s_[pos_])) pos_++;
    if (pos_ >= s_.size()) return false;
    const size_t b = pos_;
    while (pos_ < s_.size() && !IsSpace(s_[pos_])) pos_++;
    *tok = s_.substr(b, pos_ - b);
    return true;
}

bool Tokens::NextFloat(float* v) {
    const size_t save = pos_;
    String t;
    double d;
    if (!Next(&t) || !ToDouble(t, &d)) {
        pos_ = save;
        return false;
    }
    *v = static_cast<float>(d);
    return true;
}

bool Tokens::NextInt(int* v) {
    const size_t save = pos_;
    String t;
    long l;
    if (!Next(&t) || !ToInt(t, &l)) {
        pos_ = save;
        return false;
    }
    *v = static_cast<int>(l);
    return true;
}

String Tokens::Rest() {
    while (pos_ < s_.size() && IsSpace(s_[pos_])) pos_++;
    String r = s_.substr(pos_);
    pos_ = s_.size();
    while (!r.empty() && IsSpace(r.back())) r.resize(r.size() - 1);
    return r;
}

bool Tokens::AtEnd() const {
    size_t p = pos_;
    while (p < s_.size() && IsSpace(s_[p])) p++;
    return p >= s_.size();
}

bool Lines::Next(String* line) {
    if (pos_ >= s_.size()) return false;
    size_t e = s_.find('\n', pos_);
    if (e == String::npos) e = s_.size();
    size_t len = e - pos_;
    if (len > 0 && s_[pos_ + len - 1] == '\r') len--;
    *line = s_.substr(pos_, len);
    pos_ = e < s_.size() ? e + 1 : s_.size();
    return true;
}

}  // namespace amr
