#ifndef CORE_CBASE_TEXT_H
#define CORE_CBASE_TEXT_H

#include <stddef.h>

#include "CBase/Array.h"
#include "CBase/String.h"

// 텍스트 읽기 도구 (istringstream, getline, strtod 대체). 숫자 읽기는 로케일(소수점 기호 설정)과 무관하게 항상 '.' 이다.
namespace amr {

// s 에서 실수 하나를 읽는다 (앞의 공백은 건너뜀). 읽었으면 true 와 읽은 끝 위치(*end). "nan", "inf", "-inf" 도 읽는다.
// 십진수가 아니거나 비어 있으면 false (이때 *end 는 s).
bool ParseDouble(const char* s, double* out, const char** end);
bool ParseInt(const char* s, long* out, const char** end);

// 문자열 전체가 숫자 하나(앞뒤 공백만 허용)일 때만 true
bool ToDouble(const String& s, double* out);
bool ToInt(const String& s, long* out);

String Trim(const String& s);   // 앞뒤의 공백(스페이스, 탭, 줄바꿈)을 없앤다
String ToLower(const String& s);  // ASCII 만 바꾼다
bool StartsWith(const String& s, const char* prefix);

// 공백으로 나눈 토큰을 하나씩 읽는다 (istringstream 의 >> 와 같다).
class Tokens {
public:
    explicit Tokens(const String& s) : s_(s), pos_(0) {}
    bool Next(String* tok);          // 다음 토큰. 더 없으면 false
    bool NextFloat(float* v);        // 다음 토큰을 실수로. 토큰이 없거나 숫자가 아니면 false (토큰은 소비하지 않는다)
    bool NextInt(int* v);
    String Rest();                   // 남은 부분 전체 (앞의 공백 제거, 줄 끝의 공백/'\r' 제거)
    bool AtEnd() const;

private:
    const String s_;
    size_t pos_;
};

// 문자열을 줄 단위로 읽는다 (getline). "\r\n" 도 줄바꿈으로 보고 '\r' 은 줄에 넣지 않는다.
class Lines {
public:
    explicit Lines(const String& s) : s_(s), pos_(0) {}
    bool Next(String* line);

private:
    const String s_;
    size_t pos_;
};

}  // namespace amr

#endif  // CORE_CBASE_TEXT_H
