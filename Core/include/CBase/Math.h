#ifndef CORE_CBASE_MATH_H
#define CORE_CBASE_MATH_H

#include <float.h>
#include <math.h>

// 수학 함수 (sqrt 등 대체). float 과 double 중 인자 타입에 맞는 것이 불리고, 결과 타입도 같다.
// C 라이브러리(math.h)의 함수를 그대로 감싼 것이다.
namespace amr {

const float kFloatMax = FLT_MAX;
const double kDoubleMax = DBL_MAX;

inline float Abs(float x) { return fabsf(x); }
inline double Abs(double x) { return fabs(x); }
inline float Sqrt(float x) { return sqrtf(x); }
inline double Sqrt(double x) { return sqrt(x); }
inline float Sin(float x) { return sinf(x); }
inline double Sin(double x) { return sin(x); }
inline float Cos(float x) { return cosf(x); }
inline double Cos(double x) { return cos(x); }
inline float Tan(float x) { return tanf(x); }
inline double Tan(double x) { return tan(x); }
inline float Asin(float x) { return asinf(x); }
inline double Asin(double x) { return asin(x); }
inline float Acos(float x) { return acosf(x); }
inline double Acos(double x) { return acos(x); }
inline float Atan2(float y, float x) { return atan2f(y, x); }
inline double Atan2(double y, double x) { return atan2(y, x); }
inline float Pow(float x, float y) { return powf(x, y); }
inline double Pow(double x, double y) { return pow(x, y); }
inline float Exp(float x) { return expf(x); }
inline double Exp(double x) { return exp(x); }
inline float Log(float x) { return logf(x); }
inline double Log(double x) { return log(x); }
inline float Log10(float x) { return log10f(x); }
inline double Log10(double x) { return log10(x); }
inline float Floor(float x) { return floorf(x); }
inline double Floor(double x) { return floor(x); }
inline float Ceil(float x) { return ceilf(x); }
inline double Ceil(double x) { return ceil(x); }
inline float Tanh(float x) { return tanhf(x); }
inline double Tanh(double x) { return tanh(x); }
inline float Fmod(float x, float y) { return fmodf(x, y); }
inline double Fmod(double x, double y) { return fmod(x, y); }

// 유한한 수인가 (NaN 도 무한대도 아님). 비교만 쓰므로 컴파일러의 매크로/오버로드 차이가 없다.
inline bool IsFinite(double x) { return x == x && x - x == 0.0; }
inline bool IsFinite(float x) { return x == x && x - x == 0.0f; }
inline bool IsNan(double x) { return x != x; }
inline bool IsNan(float x) { return x != x; }

// 가장 가까운 정수로 반올림 (0.5 는 0 에서 먼 쪽). 범위를 넘으면 정의되지 않는다.
inline long Lround(double x) { return static_cast<long>(x >= 0.0 ? floor(x + 0.5) : -floor(-x + 0.5)); }

// 안 쓰는 값이 아니라 "숫자가 아님" 을 나타내는 float (테스트용)
inline float NanFloat() {
    volatile float zero = 0.0f;
    return zero / zero;
}

}  // namespace amr

#endif  // CORE_CBASE_MATH_H
