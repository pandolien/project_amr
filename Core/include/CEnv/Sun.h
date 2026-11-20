#ifndef CORE_CENV_SUN_H
#define CORE_CENV_SUN_H

#include "3d/dhVector.h"
#include "CBase/String.h"

// 태양 위치: 날짜/시각과 위도/경도로 태양의 방위각과 고도를 계산한다 (NOAA 태양 위치 계산식, 줄리안 세기 기반, 오차 약 0.01~0.05도).
// 대기 굴절은 넣지 않았다 (고도가 낮을 때 실제 보이는 태양은 최대 약 0.6도 더 높다).
//
// 월드 좌표계와 방위: 월드는 +z 가 위다. +y 가 가리키는 방위 northBearingDeg 를 정하면 되고 (기본 0 = +y 가 정북, +x 가 정동 = ROS 의 ENU),
// 방위는 북에서 시계 방향으로 잰 각도다 (동 = 90도). 예를 들어 월드 +x 가 정북이면 +y 는 정서이므로 northBearingDeg = 270.
namespace simul {
namespace env {

struct SunTime {
    int year, month, day;     // 현지 날짜 (그레고리력)
    int hour, minute;         // 현지 시각 (24시간제)
    double second;
    double utcOffsetHours;    // 현지 시각 = UTC + 이 값 (한국 +9, 서머타임이면 그 값이 반영된 오프셋)
    SunTime() : year(2000), month(1), day(1), hour(12), minute(0), second(0.0), utcOffsetHours(0.0) {}
};

struct SunSite {
    double latitudeDeg;       // 북위 +, 남위 -  (-90 ~ 90)
    double longitudeDeg;      // 동경 +, 서경 -  (-180 ~ 180)
    double northBearingDeg;   // 월드 +y 가 가리키는 방위 (북에서 시계 방향, 0 이면 +y 가 북)
    SunSite() : latitudeDeg(37.5665), longitudeDeg(126.9780), northBearingDeg(0.0) {}  // 기본: 서울
};

struct SunState {
    double azimuthDeg;        // 방위각: 북에서 시계 방향 (동 90, 남 180, 서 270)
    double elevationDeg;      // 고도: 수평선 위 +, 아래 -
    double declinationDeg;    // 태양의 적위
    double equationOfTimeMin; // 균시차 (분)
    dhVector direction;       // 월드 좌표에서 표면 -> 태양 방향 단위 벡터
    bool aboveHorizon;
    SunState() : azimuthDeg(0), elevationDeg(0), declinationDeg(0), equationOfTimeMin(0), aboveHorizon(false) {}
};

// 계산. 날짜/시각/위도/경도가 올바르지 않으면 false 와 이유를 돌려준다.
bool ComputeSun(const SunTime& time, const SunSite& site, SunState* out, String* error = 0);

// 올바른 날짜인가 (윤년 포함). 월 1~12, 일 1~그 달의 일수, 시 0~23, 분 0~59, 초 0 이상 60 미만
bool IsValidSunTime(const SunTime& time, String* error = 0);

// 현재 컴퓨터 시각 (현지 시간과 UTC 오프셋을 운영체제에서 읽는다)
SunTime SunTimeNow();

// 분 단위로 더한다 (날짜/월/해가 넘어가면 맞춰준다). 시간 흐름(타임랩스)에 쓴다.
SunTime AddMinutes(const SunTime& time, double minutes);

// 해가 뜨는/지는 시각 (현지 시각, 분 단위 하루 안의 값, 0 ~ 1440). 극지방처럼 뜨지/지지 않으면 false (polarDay: 해가 안 지는지).
bool SunriseSunset(const SunTime& date, const SunSite& site, double* sunriseMinute, double* sunsetMinute, bool* polarDay = 0);

}  // namespace env
}  // namespace simul

#endif  // CORE_CENV_SUN_H
