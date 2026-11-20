#include "CEnv/Sun.h"

#include <math.h>
#include <time.h>

#include "CBase/Math.h"

namespace simul {
namespace env {

namespace {

const double kPi = 3.14159265358979323846;
double Rad(double d) { return d * kPi / 180.0; }
double Deg(double r) { return r * 180.0 / kPi; }

bool IsLeap(int y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }

int DaysInMonth(int y, int m) {
    static const int d[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    return (m == 2 && IsLeap(y)) ? 29 : d[m - 1];
}

// 그레고리력 날짜 -> 줄리안 일수 (정오 기준이 아니라 0시 기준: 해당 날 0시 UT 의 JD, 소수 .5 로 끝난다)
double JulianDay0(int year, int month, int day) {
    int y = year, m = month;
    if (m <= 2) {
        y -= 1;
        m += 12;
    }
    const int a = y / 100;
    const int b = 2 - a + a / 4;
    return floor(365.25 * (y + 4716)) + floor(30.6001 * (m + 1)) + day + b - 1524.5;
}

double Mod(double x, double m) {
    double r = fmod(x, m);
    if (r < 0) r += m;
    return r;
}

struct SolarCoords {
    double decl;   // 적위 (rad)
    double eqTime; // 균시차 (분)
};

// 줄리안 세기 t (J2000 기준) 에서 태양의 적위와 균시차 (NOAA 계산식)
SolarCoords SolarAt(double t) {
    const double L0 = Mod(280.46646 + t * (36000.76983 + t * 0.0003032), 360.0);
    const double M = 357.52911 + t * (35999.05029 - 0.0001537 * t);
    const double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
    const double Mr = Rad(M);
    const double C = sin(Mr) * (1.914602 - t * (0.004817 + 0.000014 * t)) + sin(2 * Mr) * (0.019993 - 0.000101 * t) + sin(3 * Mr) * 0.000289;
    const double trueLong = L0 + C;
    const double omega = 125.04 - 1934.136 * t;
    const double lambda = trueLong - 0.00569 - 0.00478 * sin(Rad(omega));
    const double eps0 = 23.0 + (26.0 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60.0) / 60.0;
    const double eps = eps0 + 0.00256 * cos(Rad(omega));
    SolarCoords s;
    s.decl = asin(sin(Rad(eps)) * sin(Rad(lambda)));
    const double y = tan(Rad(eps) / 2.0) * tan(Rad(eps) / 2.0);
    const double L0r = Rad(L0);
    s.eqTime = 4.0 * Deg(y * sin(2 * L0r) - 2.0 * e * sin(Mr) + 4.0 * e * y * sin(Mr) * cos(2 * L0r) - 0.5 * y * y * sin(4 * L0r) - 1.25 * e * e * sin(2 * Mr));
    return s;
}

}  // namespace

bool IsValidSunTime(const SunTime& t, String* error) {
    String why;
    if (t.year < 1800 || t.year > 2200) why = "연도는 1800 ~ 2200 이어야 합니다";
    else if (t.month < 1 || t.month > 12) why = "월은 1 ~ 12 이어야 합니다";
    else if (t.day < 1 || t.day > DaysInMonth(t.year, t.month)) why = "일이 그 달의 날짜 범위를 벗어났습니다";
    else if (t.hour < 0 || t.hour > 23) why = "시는 0 ~ 23 이어야 합니다";
    else if (t.minute < 0 || t.minute > 59) why = "분은 0 ~ 59 이어야 합니다";
    else if (!(t.second >= 0.0 && t.second < 60.0)) why = "초는 0 이상 60 미만이어야 합니다";
    else if (!(t.utcOffsetHours >= -14.0 && t.utcOffsetHours <= 14.0)) why = "UTC 오프셋은 -14 ~ +14 시간이어야 합니다";
    if (why.empty()) return true;
    if (error) *error = why;
    return false;
}

bool ComputeSun(const SunTime& time, const SunSite& site, SunState* out, String* error) {
    if (!IsValidSunTime(time, error)) return false;
    if (!(site.latitudeDeg >= -90.0 && site.latitudeDeg <= 90.0)) {
        if (error) *error = "위도는 -90 ~ 90 도여야 합니다";
        return false;
    }
    if (!(site.longitudeDeg >= -180.0 && site.longitudeDeg <= 180.0)) {
        if (error) *error = "경도는 -180 ~ 180 도여야 합니다";
        return false;
    }
    // 현지 시각 -> UTC (분 단위 하루 안 + 날짜는 JD 에 포함)
    const double localMin = time.hour * 60.0 + time.minute + time.second / 60.0;
    const double utcMin = localMin - time.utcOffsetHours * 60.0;
    const double jd = JulianDay0(time.year, time.month, time.day) + utcMin / 1440.0;
    const double t = (jd - 2451545.0) / 36525.0;
    const SolarCoords sc = SolarAt(t);

    const double minuteOfDayUtc = Mod(utcMin, 1440.0);
    const double trueSolarTime = Mod(minuteOfDayUtc + sc.eqTime + 4.0 * site.longitudeDeg, 1440.0);
    const double H = Rad(trueSolarTime / 4.0 - 180.0);   // 시간각: 정오 0, 오전 -, 오후 +
    const double lat = Rad(site.latitudeDeg);
    double sinEl = sin(lat) * sin(sc.decl) + cos(lat) * cos(sc.decl) * cos(H);
    sinEl = sinEl < -1.0 ? -1.0 : (sinEl > 1.0 ? 1.0 : sinEl);
    const double el = asin(sinEl);
    // 방위각: 북에서 시계 방향. (동서 성분 sinH, 남북 성분을 atan2 로)
    const double azFromSouth = atan2(sin(H), cos(H) * sin(lat) - tan(sc.decl) * cos(lat));   // 남쪽 기준, 서쪽으로 +
    const double az = Mod(azFromSouth + kPi, 2.0 * kPi);

    out->azimuthDeg = Deg(az);
    out->elevationDeg = Deg(el);
    out->declinationDeg = Deg(sc.decl);
    out->equationOfTimeMin = sc.eqTime;
    out->aboveHorizon = el > 0.0;
    // 동, 북, 위 성분 -> 월드 (월드 +y 의 방위를 northBearing 으로 두고 돌린다)
    const double e = sin(az) * cos(el), n = cos(az) * cos(el), u = sin(el);
    const double a = Rad(site.northBearingDeg);
    out->direction = dhVector(static_cast<float>(e * cos(a) - n * sin(a)), static_cast<float>(e * sin(a) + n * cos(a)), static_cast<float>(u));
    return true;
}

SunTime SunTimeNow() {
    SunTime s;
    const time_t now = ::time(0);
    struct tm lt, gt;
#ifdef _WIN32
    localtime_s(&lt, &now);
    gmtime_s(&gt, &now);
#else
    localtime_r(&now, &lt);
    gmtime_r(&now, &gt);
#endif
    s.year = lt.tm_year + 1900;
    s.month = lt.tm_mon + 1;
    s.day = lt.tm_mday;
    s.hour = lt.tm_hour;
    s.minute = lt.tm_min;
    s.second = lt.tm_sec;
    // 현지 - UTC (분). 날짜가 다르면 하루(1440분)를 보정한다
    int diff = (lt.tm_hour - gt.tm_hour) * 60 + (lt.tm_min - gt.tm_min);
    const int dayDiff = (lt.tm_year - gt.tm_year) * 366 + (lt.tm_yday - gt.tm_yday);
    if (dayDiff > 1) diff -= 1440;      // 연말/연초 경계: 현지가 다음 해
    else if (dayDiff < -1) diff += 1440;
    else diff += dayDiff * 1440;
    s.utcOffsetHours = diff / 60.0;
    return s;
}

SunTime AddMinutes(const SunTime& time, double minutes) {
    SunTime r = time;
    double total = time.hour * 60.0 + time.minute + time.second / 60.0 + minutes;
    int dayShift = static_cast<int>(floor(total / 1440.0));
    total -= dayShift * 1440.0;
    int h = static_cast<int>(total / 60.0);
    double rest = total - h * 60.0;
    int m = static_cast<int>(rest);
    double sec = (rest - m) * 60.0;
    if (sec >= 59.99999) { sec = 0.0; m += 1; }   // 부동소수 오차로 59.9999.. 가 나오면 올린다
    if (m >= 60) { m -= 60; h += 1; }
    if (h >= 24) { h -= 24; dayShift += 1; }
    r.hour = h;
    r.minute = m;
    r.second = sec;
    int y = time.year, mo = time.month, d = time.day;
    while (dayShift > 0) {
        d++;
        if (d > DaysInMonth(y, mo)) { d = 1; mo++; if (mo > 12) { mo = 1; y++; } }
        dayShift--;
    }
    while (dayShift < 0) {
        d--;
        if (d < 1) { mo--; if (mo < 1) { mo = 12; y--; } d = DaysInMonth(y, mo); }
        dayShift++;
    }
    r.year = y;
    r.month = mo;
    r.day = d;
    return r;
}

bool SunriseSunset(const SunTime& date, const SunSite& site, double* sunriseMinute, double* sunsetMinute, bool* polarDay) {
    // 해당 날 현지 정오의 적위/균시차로 일출/일몰 시간각을 구한다 (태양 중심이 고도 -0.833도: 굴절 + 시반지름)
    SunTime noon = date;
    noon.hour = 12;
    noon.minute = 0;
    noon.second = 0.0;
    if (!IsValidSunTime(noon, 0)) return false;
    const double jd = JulianDay0(noon.year, noon.month, noon.day) + (12.0 * 60.0 - noon.utcOffsetHours * 60.0) / 1440.0;
    const SolarCoords sc = SolarAt((jd - 2451545.0) / 36525.0);
    const double lat = Rad(site.latitudeDeg);
    const double cosH = (cos(Rad(90.833)) - sin(lat) * sin(sc.decl)) / (cos(lat) * cos(sc.decl));
    if (polarDay) *polarDay = cosH < -1.0;
    if (cosH < -1.0 || cosH > 1.0) return false;
    const double H = Deg(acos(cosH));   // 도
    const double solarNoonUtc = 720.0 - 4.0 * site.longitudeDeg - sc.eqTime;   // UTC 분
    const double off = date.utcOffsetHours * 60.0;
    if (sunriseMinute) *sunriseMinute = solarNoonUtc - 4.0 * H + off;
    if (sunsetMinute) *sunsetMinute = solarNoonUtc + 4.0 * H + off;
    return true;
}

}  // namespace env
}  // namespace simul
