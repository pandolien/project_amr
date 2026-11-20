// CEnv 태양 위치 테스트: 알려진 값(하지/동지/춘분의 정오 고도, 방위, 일출/일몰), 날짜 계산, 월드 방향, 잘못된 입력
#include <math.h>
#include <stdio.h>

#include "CBase/Math.h"
#include "CEnv/Sun.h"

using namespace simul;
using namespace simul::env;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}
bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }

SunTime At(int y, int mo, int d, int h, int mi, double off) {
    SunTime t;
    t.year = y; t.month = mo; t.day = d; t.hour = h; t.minute = mi; t.second = 0; t.utcOffsetHours = off;
    return t;
}

// 서울 (37.5665 N, 126.978 E), 한국 표준시 +9
void testSeoulSolstices() {
    SunSite seoul;
    SunState s;
    String err;
    // 하지 2024-06-21: 남중 약 12:33 KST. 고도 = 90 - 37.5665 + 23.44 = 75.87, 방위 180
    check(ComputeSun(At(2024, 6, 21, 12, 34, 9), seoul, &s, &err), ("compute: " + err).c_str());
    check(near(s.elevationDeg, 75.87, 0.3), "summer solstice noon elevation in Seoul");
    check(near(s.azimuthDeg, 180.0, 3.0), "summer solstice noon: sun is due south");
    check(near(s.declinationDeg, 23.44, 0.05), "declination at the June solstice");
    check(s.aboveHorizon, "above horizon");
    check(near(s.direction.z, sin(s.elevationDeg * 3.14159265358979 / 180.0), 1e-5), "direction z = sin(elevation)");
    check(s.direction.y < 0.0, "north is +y by default, so a southern sun has negative y");
    // 동지 2024-12-21: 고도 = 90 - 37.5665 - 23.44 = 28.99
    check(ComputeSun(At(2024, 12, 21, 12, 24, 9), seoul, &s, &err), "compute winter");
    check(near(s.elevationDeg, 28.99, 0.4), "winter solstice noon elevation in Seoul");
    check(near(s.declinationDeg, -23.44, 0.05), "declination at the December solstice");
    // 춘분 즈음 (2024-03-20): 적위 ~0, 정오 고도 ~ 90 - 37.57 = 52.4
    check(ComputeSun(At(2024, 3, 20, 12, 30, 9), seoul, &s, &err), "compute equinox");
    check(near(s.elevationDeg, 52.4, 0.8), "equinox noon elevation in Seoul");
}

void testDayArc() {
    SunSite seoul;
    SunState am, pm, mid, night;
    check(ComputeSun(At(2024, 6, 21, 9, 0, 9), seoul, &am), "9:00");
    check(ComputeSun(At(2024, 6, 21, 16, 0, 9), seoul, &pm), "16:00");
    check(am.azimuthDeg > 60 && am.azimuthDeg < 120, "morning sun is in the east");
    check(pm.azimuthDeg > 240 && pm.azimuthDeg < 300, "afternoon sun is in the west");
    check(am.direction.x > 0.2 && pm.direction.x < -0.2, "east is +x in the default world");
    check(ComputeSun(At(2024, 6, 21, 0, 30, 9), seoul, &night), "00:30");
    check(!night.aboveHorizon && night.elevationDeg < -20, "the sun is below the horizon at night");
    check(ComputeSun(At(2024, 6, 21, 12, 34, 9), seoul, &mid), "noon");
    check(mid.elevationDeg > am.elevationDeg && mid.elevationDeg > pm.elevationDeg, "noon is the highest");
}

void testSouthernAndEquator() {
    SunState s;
    // 적도, 춘분 즈음 정오 (UTC, 경도 0): 거의 천정
    SunSite eq;
    eq.latitudeDeg = 0.0;
    eq.longitudeDeg = 0.0;
    check(ComputeSun(At(2024, 3, 20, 12, 0, 0), eq, &s), "equator");
    check(s.elevationDeg > 88.0, "equator at the equinox noon: near the zenith");
    // 남반구 (시드니 -33.87, 151.21) 12월 하지 즈음 정오: 태양은 북쪽 (방위 0 또는 360 근처)
    SunSite syd;
    syd.latitudeDeg = -33.87;
    syd.longitudeDeg = 151.21;
    check(ComputeSun(At(2024, 12, 21, 13, 0, 11), syd, &s), "sydney");   // 서머타임 +11
    check(s.azimuthDeg < 25.0 || s.azimuthDeg > 335.0, "southern hemisphere: the noon sun is in the north");
    check(s.elevationDeg > 75.0 && s.elevationDeg < 83.0, "sydney summer noon is high (about 79.5)");
}

// 월드 +y 가 가리키는 방위를 바꾸면 방향이 그만큼 돈다
void testNorthBearing() {
    SunSite a, b;
    b.northBearingDeg = 90.0;   // 월드 +y 가 정동 -> 정북은 월드 -x
    SunState sa, sb;
    const SunTime t = At(2024, 6, 21, 12, 34, 9);   // 남중: 태양은 정남
    check(ComputeSun(t, a, &sa) && ComputeSun(t, b, &sb), "north bearing");
    check(sa.direction.y < -0.2 && fabs(sa.direction.x) < 0.05, "default: the southern sun is on -y");
    check(sb.direction.x > 0.2 && fabs(sb.direction.y) < 0.05, "+y is east: south is +x");
    check(near(sa.direction.z, sb.direction.z, 1e-6), "height does not depend on the bearing");
    const double la = sqrt(sa.direction.x * sa.direction.x + sa.direction.y * sa.direction.y + sa.direction.z * sa.direction.z);
    check(near(la, 1.0, 1e-5), "direction is a unit vector");
}

void testSunriseSunset() {
    SunSite seoul;
    double rise = 0, set = 0;
    bool polar = false;
    check(SunriseSunset(At(2024, 6, 21, 12, 0, 9), seoul, &rise, &set, &polar), "sunrise/sunset in June");
    check(near(rise, 5 * 60 + 12, 12.0) && near(set, 19 * 60 + 57, 12.0), "Seoul on the June solstice: about 05:12 and 19:57");
    check(SunriseSunset(At(2024, 12, 21, 12, 0, 9), seoul, &rise, &set, &polar), "sunrise/sunset in December");
    check(near(rise, 7 * 60 + 43, 12.0) && near(set, 17 * 60 + 17, 12.0), "Seoul on the December solstice: about 07:43 and 17:17");
    SunSite arctic;
    arctic.latitudeDeg = 80.0;
    arctic.longitudeDeg = 0.0;
    check(!SunriseSunset(At(2024, 6, 21, 12, 0, 0), arctic, &rise, &set, &polar) && polar, "midnight sun at 80 N in June");
    check(!SunriseSunset(At(2024, 12, 21, 12, 0, 0), arctic, &rise, &set, &polar) && !polar, "polar night at 80 N in December");
}

void testAddMinutes() {
    SunTime t = At(2024, 2, 28, 23, 30, 9);
    SunTime u = AddMinutes(t, 60);
    check(u.year == 2024 && u.month == 2 && u.day == 29 && u.hour == 0 && u.minute == 30, "leap day rollover");
    u = AddMinutes(At(2023, 12, 31, 23, 59, 0), 2);
    check(u.year == 2024 && u.month == 1 && u.day == 1 && u.hour == 0 && u.minute == 1, "year rollover");
    u = AddMinutes(At(2024, 3, 1, 0, 10, 0), -20);
    check(u.month == 2 && u.day == 29 && u.hour == 23 && u.minute == 50, "going back across a month");
    u = AddMinutes(At(2024, 1, 1, 12, 0, 0), 24 * 60 * 60.0);
    check(u.year == 2024 && u.month == 3 && u.day == 1, "60 days after Jan 1 2024 is Mar 1");
    check(IsValidSunTime(u), "result is valid");
}

void testErrors() {
    SunSite s;
    SunState st;
    String err;
    check(!ComputeSun(At(2023, 2, 29, 12, 0, 0), s, &st, &err) && !err.empty(), "Feb 29 2023 is rejected");
    check(!ComputeSun(At(2024, 13, 1, 12, 0, 0), s, &st, &err), "month 13");
    check(!ComputeSun(At(2024, 1, 1, 24, 0, 0), s, &st, &err), "hour 24");
    check(!ComputeSun(At(2024, 1, 1, 12, 60, 0), s, &st, &err), "minute 60");
    check(!ComputeSun(At(2024, 1, 1, 12, 0, 20), s, &st, &err), "offset 20");
    s.latitudeDeg = 91.0;
    check(!ComputeSun(At(2024, 1, 1, 12, 0, 0), s, &st, &err), "latitude 91");
    s.latitudeDeg = 0.0;
    s.longitudeDeg = 181.0;
    check(!ComputeSun(At(2024, 1, 1, 12, 0, 0), s, &st, &err), "longitude 181");
    volatile double zero = 0.0;   // 컴파일 시점 0 나누기는 MSVC 가 오류(C2124)로 막으므로 실행 시점에 NaN 을 만든다
    s.longitudeDeg = zero / zero;
    check(!ComputeSun(At(2024, 1, 1, 12, 0, 0), s, &st, &err), "NaN longitude");
    const SunTime now = SunTimeNow();
    check(IsValidSunTime(now), "the system clock gives a valid time");
}

}  // namespace

int main() {
    testSeoulSolstices();
    testDayArc();
    testSouthernAndEquator();
    testNorthBearing();
    testSunriseSunset();
    testAddMinutes();
    testErrors();
    if (g_fail) {
        printf("test_cenv: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_cenv: all passed\n");
    return 0;
}
