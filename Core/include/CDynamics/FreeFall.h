#ifndef CORE_CDYNAMICS_FREEFALL_H
#define CORE_CDYNAMICS_FREEFALL_H

namespace simul {
namespace dynamics {

// 바닥(높이 0 인 수평면) 위에서 중력을 받아 떨어지는 물체 하나의 수직 운동 (회전 없음).
// height 는 물체의 가장 낮은 점의 바닥 위 높이(m), velocity 는 위쪽이 + (m/s).
// 한 스텝은 등가속도 운동의 정확한 식(h += v dt + g dt^2 / 2, v += g dt)을 쓰고, 바닥에 닿으면 튕김 계수만큼 되튄다.
// 속도가 충분히 작아지면 멈춘 것으로 본다. 물체가 이미 바닥 아래(height < 0)에 있으면 건드리지 않는다.
struct FallBody {
    double height;
    double velocity;
    double restitution;   // 0 = 튕기지 않음, 1 = 완전 탄성. 기본 0.3
    double restSpeed;     // 바닥에 닿을 때 이 속도(m/s) 미만이면 멈춘다. 기본 0.4
    FallBody() : height(0), velocity(0), restitution(0.3), restSpeed(0.4) {}

    // gz: 중력 가속도의 수직 성분 (m/s^2, 아래로 떨어지면 음수. 0 이상이면 떨어지지 않는다). 아직 움직이는 중이면 true.
    // dt 는 0 보다 커야 하고, 한 번에 너무 길면(> 0.1 s) 0.1 s 로 자른다.
    bool Step(double dt, double gz);
    bool AtRest() const { return height <= 0.0 && velocity <= 0.0; }
};

}  // namespace dynamics
}  // namespace simul

#endif  // CORE_CDYNAMICS_FREEFALL_H
