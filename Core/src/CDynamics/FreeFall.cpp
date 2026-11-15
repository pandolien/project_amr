#include "CDynamics/FreeFall.h"

#include <cmath>

namespace amr {
namespace dynamics {

bool FallBody::Step(double dt, double gz) {
    if (!(dt > 0.0) || !(gz < 0.0) || !std::isfinite(height) || !std::isfinite(velocity)) {
        velocity = 0.0;
        return false;
    }
    if (dt > 0.1) dt = 0.1;
    if (height < 0.0) {  // 바닥 아래: 건드리지 않는다
        velocity = 0.0;
        return false;
    }
    if (height == 0.0 && velocity <= 0.0) {  // 바닥에 놓여 있다
        velocity = 0.0;
        return false;
    }
    const double h = height + velocity * dt + 0.5 * gz * dt * dt;
    const double v = velocity + gz * dt;
    if (h > 0.0) {
        height = h;
        velocity = v;
        return true;
    }
    // 이번 스텝에 바닥에 닿았다
    height = 0.0;
    if (-v < restSpeed) {
        velocity = 0.0;
        return false;
    }
    velocity = -v * restitution;
    if (velocity < restSpeed * 0.5) {  // 튕겨 오른 속도가 너무 작으면 그대로 멈춘다
        velocity = 0.0;
        return false;
    }
    return true;
}

}  // namespace dynamics
}  // namespace amr
