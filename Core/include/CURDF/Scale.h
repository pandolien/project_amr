#ifndef CORE_CURDF_SCALE_H
#define CORE_CURDF_SCALE_H

#include "CURDF/Urdf.h"

// 로봇 전체를 s 배로 키우거나 줄인다 (모든 길이가 s 배, 모양은 그대로).
//   길이     : 관절/시각/충돌/관성 원점의 이동, 상자 변, 원통/구 반지름과 길이, 메시 scale, 이동 관절의 한계(lower, upper)
//   질량     : massScales 가 true 이면 같은 밀도로 만든 것으로 보고 s^3 배, 관성 모멘트는 s^5 배 (false 이면 질량과 관성은 그대로)
//   힘/토크  : massScales 일 때 effort, damping, friction 을 질량 변화에 맞춘다 (회전 관절 s^4, 이동 관절 s^3). 이동 관절의 속도 한계는 s 배.
//   그대로   : 회전 관절의 각도, 축 방향, 색, 라이다의 측정 범위(센서 사양이므로)
// s <= 0 이면 false.
namespace simul {
namespace urdf {

bool ScaleRobot(const Robot& in, double s, bool massScales, Robot* out);

}  // namespace urdf
}  // namespace simul

#endif  // CORE_CURDF_SCALE_H
