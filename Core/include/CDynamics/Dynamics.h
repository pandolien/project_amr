#ifndef CORE_CDYNAMICS_DYNAMICS_H
#define CORE_CDYNAMICS_DYNAMICS_H

#include "CBase/String.h"
#include "CBase/Array.h"

#include "CDynamics/Gravity.h"
#include "CURDF/Urdf.h"

// 로봇 팔의 동역학: 운동방정식  M(q) q'' + h(q, q') = tau - (감쇠, 마찰)   (h = 코리올리/원심력 + 중력 g(q))
// 좌표는 구동 관절(Robot::ActuatedJoints: 움직이는 관절 중 mimic 이 아닌 것)이다. mimic 관절은 q_mimic = mult * q_master + offset
// 로 마스터를 따라가므로 가상일의 원리로 마스터에 합쳐진다. 계산은 모두 double, 단위는 SI (rad, m, kg, N, N·m, 초).
// 좌표계는 루트 링크 좌표계이고 중력 벡터도 그 좌표계로 준다 (Gravity 와 같다).
// 필요한 정보: 링크 <inertial> 의 mass, 질량중심(origin), 관성 텐서. 없는 링크는 질량 0 으로 본다.
namespace simul {
namespace dynamics {

// 구동 관절의 이름 (Robot::ActuatedJoints 순서)
simul::Array<simul::String> ActuatedJointNames(const urdf::Robot& robot);

// 역동역학 (재귀 뉴턴-오일러, RNEA): 주어진 관절 값 q, 속도 qd, 가속도 qdd 를 만들려면 각 구동 관절이 내야 하는 토크/힘
//   = M(q) qdd + C(q, qd) qd + g(q).  qd, qdd 에 없는 관절은 0. 감쇠와 마찰은 포함하지 않는다.
//   qdd = 0, qd = 0 이면 GravityTorques 와 같다.
JointTorques InverseDynamics(const urdf::Robot& robot, const urdf::JointValues& q, const JointTorques& qd, const JointTorques& qdd,
                             const Gravity& g = Gravity(), bool clampToLimits = true);

// 질량 행렬 M(q) (구동 관절 기준, 대칭 양의 정부호). m 은 n x n 행 우선.
struct MassMatrix {
    simul::Array<simul::String> joints;
    simul::Array<double> m;
    size_t n() const { return joints.size(); }
    double At(size_t i, size_t j) const { return m[i * joints.size() + j]; }
};
MassMatrix ComputeMassMatrix(const urdf::Robot& robot, const urdf::JointValues& q, bool clampToLimits = true);

// 관절 마찰: 점성 감쇠 d*qd 와 쿨롱 마찰 f*tanh(qd/frictionSmoothing). d, f 는 URDF <dynamics> 의 damping, friction.
// 감쇠가 없는(0) 관절에는 defaultDamping 을 쓴다 (모터가 꺼진 팔이 영원히 흔들리지 않도록. 기본 0 = 쓰지 않음).
struct DynamicsOptions {
    double defaultDamping;     // N·m·s/rad 또는 N·s/m
    double frictionSmoothing;  // m/s 또는 rad/s. 쿨롱 마찰의 부드러운 정도
    DynamicsOptions() : defaultDamping(0.0), frictionSmoothing(0.01) {}
};

// 순동역학: 모터 토크 tau (구동 관절, 없는 관절은 0)를 줬을 때의 구동 관절 가속도.
//   M qdd = tau - h(q, qd) - 마찰.  질량(관성)이 0 인 관절은 가속도 0 으로 둔다.
JointTorques ForwardDynamics(const urdf::Robot& robot, const urdf::JointValues& q, const JointTorques& qd, const JointTorques& tau,
                             const Gravity& g = Gravity(), const DynamicsOptions& opt = DynamicsOptions());

// 시뮬레이터: 구동 관절의 (q, qd) 를 룽게-쿠타(RK4)로 적분한다. 관절 limit 에 닿으면 멈추고(속도 0), continuous 관절은 제한이 없다.
// 모터 토크 기본값은 0 (모터 꺼짐 = 중력에 처진다). SetTorques 로 일정한 토크를 줄 수 있다.
class Simulator {
public:
    Simulator();
    void Reset(const urdf::Robot& robot, const urdf::JointValues& q);  // 속도는 0 으로
    // 로봇의 배치만 바뀌었다 (모델을 옮기거나 돌려서 attach 관절의 origin 이 달라졌다): 관절 값과 속도는 이름으로 유지하고
    // 기구학을 새 로봇으로 바꾼다. 중력은 항상 루트(world) 좌표계 기준이므로, 돌린 모델도 바로 새 방향으로 중력을 받는다.
    // 구동 관절 구성이 달라졌으면 Reset 과 같게 처음부터 시작한다.
    void UpdateRobot(const urdf::Robot& robot);
    bool IsReady() const { return ready_; }
    void SetGravity(const Gravity& g) { gravity_ = g; }
    void SetOptions(const DynamicsOptions& o) { options_ = o; }
    void SetTorques(const JointTorques& tau) { tau_ = tau; }
    // 사용자가 관절 값을 직접 바꿨다: 그 관절의 속도는 0 으로 한다 (limit 안으로 제한)
    void SetValue(const simul::String& joint, double value);
    void Step(double dt);  // dt 초만큼 진행. 내부에서 1/500 초 이하로 나눈다
    urdf::JointValues Values() const;     // 구동 관절 값 (mimic 은 포함하지 않음. Robot 이 계산한다)
    JointTorques Velocities() const;
    double KineticEnergy() const;
    double PotentialEnergy() const;       // 중력 위치에너지 (기준은 임의)
    double TotalEnergy() const { return KineticEnergy() + PotentialEnergy(); }

private:
    void derivative(const simul::Array<double>& q, const simul::Array<double>& qd, simul::Array<double>* dq, simul::Array<double>* dqd) const;
    urdf::JointValues toValues(const simul::Array<double>& q) const;
    urdf::Robot robot_;
    simul::Array<int> act_;
    simul::Array<double> q_, qd_;
    Gravity gravity_;
    DynamicsOptions options_;
    JointTorques tau_;
    bool ready_;
};

}  // namespace dynamics
}  // namespace simul

#endif  // CORE_CDYNAMICS_DYNAMICS_H
