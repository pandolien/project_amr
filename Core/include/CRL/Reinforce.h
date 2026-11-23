// REINFORCE (정책 기울기, on-policy). 표 형태 softmax 정책.
//
//   정책 pi(a|s) = softmax(theta[s][:])   <- 행동의 "확률" 을 직접 학습한다
//   에피소드가 끝나면 각 시점 t 에서 G_t = r_t + gamma * G_{t+1} (그 뒤에 받은 보상 합)를 구하고,
//   advantage = G_t - V(s_t)   (V 는 기준선: 평균보다 얼마나 좋았나)
//   theta[s][b] += alpha * advantage * ( 1[b == a_t] - pi(b|s) )
//   -> 평균보다 좋았던 행동은 확률을 올리고, 나빴던 행동은 내린다.
#ifndef CORE_CRL_REINFORCE_H
#define CORE_CRL_REINFORCE_H

#include "CRL/Env.h"

namespace simul {
namespace rl {

struct ReinforceConfig {
    double alpha;   // 정책 학습률
    double beta;    // 기준선 V 학습률
    double gamma;
    ReinforceConfig() : alpha(0.02), beta(0.1), gamma(0.95) {}
};

class Reinforce : public Policy {
public:
    Reinforce(int numStates, int numActions, const ReinforceConfig& config = ReinforceConfig());

    void Probabilities(int state, double* out) const;   // out 은 numActions 개
    int Sample(int state, Rng* rng) const;              // 확률대로 뽑는다
    int Act(int state) const override;                  // 확률이 가장 큰 행동
    // 에피소드 하나(상태, 행동, 보상 기록)로 정책을 갱신한다
    void UpdateEpisode(const Array<int>& states, const Array<int>& actions, const Array<double>& rewards);

private:
    size_t idx(int s, int a) const { return static_cast<size_t>(s * nActions_ + a); }
    int nStates_, nActions_;
    ReinforceConfig cfg_;
    Array<double> theta_;
    Array<double> baseline_;
};

void TrainReinforce(Env* env, Reinforce* agent, int episodes, Rng* rng, Array<EpisodeLog>* log = nullptr);

}  // namespace rl
}  // namespace simul

#endif  // CORE_CRL_REINFORCE_H
