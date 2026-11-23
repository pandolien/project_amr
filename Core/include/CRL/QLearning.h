// 표 형태 Q-러닝 (가치 기반, off-policy).
//
//   Q(s,a) = "상태 s 에서 행동 a 를 하고 그 뒤로 잘 하면 앞으로 받을 보상의 합(할인)"
//   갱신:  Q(s,a) += alpha * ( r + gamma * max_a' Q(s',a') - Q(s,a) )     (끝났으면 max 항은 0)
//   행동 선택: 확률 epsilon 으로 무작위(탐험), 아니면 Q 가 가장 큰 행동(활용)
#ifndef CORE_CRL_QLEARNING_H
#define CORE_CRL_QLEARNING_H

#include "CRL/Env.h"

namespace simul {
namespace rl {

struct QLearningConfig {
    double alpha;         // 학습률
    double gamma;         // 할인율 (0 에 가까우면 눈앞 보상만, 1 에 가까우면 먼 미래까지)
    double epsilon;       // 시작 탐험 확률
    double epsilonMin;
    double epsilonDecay;  // 에피소드가 끝날 때마다 epsilon *= epsilonDecay
    QLearningConfig() : alpha(0.3), gamma(0.95), epsilon(1.0), epsilonMin(0.05), epsilonDecay(0.995) {}
};

class QLearning : public Policy {
public:
    QLearning(int numStates, int numActions, const QLearningConfig& config = QLearningConfig());

    int SelectAction(int state, Rng* rng) const;   // epsilon-greedy
    int Act(int state) const override;             // 탐험 없이 가장 좋은 행동
    void Update(int state, int action, double reward, int nextState, bool terminal);
    void EndEpisode();                             // epsilon 을 줄인다

    double Q(int state, int action) const { return q_[idx(state, action)]; }
    double MaxQ(int state) const;
    double Epsilon() const { return epsilon_; }

private:
    size_t idx(int s, int a) const { return static_cast<size_t>(s * nActions_ + a); }
    int nStates_, nActions_;
    QLearningConfig cfg_;
    double epsilon_;
    Array<double> q_;
};

// 환경에서 episodes 번 학습한다. log 를 주면 에피소드마다 기록한다.
void TrainQLearning(Env* env, QLearning* agent, int episodes, Rng* rng, Array<EpisodeLog>* log = nullptr);

}  // namespace rl
}  // namespace simul

#endif  // CORE_CRL_QLEARNING_H
