#include "CRL/QLearning.h"

namespace simul {
namespace rl {

QLearning::QLearning(int numStates, int numActions, const QLearningConfig& config)
    : nStates_(numStates), nActions_(numActions), cfg_(config), epsilon_(config.epsilon),
      q_(static_cast<size_t>(numStates * numActions), 0.0) {}

int QLearning::Act(int state) const {
    int best = 0;
    for (int a = 1; a < nActions_; a++)
        if (Q(state, a) > Q(state, best)) best = a;
    return best;
}

int QLearning::SelectAction(int state, Rng* rng) const {
    if (rng->Uniform() < epsilon_) return rng->UniformInt(nActions_);
    return Act(state);
}

double QLearning::MaxQ(int state) const { return Q(state, Act(state)); }

void QLearning::Update(int state, int action, double reward, int nextState, bool terminal) {
    const double future = terminal ? 0.0 : cfg_.gamma * MaxQ(nextState);
    double& q = q_[idx(state, action)];
    q += cfg_.alpha * (reward + future - q);
}

void QLearning::EndEpisode() {
    epsilon_ *= cfg_.epsilonDecay;
    if (epsilon_ < cfg_.epsilonMin) epsilon_ = cfg_.epsilonMin;
}

void TrainQLearning(Env* env, QLearning* agent, int episodes, Rng* rng, Array<EpisodeLog>* log) {
    for (int ep = 0; ep < episodes; ep++) {
        int s = env->Reset(rng);
        EpisodeLog entry;
        for (;;) {
            const int a = agent->SelectAction(s, rng);
            StepResult sr;
            const int s2 = env->Step(a, &sr);
            // 시간 제한(truncated)으로 끊긴 것은 "끝"이 아니므로 terminal 만 미래를 0 으로 본다
            agent->Update(s, a, sr.reward, s2, sr.terminal);
            entry.totalReward += sr.reward;
            entry.steps++;
            s = s2;
            if (sr.Done()) break;
        }
        entry.success = env->Succeeded();
        agent->EndEpisode();
        if (log) log->push_back(entry);
    }
}

}  // namespace rl
}  // namespace simul
