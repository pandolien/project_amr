#include "CBase/Math.h"
#include "CRL/Reinforce.h"

namespace simul {
namespace rl {

Reinforce::Reinforce(int numStates, int numActions, const ReinforceConfig& config)
    : nStates_(numStates), nActions_(numActions), cfg_(config),
      theta_(static_cast<size_t>(numStates * numActions), 0.0),
      baseline_(static_cast<size_t>(numStates), 0.0) {}

void Reinforce::Probabilities(int state, double* out) const {
    double mx = theta_[idx(state, 0)];
    for (int a = 1; a < nActions_; a++)
        if (theta_[idx(state, a)] > mx) mx = theta_[idx(state, a)];
    double sum = 0.0;
    for (int a = 0; a < nActions_; a++) {
        out[a] = Exp(theta_[idx(state, a)] - mx);   // 최댓값을 빼서 overflow 방지
        sum += out[a];
    }
    for (int a = 0; a < nActions_; a++) out[a] /= sum;
}

int Reinforce::Sample(int state, Rng* rng) const {
    Array<double> p(static_cast<size_t>(nActions_), 0.0);
    Probabilities(state, &p[0]);
    double u = rng->Uniform();
    for (int a = 0; a < nActions_; a++) {
        u -= p[static_cast<size_t>(a)];
        if (u < 0.0) return a;
    }
    return nActions_ - 1;
}

int Reinforce::Act(int state) const {
    int best = 0;
    for (int a = 1; a < nActions_; a++)
        if (theta_[idx(state, a)] > theta_[idx(state, best)]) best = a;
    return best;
}

void Reinforce::UpdateEpisode(const Array<int>& states, const Array<int>& actions, const Array<double>& rewards) {
    const size_t n = states.size();
    Array<double> p(static_cast<size_t>(nActions_), 0.0);
    // 뒤에서부터 누적 보상 G 를 구하며 갱신한다 (G_t = r_t + gamma * G_{t+1})
    double g = 0.0;
    for (size_t k = n; k-- > 0;) {
        g = rewards[k] + cfg_.gamma * g;
        const int s = states[k], a = actions[k];
        const double advantage = g - baseline_[static_cast<size_t>(s)];
        baseline_[static_cast<size_t>(s)] += cfg_.beta * advantage;
        Probabilities(s, &p[0]);
        for (int b = 0; b < nActions_; b++)
            theta_[idx(s, b)] += cfg_.alpha * advantage * ((b == a ? 1.0 : 0.0) - p[static_cast<size_t>(b)]);
    }
}

void TrainReinforce(Env* env, Reinforce* agent, int episodes, Rng* rng, Array<EpisodeLog>* log) {
    for (int ep = 0; ep < episodes; ep++) {
        Array<int> states, actions;
        Array<double> rewards;
        int s = env->Reset(rng);
        EpisodeLog entry;
        for (;;) {
            const int a = agent->Sample(s, rng);
            StepResult sr;
            const int s2 = env->Step(a, &sr);
            states.push_back(s);
            actions.push_back(a);
            rewards.push_back(sr.reward);
            entry.totalReward += sr.reward;
            entry.steps++;
            s = s2;
            if (sr.Done()) break;
        }
        entry.success = env->Succeeded();
        agent->UpdateEpisode(states, actions, rewards);
        if (log) log->push_back(entry);
    }
}

}  // namespace rl
}  // namespace simul
