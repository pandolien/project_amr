#include "CRL/Env.h"

namespace simul {
namespace rl {

EvalResult Evaluate(Env* env, const Policy& policy, int episodes, Rng* rng) {
    EvalResult r;
    if (episodes <= 0) return r;
    double sumReward = 0.0, sumSteps = 0.0;
    int successes = 0;
    for (int ep = 0; ep < episodes; ep++) {
        int s = env->Reset(rng);
        for (;;) {
            StepResult sr;
            s = env->Step(policy.Act(s), &sr);
            sumReward += sr.reward;
            sumSteps += 1.0;
            if (sr.Done()) break;
        }
        if (env->Succeeded()) successes++;
    }
    r.episodes = episodes;
    r.meanReward = sumReward / episodes;
    r.meanSteps = sumSteps / episodes;
    r.successRate = static_cast<double>(successes) / episodes;
    return r;
}

}  // namespace rl
}  // namespace simul
