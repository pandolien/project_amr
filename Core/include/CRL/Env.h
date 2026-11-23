// 강화학습 환경(Env) 인터페이스와 공통 도구.
//
//   에이전트 --행동(action)--> 환경
//   에이전트 <--상태(state), 보상(reward), 끝났는가--  환경
//
// 이 모듈은 "이산 상태 + 이산 행동" (표 형태) 만 다룬다. 상태는 0 .. NumStates()-1 의 번호다.
#ifndef CORE_CRL_ENV_H
#define CORE_CRL_ENV_H

#include "CBase/Array.h"
#include "CRL/Random.h"

namespace simul {
namespace rl {

struct StepResult {
    double reward;
    bool terminal;    // 진짜 끝(목표 도달). 이 뒤의 미래 보상은 0 이다
    bool truncated;   // 시간 제한으로 끊김. 미래가 없는 것이 아니라 그냥 멈춘 것이다
    StepResult() : reward(0.0), terminal(false), truncated(false) {}
    bool Done() const { return terminal || truncated; }
};

class Env {
public:
    virtual ~Env() {}
    virtual int NumStates() const = 0;
    virtual int NumActions() const = 0;
    // 에피소드를 새로 시작하고 처음 상태를 돌려준다
    virtual int Reset(Rng* rng) = 0;
    // 행동 하나를 실행하고 다음 상태를 돌려준다. 보상과 종료 여부는 result 에 채운다
    virtual int Step(int action, StepResult* result) = 0;
    // 방금 끝난 에피소드가 목표 도달로 끝났는가
    virtual bool Succeeded() const { return false; }
};

// 상태 -> 행동을 정하는 것 (학습된 결과)
class Policy {
public:
    virtual ~Policy() {}
    virtual int Act(int state) const = 0;
};

struct EpisodeLog {
    double totalReward;
    int steps;
    bool success;
    EpisodeLog() : totalReward(0.0), steps(0), success(false) {}
};

struct EvalResult {
    int episodes;
    double meanReward;
    double meanSteps;
    double successRate;   // 0 .. 1
    EvalResult() : episodes(0), meanReward(0.0), meanSteps(0.0), successRate(0.0) {}
};

// 학습 없이 정책만 따라가서 성적을 잰다
EvalResult Evaluate(Env* env, const Policy& policy, int episodes, Rng* rng);

}  // namespace rl
}  // namespace simul

#endif  // CORE_CRL_ENV_H
