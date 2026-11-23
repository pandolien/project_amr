// CRL 테스트: 난수 재현성, 시뮬레이션 기반 환경(ArmReachEnv), Q-러닝, REINFORCE
#include <stdio.h>

#include "CRL/ArmReachEnv.h"
#include "CRL/QLearning.h"
#include "CRL/Reinforce.h"

using namespace simul;
using namespace simul::rl;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

String ArmPath() { return String(SIMUL_TEST_DATA_DIR) + "/Object/samples/arm.urdf"; }

void testRng() {
    Rng a(42), b(42), c(43);
    bool same = true, differs = false, inRange = true;
    for (int i = 0; i < 100; i++) {
        const uint32_t x = a.NextU32(), y = b.NextU32(), z = c.NextU32();
        same = same && x == y;
        differs = differs || x != z;
    }
    for (int i = 0; i < 1000; i++) {
        const double u = a.Uniform();
        const int k = a.UniformInt(7);
        inRange = inRange && u >= 0.0 && u < 1.0 && k >= 0 && k < 7;
    }
    check(same, "same seed -> same sequence");
    check(differs, "different seed -> different sequence");
    check(inRange, "Uniform/UniformInt stay in range");
}

void testEnv() {
    ArmReachEnv env;
    String err;
    check(env.Init(ArmPath(), ArmReachConfig(), &err), "env init");
    check(env.NumStates() == 13 * 17 && env.NumActions() == 4, "state/action counts");

    ArmReachConfig bad;
    bad.jointA = "no_such_joint";
    ArmReachEnv env2;
    check(!env2.Init(ArmPath(), bad, &err) && !err.empty(), "unknown joint is rejected");

    Rng rng(1);
    const int s0 = env.Reset(&rng);
    check(s0 >= 0 && s0 < env.NumStates() && s0 != env.GoalState(), "reset gives a non-goal start state");
    // 한 칸 움직이는 행동: A+ 는 칸 번호 A 가 정확히 1 늘어난다 (경계가 아니면)
    for (int trial = 0; trial < 20; trial++) {
        const int s = env.Reset(&rng);
        StepResult r;
        const int s2 = env.Step(0, &r);
        const bool atEdge = env.BinA(s) == env.Config().binsA - 1;
        check(atEdge ? s2 == s : (env.BinA(s2) == env.BinA(s) + 1 && env.BinB(s2) == env.BinB(s)), "action A+ moves exactly one bin");
        check(r.reward < 0.0 || r.terminal, "reward is a cost unless the goal was reached");
    }
    // 가장자리에서 밖으로 나가는 행동은 제자리
    {
        ArmReachConfig cfg;
        cfg.goalA = 5;
        cfg.maxSteps = 1000;
        ArmReachEnv e;
        check(e.Init(ArmPath(), cfg, &err), "init (edge test)");
        Rng r2(3);
        int s = e.Reset(&r2);
        StepResult sr;
        for (int i = 0; i < 40; i++) s = e.Step(1, &sr);   // A- 를 계속 눌러 끝까지
        check(e.BinA(s) == 0, "A- repeatedly reaches bin 0");
        const int again = e.Step(1, &sr);
        check(again == s, "moving past the edge keeps the state");
    }
    // 목표 도달: 목표 바로 옆 칸에서 한 칸 가면 terminal + 성공
    {
        ArmReachConfig cfg;
        ArmReachEnv e;
        check(e.Init(ArmPath(), cfg, &err), "init (goal test)");
        // 목표 자세로의 거리는 목표 칸에서 0 이어야 한다: 목표의 한 칸 아래에서 시작하도록 반복 시도
        Rng r3(5);
        bool reached = false;
        for (int tries = 0; tries < 2000 && !reached; tries++) {
            int s = e.Reset(&r3);
            if (e.BinA(s) == cfg.goalA - 1 && e.BinB(s) == cfg.goalB) {
                StepResult sr;
                e.Step(0, &sr);
                check(sr.terminal && !sr.truncated && e.Succeeded() && e.DistanceToGoal() < cfg.goalTolerance, "stepping onto the goal ends the episode with success");
                reached = true;
            }
        }
        check(reached, "found a start next to the goal");
    }
    // 시간 제한은 terminal 이 아니라 truncated
    {
        ArmReachConfig cfg;
        cfg.maxSteps = 1;
        ArmReachEnv e;
        check(e.Init(ArmPath(), cfg, &err), "init (truncation test)");
        Rng r4(7);
        e.Reset(&r4);
        StepResult sr;
        e.Step(2, &sr);
        check(sr.truncated && !sr.terminal && !e.Succeeded(), "time limit is truncated, not terminal");
    }
}

void testQLearning() {
    ArmReachEnv env;
    String err;
    check(env.Init(ArmPath(), ArmReachConfig(), &err), "env init (q)");
    Rng rng(123);
    QLearning agent(env.NumStates(), env.NumActions());
    Array<EpisodeLog> log;
    TrainQLearning(&env, &agent, 1500, &rng, &log);
    check(log.size() == 1500, "one log entry per episode");
    check(agent.Epsilon() < 0.1, "epsilon decays");
    Rng evalRng(999);
    const EvalResult r = Evaluate(&env, agent, 100, &evalRng);
    printf("Q-learning: success=%.2f meanSteps=%.1f meanReward=%.2f\n", r.successRate, r.meanSteps, r.meanReward);
    check(r.successRate >= 0.95, "Q-learning policy reaches the goal (>= 95%)");

    // 같은 seed 면 같은 결과 (재현성)
    ArmReachEnv env2;
    check(env2.Init(ArmPath(), ArmReachConfig(), &err), "env init (q repeat)");
    Rng rng2(123);
    QLearning agent2(env2.NumStates(), env2.NumActions());
    TrainQLearning(&env2, &agent2, 1500, &rng2, nullptr);
    bool same = true;
    for (int s = 0; s < env.NumStates(); s++)
        for (int a = 0; a < env.NumActions(); a++) same = same && agent.Q(s, a) == agent2.Q(s, a);
    check(same, "same seed -> identical Q table");
}

void testReinforce() {
    ArmReachEnv env;
    String err;
    check(env.Init(ArmPath(), ArmReachConfig(), &err), "env init (reinforce)");
    Rng rng(321);
    Reinforce agent(env.NumStates(), env.NumActions());
    TrainReinforce(&env, &agent, 3000, &rng, nullptr);
    double p[4];
    agent.Probabilities(0, p);
    check(p[0] + p[1] + p[2] + p[3] > 0.999999 && p[0] + p[1] + p[2] + p[3] < 1.000001, "probabilities sum to 1");
    Rng evalRng(999);
    const EvalResult r = Evaluate(&env, agent, 100, &evalRng);
    printf("REINFORCE: success=%.2f meanSteps=%.1f meanReward=%.2f\n", r.successRate, r.meanSteps, r.meanReward);
    check(r.successRate >= 0.8, "REINFORCE policy reaches the goal (>= 80%)");
}

}  // namespace

int main() {
    testRng();
    testEnv();
    testQLearning();
    testReinforce();
    if (g_fail == 0) printf("test_crl: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
