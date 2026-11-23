// 강화학습 학습용 예제: 로봇 팔 끝을 목표 지점으로 보내는 정책을 배운다.
//
//   ./build/bin/rl_arm_reach                       # Q-러닝 (기본)
//   ./build/bin/rl_arm_reach --algo reinforce      # 정책 기울기
//   ./build/bin/rl_arm_reach --episodes 3000 --seed 7
//   ./build/bin/rl_arm_reach --urdf Data/Object/samples/arm.urdf
//
// 출력: 학습 중 성공률 변화(학습 곡선) -> 학습 후 평가 -> 상태별 정책 지도
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CRL/ArmReachEnv.h"
#include "CRL/QLearning.h"
#include "CRL/Reinforce.h"

using namespace simul;
using namespace simul::rl;

#ifndef SIMUL_DATA_DIR
#define SIMUL_DATA_DIR "Data"
#endif

namespace {

void PrintCurve(const Array<EpisodeLog>& log, int window) {
    printf("\n[학습 곡선] %d 에피소드마다: 성공률 / 평균 스텝 / 평균 총보상\n", window);
    for (size_t start = 0; start + static_cast<size_t>(window) <= log.size(); start += static_cast<size_t>(window)) {
        int ok = 0;
        double steps = 0, reward = 0;
        for (size_t i = start; i < start + static_cast<size_t>(window); i++) {
            ok += log[i].success ? 1 : 0;
            steps += log[i].steps;
            reward += log[i].totalReward;
        }
        printf("  %5zu~%5zu : %5.1f%%  %5.1f  %7.2f\n", start + 1, start + static_cast<size_t>(window), 100.0 * ok / window, steps / window, reward / window);
    }
}

// 행: 관절 A 칸, 열: 관절 B 칸. 각 칸에서 정책이 고른 행동을 문자로 보여 준다.
//   '^' = A 칸 +1,  'v' = A 칸 -1,  '>' = B 칸 +1,  '<' = B 칸 -1,  'G' = 목표 칸
void PrintPolicyMap(const ArmReachEnv& env, const Policy& policy) {
    const char symbols[4] = {'^', 'v', '>', '<'};
    printf("\n[정책 지도] 행 = 관절 %s 칸(위가 큰 값), 열 = 관절 %s 칸\n", env.Config().jointA.c_str(), env.Config().jointB.c_str());
    for (int a = env.Config().binsA - 1; a >= 0; a--) {
        printf("  A%02d ", a);
        for (int b = 0; b < env.Config().binsB; b++) {
            const int s = env.StateOf(a, b);
            printf("%c ", s == env.GoalState() ? 'G' : symbols[policy.Act(s)]);
        }
        printf("\n");
    }
}

}  // namespace

int main(int argc, char** argv) {
    String urdf = String(SIMUL_DATA_DIR) + "/Object/samples/arm.urdf";
    String algo = "q";
    int episodes = 1500;
    unsigned long long seed = 1;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--urdf") && i + 1 < argc) urdf = argv[++i];
        else if (!strcmp(argv[i], "--algo") && i + 1 < argc) algo = argv[++i];
        else if (!strcmp(argv[i], "--episodes") && i + 1 < argc) episodes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = strtoull(argv[++i], nullptr, 10);
        else {
            printf("사용법: rl_arm_reach [--algo q|reinforce] [--episodes N] [--seed S] [--urdf 파일]\n");
            return 1;
        }
    }
    if (episodes < 1) episodes = 1;

    ArmReachEnv env;
    String err;
    if (!env.Init(urdf, ArmReachConfig(), &err)) {
        printf("환경을 만들 수 없습니다: %s\n", err.c_str());
        return 1;
    }
    printf("환경: %s  상태 %d개, 행동 %d개, 목표 위치 (%.3f, %.3f, %.3f) m\n", urdf.c_str(), env.NumStates(), env.NumActions(),
           static_cast<double>(env.GoalPosition().x), static_cast<double>(env.GoalPosition().y), static_cast<double>(env.GoalPosition().z));

    Rng rng(seed);
    Array<EpisodeLog> log;
    EvalResult before, after;
    Rng evalRng(seed + 1000);
    if (algo == "reinforce") {
        Reinforce agent(env.NumStates(), env.NumActions());
        before = Evaluate(&env, agent, 100, &evalRng);   // 학습 전: 확률이 모두 같아 항상 같은 행동만 고른다
        TrainReinforce(&env, &agent, episodes, &rng, &log);
        Rng evalRng2(seed + 1000);
        after = Evaluate(&env, agent, 100, &evalRng2);
        PrintCurve(log, episodes >= 10 ? episodes / 10 : 1);
        PrintPolicyMap(env, agent);
    } else {
        QLearning agent(env.NumStates(), env.NumActions());
        before = Evaluate(&env, agent, 100, &evalRng);
        TrainQLearning(&env, &agent, episodes, &rng, &log);
        Rng evalRng2(seed + 1000);
        after = Evaluate(&env, agent, 100, &evalRng2);
        PrintCurve(log, episodes >= 10 ? episodes / 10 : 1);
        PrintPolicyMap(env, agent);
    }
    printf("\n[평가 100 에피소드] 학습 전 성공률 %.0f%%  ->  학습 후 성공률 %.0f%% (평균 %.1f 스텝)\n", 100.0 * before.successRate,
           100.0 * after.successRate, after.meanSteps);
    return 0;
}
