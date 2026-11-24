// 마을 접속 서버, 화면 없는 버전 (Qt 필요 없음): 웹으로 접속한 사람들의 캐릭터를 움직인다.
//
//   village_server [Data/Object 폴더] [시작 x] [시작 y]
//
// 데스크톱 앱(simul_main --serve)과 달리 엔진과 3D 화면이 없다. 같은 계산(CSimul/Players.h 의 PlayerWorld)만 돌려서,
// 엔진 없이 웹 접속만 시험하거나 서버 컴퓨터에 화면이 없을 때 쓴다. (엔진과 이어 쓰려면 simul_main --serve)
//
// 표준 입력으로 한 줄씩 명령을 받고, "tick" 을 받을 때마다 모두의 위치를 JSON 한 줄로 표준 출력에 낸다.
// 시간은 tick 이 가져온다(서버는 자기 시계를 보지 않는다). 그래서 결과가 늘 같고 시험하기 쉽다.
//
//   join 1 한솔                       접속 (id 는 숫자, 이름에 공백은 없다)
//   leave 1                           퇴장
//   input 1 <앞뒤 -1..1> <좌우 -1..1> <달리기 0/1> <바라보는 방향 rad>
//   chat 1 안녕하세요                  말풍선 (id 다음의 나머지 줄 전체가 글)
//   tick 0.05                         0.05 초 흐르게 하고 상태를 낸다

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CBase/File.h"
#include "CSimul/Players.h"

using namespace simul;
using namespace simul::sim;

namespace {

bool ReadText(const char* path, String* out) {
    Array<unsigned char> b;
    String err;
    if (!ReadAllBytes(path, &b, &err) || b.empty()) return false;
    *out = String(reinterpret_cast<const char*>(&b[0]), b.size());
    return true;
}

void PrintState(const PlayerWorld& w, double t) {
    printf("{\"t\":%.3f,\"p\":[", t);
    for (int i = 0; i < w.Count(); i++) {
        const Player& p = w.At(i);
        printf("%s{\"id\":%d,\"n\":\"%s\",\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"yaw\":%.4f,\"sid\":%d", i ? "," : "", p.id, p.name.c_str(), p.x, p.y, p.z, p.yaw, p.sayId);
        if (p.sayLeft > 0 && !p.say.empty()) printf(",\"s\":\"%s\"", p.say.c_str());
        printf("}");
    }
    printf("]}\n");
    fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
    const String dir = argc > 1 ? argv[1] : "Data/Object";
    PlayerWorld world;
    if (argc > 2) world.SetSpawn(atof(argv[2]), argc > 3 ? atof(argv[3]) : 0.0);
    String text, err;
    if (!ReadText((dir + "/villagers/obstacles.txt").c_str(), &text) || !world.ParseObstacles(text, &err)) {
        fprintf(stderr, "장애물 파일을 읽을 수 없습니다: %s/villagers/obstacles.txt\n", dir.c_str());
        return 1;
    }
    if (!ReadText((dir + "/ground.txt").c_str(), &text) || !world.LoadGround(text, &err)) {
        fprintf(stderr, "땅 높이 파일을 읽을 수 없습니다: %s/ground.txt %s (node tools/make_ground.js)\n", dir.c_str(), err.c_str());
        return 1;
    }
    printf("{\"ready\":true,\"obstacles\":%d}\n", world.ObstacleCount());
    fflush(stdout);

    double time = 0;
    char line[512];
    while (fgets(line, sizeof(line), stdin)) {
        char cmd[16] = "";
        int id = 0;
        if (sscanf(line, "%15s", cmd) != 1) continue;
        if (strcmp(cmd, "tick") == 0) {
            double dt = 0;
            if (sscanf(line, "%*s %lf", &dt) != 1 || dt <= 0) continue;
            if (dt > 0.25) dt = 0.25;
            time += dt;
            world.Step(dt);
            PrintState(world, time);
        } else if (strcmp(cmd, "join") == 0) {
            char name[64] = "";
            if (sscanf(line, "%*s %d %63s", &id, name) >= 1) world.Join(id, name);
        } else if (strcmp(cmd, "leave") == 0) {
            if (sscanf(line, "%*s %d", &id) == 1) world.Leave(id);
        } else if (strcmp(cmd, "chat") == 0) {
            int n = 0;
            if (sscanf(line, "%*s %d %n", &id, &n) >= 1 && n > 0) world.Say(id, line + n);
        } else if (strcmp(cmd, "input") == 0) {
            int fwd = 0, side = 0, run = 0;
            double yaw = 0;
            if (sscanf(line, "%*s %d %d %d %d %lf", &id, &fwd, &side, &run, &yaw) == 5) world.SetInput(id, fwd, side, run, yaw);
        }
    }
    return 0;
}
