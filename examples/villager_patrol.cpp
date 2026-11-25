// 주민 순찰 예제 (창 없음, Qt 필요 없음).
//
// 지도(factory_map.glb)와 주민(villager_N.glb)을 올리고, waypoints.txt 의 지점을 위에서 아래로 차례로
// simul_move_to 로 걷게 한다. 길 찾기는 없다: 정해 둔 지점으로 직선으로 가고, 도착 이벤트가 오면 다음 지점으로 보낸다.
// 마지막 지점에 닿으면 처음으로 돌아간다.
//
//   villager_patrol [Data/Object 폴더] [시뮬레이션 시간(초)]
//
// 지점 파일 형식은 Data/Object/villagers/waypoints.txt 맨 위 주석을 본다.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "simul_api.h"

#define MAX_VILLAGERS 8
#define MAX_POINTS 256

struct Villager {
    char name[64];
    double speed;
    int count;                    // 지점 수
    double pt[MAX_POINTS][3];
    int next;                     // 지금 향해 가는 지점 번호
    int reached;                  // 지금까지 도착한 횟수
};

static int ReadWaypoints(const char* file, Villager* v, int maxV) {
    FILE* f = fopen(file, "r");
    if (!f) return -1;
    int n = 0;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        if (line[0] == '[') {
            if (n >= maxV) break;
            Villager* c = &v[n++];
            memset(c, 0, sizeof(*c));
            c->speed = 1.0;
            sscanf(line, "[%63[^]]]", c->name);
            continue;
        }
        if (n == 0) continue;
        Villager* c = &v[n - 1];
        if (strncmp(line, "speed", 5) == 0) {
            sscanf(line + 5, "%lf", &c->speed);
        } else if (c->count < MAX_POINTS) {
            double* p = c->pt[c->count];
            if (sscanf(line, "%lf %lf %lf", &p[0], &p[1], &p[2]) == 3) c->count++;
        }
    }
    fclose(f);
    return n;
}

// 현재 지점(from)에서 다음 지점(to)으로: 가는 방향을 보게 돌리고 직선으로 걷는다. 모델의 앞쪽은 엔진에서 -Y.
static void Go(simul_sim* sim, const Villager* v, int from, int to) {
    const double* a = v->pt[from];
    const double* b = v->pt[to];
    const double yaw = atan2(b[0] - a[0], -(b[1] - a[1]));
    simul_set_pose(sim, v->name, a[0], a[1], a[2], 0.0, 0.0, sin(yaw / 2), cos(yaw / 2));
    simul_move_to(sim, v->name, b[0], b[1], b[2], v->speed);
}

int main(int argc, char** argv) {
    const char* dir = argc > 1 ? argv[1] : "Data/Object";
    const double seconds = argc > 2 ? atof(argv[2]) : 30.0;
    const double dt = 0.02;
    char path[512];

    static Villager villagers[MAX_VILLAGERS];
    snprintf(path, sizeof(path), "%.200s/villagers/waypoints.txt", dir);
    const int nv = ReadWaypoints(path, villagers, MAX_VILLAGERS);
    if (nv <= 0) { fprintf(stderr, "지점 파일을 읽지 못했습니다: %s\n", path); return 1; }

    simul_sim* sim = simul_create();
    snprintf(path, sizeof(path), "%.200s/factory_map.glb", dir);
    if (simul_add_model(sim, path, 0, 0, 0, 0, 0, 0) < 0) { fprintf(stderr, "지도: %s\n", simul_last_error(sim)); return 1; }

    for (int i = 0; i < nv; i++) {
        Villager* v = &villagers[i];
        if (v->count < 2) { fprintf(stderr, "%s: 지점이 2개 이상 필요합니다\n", v->name); return 1; }
        snprintf(path, sizeof(path), "%.200s/villagers/%.60s.glb", dir, v->name);
        const double* p0 = v->pt[0];
        if (simul_add_model(sim, path, p0[0], p0[1], p0[2], 0, 0, 0) < 0) { fprintf(stderr, "%s: %s\n", v->name, simul_last_error(sim)); return 1; }
        v->next = 1;
        Go(sim, v, 0, 1);
    }

    int nextPrint = 0;
    while (simul_time(sim) < seconds) {
        simul_step(sim, dt);
        const int ne = simul_take_events(sim);
        for (int e = 0; e < ne; e++) {
            if (simul_event_type(sim, e) == 4) fprintf(stderr, "명령 실패: %s\n", simul_event_message(sim, e));
            if (simul_event_type(sim, e) != 2) continue;   // 도착만 본다
            for (int i = 0; i < nv; i++) {
                Villager* v = &villagers[i];
                if (strcmp(v->name, simul_event_object(sim, e)) != 0) continue;
                const int here = v->next;
                v->next = (v->next + 1) % v->count;
                v->reached++;
                Go(sim, v, here, v->next);
            }
        }
        if (simul_time(sim) >= nextPrint) {   // 1초마다 위치를 찍는다
            nextPrint++;
            printf("t=%4.1f", simul_time(sim));
            for (int i = 0; i < nv; i++) {
                double pose[7] = {0};
                simul_object_pose(sim, simul_find_object(sim, villagers[i].name), pose);
                printf("  %s(%.1f, %.1f, %.1f)", villagers[i].name, pose[0], pose[1], pose[2]);
            }
            printf("\n");
        }
    }
    for (int i = 0; i < nv; i++) printf("%s: 지점 %d번 도착\n", villagers[i].name, villagers[i].reached);
    simul_destroy(sim);
    return 0;
}
