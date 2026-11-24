// 마을 접속 서버 (창 없음, Qt 필요 없음): 웹으로 접속한 사람들의 캐릭터를 움직인다.
//
//   village_server [Data/Object 폴더] [시작 x] [시작 y]
//
// 표준 입력으로 한 줄씩 명령을 받고, "tick" 을 받을 때마다 모두의 위치를 JSON 한 줄로 표준 출력에 낸다.
// 시간은 tick 이 가져온다(서버는 자기 시계를 보지 않는다). 그래서 결과가 늘 같고 시험하기 쉽다.
// web/server.js(Node)가 이 프로그램을 띄워서 웹소켓과 이어 준다. 직접 써 볼 수도 있다:
//
//   join 1 한솔                       접속 (id 는 숫자, 이름에 공백은 없다)
//   leave 1                           퇴장
//   input 1 <앞뒤 -1..1> <좌우 -1..1> <달리기 0/1> <바라보는 방향 rad>
//   tick 0.05                         0.05 초 흐르게 하고 상태를 낸다
//
// 좌표는 엔진 좌표(x, y, 높이 z; Z-up, m). 몸의 방향 yaw 에서 앞쪽은 (sin yaw, -cos yaw), 오른쪽은 (-cos yaw, -sin yaw).
// 장애물(나무, 집 등)은 obstacles.txt 의 원, 땅 높이는 ground.txt (tools/make_ground.js)에서 읽는다.
// 장애물과 다른 접속자에게는 막히고, 물가(땅이 낮은 곳)로는 나가지 못한다.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_PLAYERS 64
#define MAX_OBSTACLES 1024

namespace {

const double kBody = 0.4;          // 접속자 몸 반지름 (장애물에 더한다)
const double kPlayerGap = 0.8;     // 접속자끼리 중심 거리의 최소값
const double kWalk = 2.5, kRun = 5.0;
const double kMinGround = 0.65;    // 이보다 낮은 땅(물가, 바다)에는 올라서지 못한다
const double kPi = 3.14159265358979323846;

struct Player {
    bool used;
    int id;
    char name[40];
    double x, y, yaw;
    int fwd, side, run;
    double inYaw;
};

struct Circle {
    double x, y, r;
};

Player g_players[MAX_PLAYERS];
Circle g_obs[MAX_OBSTACLES];
int g_obsCount = 0;
int g_gridW = 0, g_gridH = 0;
double g_originX = 0, g_originY = 0, g_cell = 1;
double* g_height = NULL;
double g_time = 0;
double g_spawnX = 0, g_spawnY = 0;

bool LoadObstacles(const char* file) {
    FILE* f = fopen(file, "r");
    if (!f) return false;
    char line[256];
    while (fgets(line, sizeof(line), f) && g_obsCount < MAX_OBSTACLES) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        Circle c;
        if (sscanf(line, "%lf %lf %lf", &c.x, &c.y, &c.r) == 3) g_obs[g_obsCount++] = c;
    }
    fclose(f);
    return true;
}

bool LoadGround(const char* file) {
    FILE* f = fopen(file, "r");
    if (!f) return false;
    char* line = static_cast<char*>(malloc(1 << 16));
    int row = 0;
    bool ok = true;
    while (ok && fgets(line, 1 << 16, f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        if (strncmp(line, "size", 4) == 0) {
            if (sscanf(line + 4, "%d %d", &g_gridW, &g_gridH) != 2 || g_gridW < 2 || g_gridH < 2 || g_gridW > 4096 || g_gridH > 4096) ok = false;
            else g_height = static_cast<double*>(malloc(sizeof(double) * static_cast<size_t>(g_gridW) * static_cast<size_t>(g_gridH)));
        } else if (strncmp(line, "origin", 6) == 0) {
            ok = sscanf(line + 6, "%lf %lf", &g_originX, &g_originY) == 2;
        } else if (strncmp(line, "cell", 4) == 0) {
            ok = sscanf(line + 4, "%lf", &g_cell) == 1 && g_cell > 0;
        } else if (g_height && row < g_gridH) {
            const char* p = line;
            for (int c = 0; c < g_gridW && ok; c++) {
                char* end = NULL;
                const double v = strtod(p, &end);
                if (end == p) ok = false;
                g_height[row * g_gridW + c] = v;
                p = end;
            }
            row++;
        }
    }
    free(line);
    fclose(f);
    return ok && g_height && row == g_gridH;
}

// 땅 높이 (이중선형). 표 밖은 아주 낮은 값(바다)
double GroundAt(double x, double y) {
    if (!g_height) return 0.0;
    const double gx = (x - g_originX) / g_cell, gy = (y - g_originY) / g_cell;
    if (gx < 0 || gy < 0 || gx > g_gridW - 1 || gy > g_gridH - 1) return -99.0;
    int ix = static_cast<int>(gx), iy = static_cast<int>(gy);
    if (ix > g_gridW - 2) ix = g_gridW - 2;
    if (iy > g_gridH - 2) iy = g_gridH - 2;
    const double fx = gx - ix, fy = gy - iy;
    const double* h = g_height;
    const double a = h[iy * g_gridW + ix], b = h[iy * g_gridW + ix + 1];
    const double c = h[(iy + 1) * g_gridW + ix], d = h[(iy + 1) * g_gridW + ix + 1];
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
}

void PushOut(double* x, double* y, double cx, double cy, double r) {
    const double dx = *x - cx, dy = *y - cy, d2 = dx * dx + dy * dy;
    if (d2 >= r * r) return;
    const double d = sqrt(d2);
    if (d < 1e-9) { *x = cx + r; return; }
    *x = cx + dx / d * r;
    *y = cy + dy / d * r;
}

// (x, y) 에 서 있어도 되는가: 걸을 수 있는 땅이고, 장애물 안이 아니고, 다른 사람과 겹치지 않는다
bool Free(double x, double y, int self, double margin) {
    if (GroundAt(x, y) < kMinGround) return false;
    for (int i = 0; i < g_obsCount; i++) {
        const double dx = x - g_obs[i].x, dy = y - g_obs[i].y, r = g_obs[i].r + kBody + margin;
        if (dx * dx + dy * dy < r * r) return false;
    }
    for (int i = 0; i < MAX_PLAYERS; i++) {
        if (!g_players[i].used || i == self) continue;
        const double dx = x - g_players[i].x, dy = y - g_players[i].y, r = kPlayerGap + margin;
        if (dx * dx + dy * dy < r * r) return false;
    }
    return true;
}

// 시작 위치 근처에서 비어 있는 곳을 나선으로 찾는다
bool FindSpawn(double* ox, double* oy) {
    for (int ring = 0; ring < 60; ring++) {
        const double r = ring * 1.2;
        const int n = ring == 0 ? 1 : 6 + ring * 4;
        for (int k = 0; k < n; k++) {
            const double a = 2.0 * kPi * k / n;
            const double x = g_spawnX + r * cos(a), y = g_spawnY + r * sin(a);
            if (GroundAt(x, y) >= 0.9 && Free(x, y, -1, 0.3)) { *ox = x; *oy = y; return true; }
        }
    }
    return false;
}

int FindPlayer(int id) {
    for (int i = 0; i < MAX_PLAYERS; i++)
        if (g_players[i].used && g_players[i].id == id) return i;
    return -1;
}

// 이름에서 JSON 을 깨뜨릴 글자를 바꾼다 (한글 등 UTF-8 바이트는 그대로 둔다)
void CleanName(const char* in, char* out, size_t cap) {
    size_t n = 0;
    for (; in[n] && n + 1 < cap; n++) {
        const unsigned char c = static_cast<unsigned char>(in[n]);
        out[n] = (c < 0x20 || c == '"' || c == '\\' || c == 0x7f) ? '_' : static_cast<char>(c);
    }
    out[n] = 0;
    if (n == 0) strcpy(out, "guest");
}

void Join(int id, const char* name) {
    if (FindPlayer(id) >= 0) return;
    int slot = -1;
    for (int i = 0; i < MAX_PLAYERS; i++)
        if (!g_players[i].used) { slot = i; break; }
    double x = g_spawnX, y = g_spawnY;
    if (slot < 0 || !FindSpawn(&x, &y)) return;
    Player& p = g_players[slot];
    memset(&p, 0, sizeof(p));
    p.used = true;
    p.id = id;
    CleanName(name, p.name, sizeof(p.name));
    p.x = x;
    p.y = y;
}

void Step(double dt) {
    for (int i = 0; i < MAX_PLAYERS; i++) {
        Player& p = g_players[i];
        if (!p.used) continue;
        p.yaw = p.inYaw;
        if (p.fwd == 0 && p.side == 0) continue;
        const double fx = sin(p.yaw), fy = -cos(p.yaw), rx = -cos(p.yaw), ry = -sin(p.yaw);
        double vx = fx * p.fwd + rx * p.side, vy = fy * p.fwd + ry * p.side;
        const double n = sqrt(vx * vx + vy * vy);
        const double step = (p.run ? kRun : kWalk) * dt;
        vx = vx / n * step;
        vy = vy / n * step;
        double x = p.x + vx, y = p.y + vy;
        for (int it = 0; it < 2; it++) {   // 겹친 것이 둘이어도 밀려 나오도록 두 번
            for (int k = 0; k < g_obsCount; k++) PushOut(&x, &y, g_obs[k].x, g_obs[k].y, g_obs[k].r + kBody);
            for (int k = 0; k < MAX_PLAYERS; k++)
                if (g_players[k].used && k != i) PushOut(&x, &y, g_players[k].x, g_players[k].y, kPlayerGap);
        }
        if (GroundAt(x, y) >= kMinGround) { p.x = x; p.y = y; continue; }
        // 물가: 한 축씩만 가 본다 (미끄러지듯 따라 걷는다)
        if (GroundAt(p.x + vx, p.y) >= kMinGround && Free(p.x + vx, p.y, i, 0)) p.x += vx;
        else if (GroundAt(p.x, p.y + vy) >= kMinGround && Free(p.x, p.y + vy, i, 0)) p.y += vy;
    }
}

void PrintState() {
    static char buf[1 << 15];
    size_t n = static_cast<size_t>(snprintf(buf, sizeof(buf), "{\"t\":%.3f,\"p\":[", g_time));
    bool first = true;
    for (int i = 0; i < MAX_PLAYERS && n + 256 < sizeof(buf); i++) {
        const Player& p = g_players[i];
        if (!p.used) continue;
        n += static_cast<size_t>(snprintf(buf + n, sizeof(buf) - n, "%s{\"id\":%d,\"n\":\"%s\",\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"yaw\":%.4f}",
                                          first ? "" : ",", p.id, p.name, p.x, p.y, GroundAt(p.x, p.y), p.yaw));
        first = false;
    }
    snprintf(buf + n, sizeof(buf) - n, "]}");
    puts(buf);
    fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
    const char* dir = argc > 1 ? argv[1] : "Data/Object";
    if (argc > 2) g_spawnX = atof(argv[2]);
    if (argc > 3) g_spawnY = atof(argv[3]);
    char path[1024];
    snprintf(path, sizeof(path), "%s/villagers/obstacles.txt", dir);
    if (!LoadObstacles(path)) { fprintf(stderr, "장애물 파일을 읽을 수 없습니다: %s\n", path); return 1; }
    snprintf(path, sizeof(path), "%s/ground.txt", dir);
    if (!LoadGround(path)) { fprintf(stderr, "땅 높이 파일을 읽을 수 없습니다: %s (node tools/make_ground.js)\n", path); return 1; }
    printf("{\"ready\":true,\"obstacles\":%d,\"ground\":%d}\n", g_obsCount, g_gridW);
    fflush(stdout);

    char line[512];
    while (fgets(line, sizeof(line), stdin)) {
        char cmd[16] = "";
        int id = 0;
        if (sscanf(line, "%15s", cmd) != 1) continue;
        if (strcmp(cmd, "tick") == 0) {
            double dt = 0;
            if (sscanf(line, "%*s %lf", &dt) != 1 || dt <= 0) continue;
            if (dt > 0.25) dt = 0.25;
            g_time += dt;
            Step(dt);
            PrintState();
        } else if (strcmp(cmd, "join") == 0) {
            char name[64] = "";
            if (sscanf(line, "%*s %d %63s", &id, name) >= 1) Join(id, name);
        } else if (strcmp(cmd, "leave") == 0) {
            if (sscanf(line, "%*s %d", &id) == 1) {
                const int i = FindPlayer(id);
                if (i >= 0) g_players[i].used = false;
            }
        } else if (strcmp(cmd, "input") == 0) {
            int fwd = 0, side = 0, run = 0;
            double yaw = 0;
            if (sscanf(line, "%*s %d %d %d %d %lf", &id, &fwd, &side, &run, &yaw) == 5) {
                const int i = FindPlayer(id);
                if (i < 0) continue;
                g_players[i].fwd = fwd > 0 ? 1 : (fwd < 0 ? -1 : 0);
                g_players[i].side = side > 0 ? 1 : (side < 0 ? -1 : 0);
                g_players[i].run = run ? 1 : 0;
                if (yaw == yaw && fabs(yaw) < 1e6) g_players[i].inYaw = yaw;   // NaN/거대값은 버린다
            }
        }
    }
    free(g_height);
    return 0;
}
