#include "CSimul/Players.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "3d/dhPose.h"
#include "3d/dhQuat.h"
#include "3d/dhVector.h"

namespace simul {
namespace sim {

namespace {

const double kBody = 0.4;          // 접속자 몸 반지름 (장애물에 더한다)
const double kPlayerGap = 0.8;     // 접속자끼리, 접속자와 NPC 의 중심 거리 최소값
const double kMinGround = 0.65;    // 이보다 낮은 땅(물가, 바다)에는 올라서지 못한다
const double kPi = 3.14159265358979323846;
const double kWallGap = 0.45;      // 방 안에서 벽에서 떨어져 서는 거리
const double kDoorReach = 2.4;     // 문 앞에서 이 거리 안이면 B 가 먹는다

void PushOut(double* x, double* y, double cx, double cy, double r) {
    const double dx = *x - cx, dy = *y - cy, d2 = dx * dx + dy * dy;
    if (d2 >= r * r) return;
    const double d = sqrt(d2);
    if (d < 1e-9) { *x = cx + r; return; }
    *x = cx + dx / d * r;
    *y = cy + dy / d * r;
}

// 텍스트를 줄 단위로 읽는다 (\n, \r\n)
bool NextLine(const String& s, size_t* pos, String* line) {
    if (*pos >= s.size()) return false;
    size_t e = *pos;
    while (e < s.size() && s[e] != '\n') e++;
    size_t end = e;
    if (end > *pos && s[end - 1] == '\r') end--;
    *line = s.substr(*pos, end - *pos);
    *pos = e < s.size() ? e + 1 : e;
    return true;
}

}  // namespace

PlayerWorld::PlayerWorld() : gw_(0), gh_(0), ox_(0), oy_(0), cell_(1), spawnX_(0), spawnY_(0), max_(64), walk_(2.5), run_(5.0) {}

bool PlayerWorld::LoadGround(const String& text, String* error) {
    gw_ = gh_ = 0;
    height_.clear();
    size_t pos = 0;
    String line;
    int w = 0, h = 0, row = 0;
    bool haveSize = false;
    while (NextLine(text, &pos, &line)) {
        if (line.empty() || line[0] == '#') continue;
        if (strncmp(line.c_str(), "size", 4) == 0 && line.size() > 4 && (line[4] == ' ' || line[4] == '\t')) {
            if (sscanf(line.c_str() + 4, "%d %d", &w, &h) != 2 || w < 2 || h < 2 || w > 4096 || h > 4096) {
                if (error) *error = "ground: size 줄이 이상합니다";
                return false;
            }
            height_.resize(static_cast<size_t>(w) * static_cast<size_t>(h), 0.0);
            haveSize = true;
        } else if (strncmp(line.c_str(), "origin", 6) == 0) {
            if (sscanf(line.c_str() + 6, "%lf %lf", &ox_, &oy_) != 2) { if (error) *error = "ground: origin 줄이 이상합니다"; return false; }
        } else if (strncmp(line.c_str(), "cell", 4) == 0) {
            if (sscanf(line.c_str() + 4, "%lf", &cell_) != 1 || cell_ <= 0) { if (error) *error = "ground: cell 줄이 이상합니다"; return false; }
        } else if (haveSize && row < h) {
            const char* p = line.c_str();
            for (int c = 0; c < w; c++) {
                char* end = NULL;
                const double v = strtod(p, &end);
                if (end == p) { if (error) *error = "ground: 높이 값이 모자랍니다"; return false; }
                height_[static_cast<size_t>(row) * static_cast<size_t>(w) + static_cast<size_t>(c)] = v;
                p = end;
            }
            row++;
        }
    }
    if (!haveSize || row != h) {
        if (error) *error = "ground: 높이 표의 줄 수가 맞지 않습니다";
        return false;
    }
    gw_ = w;
    gh_ = h;
    return true;
}

bool PlayerWorld::ParseObstacles(const String& text, String* error) {
    (void)error;
    obs_.clear();
    size_t pos = 0;
    String line;
    while (NextLine(text, &pos, &line)) {
        if (line.empty() || line[0] == '#') continue;
        double x, y, r;
        if (sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &r) == 3) obs_.push_back(Circle2(x, y, r));
    }
    return true;
}

int PlayerWorld::RoomAt(double x, double y) const {
    for (size_t i = 0; i < rooms_.size(); i++) {
        const Room& r = rooms_[i];
        if (fabs(x - r.cx) <= r.hw && fabs(y - r.cy) <= r.hd) return static_cast<int>(i);
    }
    return -1;
}

bool PlayerWorld::ParsePortals(const String& text, String* error) {
    rooms_.clear();
    size_t pos = 0;
    String line;
    Array<Circle2> blocks;
    while (NextLine(text, &pos, &line)) {
        if (line.empty() || line[0] == '#') continue;
        char kind[8] = "", name[40] = "", type[24] = "";
        if (strncmp(line.c_str(), "room ", 5) == 0) {
            Room r;
            double wall = 0;
            char w[16] = "", f[16] = "";
            if (sscanf(line.c_str(), "%7s %39s %23s %lf %lf %lf %lf %lf %lf %lf %lf %lf %15s %15s", kind, name, type, &r.outX, &r.outY, &r.outYaw,
                       &r.cx, &r.cy, &r.hw, &r.hd, &r.ceil, &r.floorZ, w, f) < 12) {
                if (error) *error = "portals: room 줄이 이상합니다: " + line;
                return false;
            }
            (void)wall;
            r.name = name;
            r.kind = type;
            rooms_.push_back(r);
        } else if (strncmp(line.c_str(), "furn ", 5) == 0) {
            double x, y, w, d, h;
            char col[16] = "";
            int solid = 0, glow = 0;
            if (sscanf(line.c_str(), "%7s %39s %lf %lf %lf %lf %lf %15s %d %d", kind, name, &x, &y, &w, &d, &h, col, &solid, &glow) < 9) {
                if (error) *error = "portals: furn 줄이 이상합니다: " + line;
                return false;
            }
            if (!solid) continue;
            // 긴 가구는 짧은 변을 지름으로 하는 원 여러 개로 막는다
            const bool alongX = w >= d;
            const double shortS = alongX ? d : w, longS = alongX ? w : d, r = shortS / 2;
            const int n = longS - shortS < 1e-6 ? 1 : static_cast<int>(ceil((longS - shortS) / (shortS * 0.8))) + 1;
            for (int k = 0; k < n; k++) {
                const double t = n == 1 ? 0.0 : -(longS - shortS) / 2 + (longS - shortS) * k / (n - 1);
                blocks.push_back(alongX ? Circle2(x + t, y, r) : Circle2(x, y + t, r));
            }
        }
    }
    for (size_t i = 0; i < blocks.size(); i++) obs_.push_back(blocks[i]);
    return true;
}

bool PlayerWorld::teleport(Player* p, double x, double y) const {
    for (int ring = 0; ring < 8; ring++) {
        const double r = ring * 0.5;
        const int n = ring == 0 ? 1 : 6 + ring * 3;
        for (int k = 0; k < n; k++) {
            const double a = 2.0 * kPi * k / n;
            const double tx = x + r * cos(a), ty = y + r * sin(a);
            if (GroundAt(tx, ty) >= kMinGround && isFree(tx, ty, p->id, 0.2)) {
                p->x = tx;
                p->y = ty;
                p->z = GroundAt(tx, ty);
                return true;
            }
        }
    }
    return false;
}

bool PlayerWorld::Enter(int id) {
    Player* p = Find(id);
    if (!p) return false;
    const int in = RoomAt(p->x, p->y);
    if (in >= 0) {   // 방 안: 남쪽 벽의 문 앞이면 나간다
        const Room& r = rooms_[static_cast<size_t>(in)];
        const double dx = p->x - r.cx, dy = p->y - (r.cy - r.hd);
        if (dx * dx + dy * dy > kDoorReach * kDoorReach) return false;
        return teleport(p, r.outX, r.outY);
    }
    for (size_t i = 0; i < rooms_.size(); i++) {   // 밖: 어느 건물 문 앞이면 들어간다
        const Room& r = rooms_[i];
        const double dx = p->x - r.outX, dy = p->y - r.outY;
        if (dx * dx + dy * dy <= kDoorReach * kDoorReach) return teleport(p, r.cx, r.cy - r.hd + 1.5);
    }
    return false;
}

double PlayerWorld::GroundAt(double x, double y) const {
    for (size_t i = 0; i < rooms_.size(); i++) {   // 방 바닥 (벽에서 조금 떨어진 안쪽만 걸을 수 있다)
        const Room& r = rooms_[i];
        if (fabs(x - r.cx) <= r.hw - kWallGap && fabs(y - r.cy) <= r.hd - kWallGap) return r.floorZ;
    }
    if (gw_ <= 0) return 1.0;   // 표가 없으면 어디나 걸을 수 있는 평평한 땅(높이 1 m)
    const double gx = (x - ox_) / cell_, gy = (y - oy_) / cell_;
    if (gx < 0 || gy < 0 || gx > gw_ - 1 || gy > gh_ - 1) return -99.0;
    int ix = static_cast<int>(gx), iy = static_cast<int>(gy);
    if (ix > gw_ - 2) ix = gw_ - 2;
    if (iy > gh_ - 2) iy = gh_ - 2;
    const double fx = gx - ix, fy = gy - iy;
    const size_t w = static_cast<size_t>(gw_);
    const size_t i0 = static_cast<size_t>(iy) * w + static_cast<size_t>(ix);
    const double a = height_[i0], b = height_[i0 + 1], c = height_[i0 + w], d = height_[i0 + w + 1];
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
}

bool PlayerWorld::isFree(double x, double y, int self, double margin) const {
    if (GroundAt(x, y) < kMinGround) return false;
    for (size_t i = 0; i < obs_.size(); i++) {
        const double dx = x - obs_[i].x, dy = y - obs_[i].y, r = obs_[i].r + kBody + margin;
        if (dx * dx + dy * dy < r * r) return false;
    }
    for (size_t i = 0; i < players_.size(); i++) {
        if (players_[i].id == self) continue;
        const double dx = x - players_[i].x, dy = y - players_[i].y, r = kPlayerGap + margin;
        if (dx * dx + dy * dy < r * r) return false;
    }
    for (size_t i = 0; i < external_.size(); i++) {
        const double dx = x - external_[i].x, dy = y - external_[i].y, r = external_[i].r + margin;
        if (dx * dx + dy * dy < r * r) return false;
    }
    return true;
}

bool PlayerWorld::findSpawn(double* ox, double* oy) const {
    for (int ring = 0; ring < 60; ring++) {
        const double r = ring * 1.2;
        const int n = ring == 0 ? 1 : 6 + ring * 4;
        for (int k = 0; k < n; k++) {
            const double a = 2.0 * kPi * k / n;
            const double x = spawnX_ + r * cos(a), y = spawnY_ + r * sin(a);
            if (GroundAt(x, y) >= 0.9 && isFree(x, y, -1, 0.3)) { *ox = x; *oy = y; return true; }
        }
    }
    return false;
}

Player* PlayerWorld::Find(int id) {
    for (size_t i = 0; i < players_.size(); i++)
        if (players_[i].id == id) return &players_[i];
    return NULL;
}

String PlayerWorld::CleanName(const String& in) {
    String out;
    for (size_t i = 0; i < in.size() && out.size() < 39; i++) {
        const unsigned char c = static_cast<unsigned char>(in[i]);
        out += (c < 0x20 || c == '"' || c == '\\' || c == 0x7f) ? '_' : static_cast<char>(c);
    }
    if (out.empty()) out = "guest";
    return out;
}

String PlayerWorld::CleanText(const String& in, int maxChars) {
    String out;
    int chars = 0;
    for (size_t i = 0; i < in.size(); i++) {
        const unsigned char c = static_cast<unsigned char>(in[i]);
        const bool cont = (c & 0xC0) == 0x80;   // UTF-8 이어지는 바이트
        if (!cont && ++chars > maxChars) break;
        if (c == '"') out += '\'';
        else if (c == '\\') out += '/';
        else if (c < 0x20 || c == 0x7f) { if (!cont) out += ' '; }
        else out += static_cast<char>(c);
    }
    size_t a = 0, b = out.size();
    while (a < b && out[a] == ' ') a++;
    while (b > a && out[b - 1] == ' ') b--;
    return out.substr(a, b - a);
}

bool PlayerWorld::Say(int id, const String& text) {
    Player* p = Find(id);
    if (!p) return false;
    const String t = CleanText(text, 60);
    if (t.empty()) return false;
    p->say = t;
    int chars = 0;
    for (size_t i = 0; i < t.size(); i++) if ((static_cast<unsigned char>(t[i]) & 0xC0) != 0x80) chars++;
    p->sayLeft = 3.0 + 0.12 * chars;
    if (p->sayLeft > 9.0) p->sayLeft = 9.0;
    p->sayId++;
    return true;
}

bool PlayerWorld::Join(int id, const String& name) {
    if (Find(id) || static_cast<int>(players_.size()) >= max_) return false;
    double x = spawnX_, y = spawnY_;
    if (!findSpawn(&x, &y)) return false;
    Player p;
    p.id = id;
    p.name = CleanName(name);
    p.x = x;
    p.y = y;
    p.z = GroundAt(x, y);
    players_.push_back(p);
    return true;
}

bool PlayerWorld::Leave(int id) {
    for (size_t i = 0; i < players_.size(); i++) {
        if (players_[i].id != id) continue;
        players_.erase(players_.begin() + static_cast<long>(i));
        return true;
    }
    return false;
}

bool PlayerWorld::SetInput(int id, int fwd, int side, int run, double yaw) {
    Player* p = Find(id);
    if (!p) return false;
    p->fwd = fwd > 0 ? 1 : (fwd < 0 ? -1 : 0);
    p->side = side > 0 ? 1 : (side < 0 ? -1 : 0);
    p->run = run ? 1 : 0;
    if (yaw == yaw && fabs(yaw) < 1e6) p->inYaw = yaw;   // NaN 과 거대한 값은 버린다
    return true;
}

void PlayerWorld::Step(double dt) {
    for (size_t i = 0; i < players_.size(); i++) {
        Player& p = players_[i];
        p.yaw = p.inYaw;
        if (p.sayLeft > 0) { p.sayLeft -= dt; if (p.sayLeft <= 0) { p.sayLeft = 0; p.say.clear(); } }
        if (p.fwd != 0 || p.side != 0) {
            const double fx = sin(p.yaw), fy = -cos(p.yaw), rx = -cos(p.yaw), ry = -sin(p.yaw);
            double vx = fx * p.fwd + rx * p.side, vy = fy * p.fwd + ry * p.side;
            const double n = sqrt(vx * vx + vy * vy);
            const double step = (p.run ? run_ : walk_) * dt;
            vx = vx / n * step;
            vy = vy / n * step;
            double x = p.x + vx, y = p.y + vy;
            for (int it = 0; it < 2; it++) {   // 겹친 것이 둘이어도 밀려 나오도록 두 번
                for (size_t k = 0; k < obs_.size(); k++) PushOut(&x, &y, obs_[k].x, obs_[k].y, obs_[k].r + kBody);
                for (size_t k = 0; k < players_.size(); k++)
                    if (k != i) PushOut(&x, &y, players_[k].x, players_[k].y, kPlayerGap);
                for (size_t k = 0; k < external_.size(); k++) PushOut(&x, &y, external_[k].x, external_[k].y, external_[k].r);
            }
            if (GroundAt(x, y) >= kMinGround) {
                p.x = x;
                p.y = y;
            } else if (GroundAt(p.x + vx, p.y) >= kMinGround && isFree(p.x + vx, p.y, p.id, 0)) {   // 물가: 한 축씩만 가 본다
                p.x += vx;
            } else if (GroundAt(p.x, p.y + vy) >= kMinGround && isFree(p.x, p.y + vy, p.id, 0)) {
                p.y += vy;
            }
        }
        p.z = GroundAt(p.x, p.y);
    }
}

// ---------------------------------------------------------------------------------------------

Players::Players() : spawner_(NULL), next_(NULL), time_(0) {}

bool Players::Join(int id, const String& name) {
    if (!world_.Join(id, name)) return false;
    String model;
    if (spawner_ && !spawner_->AddModel(id, &model)) {
        world_.Leave(id);
        return false;
    }
    Player* p = world_.Find(id);
    if (p) p->model = model;
    return true;
}

void Players::Leave(int id) {
    Player* p = world_.Find(id);
    if (!p) return;
    const String model = p->model;
    world_.Leave(id);
    if (spawner_ && !model.empty()) spawner_->RemoveModel(model);
}

bool Players::Command(const String& line) {
    char cmd[16] = "";
    int id = 0;
    if (sscanf(line.c_str(), "%15s", cmd) != 1) return false;
    if (strcmp(cmd, "join") == 0) {
        char name[64] = "";
        if (sscanf(line.c_str(), "%*s %d %63s", &id, name) >= 1) return Join(id, name);
    } else if (strcmp(cmd, "leave") == 0) {
        if (sscanf(line.c_str(), "%*s %d", &id) == 1) { Leave(id); return true; }
    } else if (strcmp(cmd, "chat") == 0) {
        int n = 0;
        if (sscanf(line.c_str(), "%*s %d %n", &id, &n) >= 1 && n > 0) return world_.Say(id, line.substr(static_cast<size_t>(n)));
    } else if (strcmp(cmd, "input") == 0) {
        int fwd = 0, side = 0, run = 0;
        double yaw = 0;
        if (sscanf(line.c_str(), "%*s %d %d %d %d %lf", &id, &fwd, &side, &run, &yaw) == 5) return world_.SetInput(id, fwd, side, run, yaw);
    } else if (strcmp(cmd, "enter") == 0) {
        if (sscanf(line.c_str(), "%*s %d", &id) == 1) return world_.Enter(id);
    }
    return false;
}

void Players::OnStart(Engine& e) {
    if (next_) next_->OnStart(e);
}

void Players::OnTick(Engine& e, double time, double dt) {
    time_ = time;
    Array<Circle2> ext;   // NPC 의 지금 위치: 접속자가 겹쳐 서지 않게 한다
    for (size_t i = 0; i < npcs_.size(); i++) {
        ObjectInfo info;
        const Handle h = e.Find(npcs_[i]);
        if (h.slot >= 0 && e.Info(h, &info)) ext.push_back(Circle2(static_cast<double>(info.pose.position.x), static_cast<double>(info.pose.position.y), 0.8));
    }
    world_.SetExternals(ext);
    world_.Step(dt);
    for (int i = 0; i < world_.Count(); i++) {
        const Player& p = world_.At(i);
        if (p.model.empty()) continue;
        const Handle h = e.Find(p.model);
        if (h.slot < 0) continue;   // 모델이 아직 엔진에 등록되기 전
        e.SetPose(h, dhPose(dhVector(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)),
                            dhQuat::FromAxisAngle(static_cast<float>(p.yaw), dhVector(0, 0, 1))));
    }
    if (next_) next_->OnTick(e, time, dt);
}

void Players::OnEvent(Engine& e, const Event& ev) {
    if (next_) next_->OnEvent(e, ev);
}

void Players::OnStop(Engine& e) {
    if (next_) next_->OnStop(e);
}

Array<Bubble> Players::Bubbles() const {
    Array<Bubble> out;
    for (int i = 0; i < world_.Count(); i++) {
        const Player& p = world_.At(i);
        if (p.sayLeft <= 0 || p.say.empty() || p.model.empty()) continue;
        Bubble b;
        b.id = p.model;
        b.text = p.say;
        b.until = time_ + p.sayLeft;
        out.push_back(b);
    }
    return out;
}

String Players::StateJson(Engine& e, double time) {
    String out;
    char buf[320];
    snprintf(buf, sizeof(buf), "{\"t\":%.3f,\"p\":[", time);
    out += buf;
    bool first = true;
    for (int i = 0; i < world_.Count(); i++) {
        const Player& p = world_.At(i);
        snprintf(buf, sizeof(buf), "%s{\"id\":%d,\"n\":\"%s\",\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"yaw\":%.4f,\"sid\":%d", first ? "" : ",", p.id, p.name.c_str(), p.x, p.y, p.z, p.yaw, p.sayId);
        out += buf;
        if (p.sayLeft > 0 && !p.say.empty()) { out += ",\"s\":\""; out += p.say; out += "\""; }
        out += "}";
        first = false;
    }
    for (size_t i = 0; i < npcs_.size(); i++) {   // NPC: id -1, -2, ... (웹은 -id 번째 주민 모델로 그린다)
        ObjectInfo info;
        const Handle h = e.Find(npcs_[i]);
        if (h.slot < 0 || !e.Info(h, &info)) continue;
        const dhVector f = info.pose.orientation.Rotate(dhVector(0, -1, 0));   // 모델 앞쪽
        const double yaw = atan2(static_cast<double>(f.x), -static_cast<double>(f.y));
        snprintf(buf, sizeof(buf), "%s{\"id\":%d,\"n\":\"%s\",\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"yaw\":%.4f}", first ? "" : ",", -static_cast<int>(i + 1),
                 PlayerWorld::CleanName(npcs_[i]).c_str(), static_cast<double>(info.pose.position.x), static_cast<double>(info.pose.position.y),
                 static_cast<double>(info.pose.position.z), yaw);
        out += buf;
        first = false;
    }
    out += "]}";
    return out;
}

}  // namespace sim
}  // namespace simul
