// 접속자(웹으로 들어온 사람)의 캐릭터: 이동, 장애물/접속자 충돌, 땅 높이, 엔진 연결.
// (Qt 와 Python 에 의존하지 않는다)
//
//   PlayerWorld  접속자들의 위치만 계산한다 (엔진 없음). 입력(앞뒤/좌우/달리기/바라보는 방향)대로 걷고,
//                나무·집 같은 원 장애물, 다른 접속자, 물가(땅이 낮은 곳)에 막힌다. 땅 높이는 ground.txt 표(이중선형).
//   Players      PlayerWorld 를 엔진에 붙이는 Listener. 접속하면 Spawner 가 장면에 주민 모델을 올리고(AddModel),
//                매 틱마다 접속자 위치를 엔진의 오브젝트 자세로 쓴다(SetPose). 그래서 데스크톱 3D 화면에도 같이 보인다.
//                NPC(순찰하는 주민)는 SetNpcNames 로 알려 주면 상태 JSON 에 같이 실리고, 접속자가 그들에게도 막힌다.
//
//   좌표는 엔진 좌표(x, y, 높이 z; Z-up, m). 몸의 방향 yaw 에서 앞쪽은 (sin yaw, -cos yaw), 오른쪽은 (-cos yaw, -sin yaw).
#ifndef CORE_CSIMUL_PLAYERS_H
#define CORE_CSIMUL_PLAYERS_H

#include "CBase/Array.h"
#include "CBase/String.h"
#include "CSimul/AStar.h"
#include "CSimul/Chat.h"
#include "CSimul/Engine.h"

namespace simul {
namespace sim {

struct Player {
    int id;
    String name;            // 화면에 보일 이름
    String model;           // 장면의 모델 이름 (Players 가 채운다)
    double x, y, z, yaw;    // 위치(발)와 몸 방향
    int fwd, side, run;     // 입력: 앞뒤 -1..1, 좌우 -1..1, 달리기 0/1
    double inYaw;           // 입력으로 받은 바라보는 방향
    String say;             // 말풍선에 보일 글 (비었거나 sayLeft 가 0 이하면 없음)
    double sayLeft;         // 말풍선이 앞으로 보일 시간 (s)
    int sayId;              // 말할 때마다 하나씩 늘어난다 (받는 쪽이 새 말인지 알아보게)
    Player() : id(0), x(0), y(0), z(0), yaw(0), fwd(0), side(0), run(0), inYaw(0), sayLeft(0), sayId(0) {}
};

class PlayerWorld {
public:
    PlayerWorld();

    bool LoadGround(const String& text, String* error);            // tools/make_ground.js 가 만든 ground.txt
    bool ParseObstacles(const String& text, String* error);        // obstacles.txt: "x y 반지름" 줄들
    void SetSpawn(double x, double y) { spawnX_ = x; spawnY_ = y; }
    void SetMaxPlayers(int n) { max_ = n < 1 ? 1 : n; }
    void SetExternals(const Array<Circle2>& c) { external_ = c; }  // 움직이는 다른 몸(NPC): 지금 위치의 원
    void SetSpeeds(double walk, double run) { walk_ = walk; run_ = run; }

    bool Join(int id, const String& name);                         // 시작 위치 근처의 빈 곳에 세운다. 가득 찼거나 같은 id 면 false
    bool Leave(int id);
    bool SetInput(int id, int fwd, int side, int run, double yaw);
    bool Say(int id, const String& text);                          // 말풍선: 글자 수 60 까지, 길이에 따라 3~9 초
    void Step(double dt);

    int Count() const { return static_cast<int>(players_.size()); }
    const Player& At(int i) const { return players_[static_cast<size_t>(i)]; }
    Player* Find(int id);
    double GroundAt(double x, double y) const;                     // 표 밖은 아주 낮은 값(바다). 표를 안 읽었으면 어디나 평평한 땅(1 m)
    int ObstacleCount() const { return static_cast<int>(obs_.size()); }
    bool HasGround() const { return gw_ > 0; }

    static String CleanName(const String& in);                     // JSON 을 깨뜨릴 글자를 바꾼다 (한글 등 UTF-8 은 그대로)
    static String CleanText(const String& in, int maxChars);      // 채팅 글: 조절 문자 제거, 따옴표/역슬래시 치환, 글자 수 제한 (UTF-8 글자 단위)

private:
    bool isFree(double x, double y, int self, double margin) const;
    bool findSpawn(double* x, double* y) const;

    Array<Player> players_;
    Array<Circle2> obs_;
    Array<Circle2> external_;
    int gw_, gh_;
    double ox_, oy_, cell_;
    Array<double> height_;
    double spawnX_, spawnY_;
    int max_;
    double walk_, run_;
};

// 장면에 모델을 올리고 내리는 쪽(데스크톱 앱)이 구현한다
class PlayerSpawner {
public:
    virtual ~PlayerSpawner() {}
    virtual bool AddModel(int id, String* modelName) = 0;          // id 에 맞는 주민 모델을 올리고 장면 이름을 돌려준다
    virtual void RemoveModel(const String& modelName) = 0;
};

class Players : public Listener {
public:
    Players();
    PlayerWorld& World() { return world_; }
    void SetSpawner(PlayerSpawner* s) { spawner_ = s; }
    void SetNext(Listener* l) { next_ = l; }                       // 순찰 등 다른 Listener 도 같이 부른다
    void SetNpcNames(const Array<String>& names) { npcs_ = names; }

    bool Join(int id, const String& name);
    void Leave(int id);
    // 한 줄 명령: "join <id> <이름>", "leave <id>", "input <id> <앞뒤> <좌우> <달리기> <방향>", "chat <id> <글>". 처리했으면 true
    bool Command(const String& line);
    // 지금 모두의 상태: {"t":..,"p":[{"id":..,"n":"..","x":..,"y":..,"z":..,"yaw":..},...]} (NPC 의 id 는 -1, -2, ...)
    String StateJson(Engine& e, double time);
    // 지금 보이는 말풍선 (모델 이름 + 글 + 사라질 시뮬레이션 시각). 데스크톱의 SpeechBubbles 에 그대로 넘긴다
    Array<Bubble> Bubbles() const;

    void OnStart(Engine& e) override;
    void OnTick(Engine& e, double time, double dt) override;
    void OnEvent(Engine& e, const Event& ev) override;
    void OnStop(Engine& e) override;

private:
    PlayerWorld world_;
    PlayerSpawner* spawner_;
    Listener* next_;
    Array<String> npcs_;
    double time_;
};

}  // namespace sim
}  // namespace simul

#endif  // CORE_CSIMUL_PLAYERS_H
