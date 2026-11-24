// CSimul 접속자 테스트: 걷기, 장애물/접속자/물가에 막힘, 입장과 퇴장, 채팅 글 걸러내기, 한 줄 명령
#include <math.h>
#include <stdio.h>

#include "CSimul/Players.h"
#include "TestUtil.h"

using namespace simul;
using namespace simul::sim;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}
bool near(double a, double b, double tol = 1e-3) { return fabs(a - b) <= tol; }

// 4x4 땅: 대부분 높고(2 m), 오른쪽 한 줄(x = 1 m)은 물가(0 m). 칸 1 m, 원점 (-2, -2)
const char* kGround =
    "size 4 4\n"
    "origin -2 -2\n"
    "cell 1\n"
    "2 2 2 0\n"
    "2 2 2 0\n"
    "2 2 2 0\n"
    "2 2 2 0\n";

void walk(PlayerWorld& w, double seconds) {
    for (double t = 0; t < seconds; t += 0.05) w.Step(0.05);
}

void testWalk() {
    PlayerWorld w;
    check(w.Join(1, "한솔"), "join");
    check(!w.Join(1, "again"), "같은 id 는 두 번 못 들어온다");
    check(w.Count() == 1, "한 명");
    check(near(w.At(0).x, 0) && near(w.At(0).y, 0), "시작 위치");
    w.SetInput(1, 1, 0, 0, 0.0);   // yaw 0: 앞쪽은 -y
    walk(w, 2.0);
    check(near(w.At(0).y, -5.0, 0.05), "2초에 5 m (2.5 m/s) 앞(-y)으로");
    check(near(w.At(0).x, 0, 1e-6), "옆으로는 안 간다");
    w.SetInput(1, 0, 1, 1, 0.0);   // 오른쪽(-x), 달리기
    walk(w, 1.0);
    check(near(w.At(0).x, -5.0, 0.05), "오른쪽은 -x 이고 달리면 5 m/s");
    check(w.Leave(1) && w.Count() == 0, "퇴장");
    check(!w.Leave(1), "없는 id 퇴장은 false");
    check(!w.SetInput(7, 1, 0, 0, 0), "없는 id 입력은 false");
}

void testBlocked() {
    PlayerWorld w;
    String err;
    w.ParseObstacles("# 나무\n0 -3 1\n", &err);
    w.Join(1, "a");
    w.SetInput(1, 1, 0, 0, 0.0);
    walk(w, 4.0);
    check(w.At(0).y > -3 + 1 + 0.4 - 0.05, "장애물(나무)을 뚫지 못한다");
    check(w.At(0).y < -3 + 1 + 0.4 + 1.0, "그래도 닿을 때까지는 간다");
    // 두 사람이 겹치지 않는다
    PlayerWorld v;
    v.Join(1, "a");
    v.Join(2, "b");
    double dx = v.At(0).x - v.At(1).x, dy = v.At(0).y - v.At(1).y;
    check(sqrt(dx * dx + dy * dy) >= 0.8, "시작할 때 겹치지 않는다");
    v.SetInput(1, 1, 0, 0, 0.0);
    v.SetInput(2, 1, 0, 0, 0.0);
    walk(v, 3.0);
    dx = v.At(0).x - v.At(1).x; dy = v.At(0).y - v.At(1).y;
    check(sqrt(dx * dx + dy * dy) >= 0.8 - 1e-6, "같이 걸어도 겹치지 않는다");
    // 외부 몸(NPC)
    PlayerWorld u;
    u.Join(1, "a");
    Array<Circle2> ext;
    ext.push_back(Circle2(0, -2, 0.8));
    u.SetExternals(ext);
    u.SetInput(1, 1, 0, 0, 0.0);
    walk(u, 3.0);
    check(u.At(0).y > -2 + 0.8 - 0.05, "NPC 에도 막힌다");
}

void testGround() {
    PlayerWorld w;
    String err;
    check(w.LoadGround(kGround, &err), "땅 높이 읽기");
    check(near(w.GroundAt(-1, -1), 2.0), "높이 2");
    check(near(w.GroundAt(1, 0), 0.0), "물가 0");
    check(w.GroundAt(-5, 0) < -50, "표 밖은 바다");
    w.SetSpawn(-1, -1);
    w.Join(1, "a");
    check(near(w.At(0).z, 2.0), "발 높이가 땅을 따른다");
        w.SetInput(1, 0, 1, 0, 3.14159265358979);   // yaw=pi 면 오른쪽이 +x (물가 쪽)
    walk(w, 5.0);
    check(w.GroundAt(w.At(0).x, w.At(0).y) >= 0.65, "물가(낮은 땅)로는 들어가지 않는다");
    check(w.At(0).x < 1.6, "물가 앞에서 멈춘다");
    PlayerWorld bad;
    check(!bad.LoadGround("size 2 2\norigin 0 0\ncell 1\n1 1\n", &err), "줄이 모자라면 실패");
}

void testChat() {
    check(PlayerWorld::CleanText("  안녕 \"하세요\" \\ 끝  ", 60) == "안녕 '하세요' / 끝", "따옴표/역슬래시 치환, 앞뒤 공백 제거");
    check(PlayerWorld::CleanText("a\nb\tc", 60) == "a b c", "조절 문자는 공백으로");
    const String cut = PlayerWorld::CleanText("가나다라마바사", 3);
    check(cut == "가나다", "글자 수(한글 3자)로 자른다");
    check(PlayerWorld::CleanName("a\"b") == "a_b", "이름의 따옴표");
    check(PlayerWorld::CleanName("") == "guest", "빈 이름");
    PlayerWorld w;
    w.Join(1, "a");
    check(!w.Say(1, "   "), "빈 말은 안 한다");
    check(!w.Say(9, "hi"), "없는 id");
    check(w.Say(1, "안녕"), "말하기");
    check(w.At(0).say == "안녕" && w.At(0).sayLeft > 3.0 && w.At(0).sayId == 1, "말풍선 시간과 번호");
    walk(w, 2.0);
    check(w.At(0).say == "안녕", "아직 보인다");
    walk(w, 2.0);
    check(w.At(0).say.empty(), "시간이 지나면 사라진다");
    w.Say(1, "또");
    check(w.At(0).sayId == 2, "말할 때마다 번호가 늘어난다");
}

void testCommands() {
    Players g;
    check(g.Command("join 5 마리"), "join 명령");
    check(g.World().Count() == 1 && g.World().At(0).name == "마리", "이름");
    check(g.Command("input 5 1 0 0 0.0"), "input 명령");
    check(g.Command("chat 5 안녕 하세요"), "chat 명령");
    check(g.World().At(0).say == "안녕 하세요", "chat 은 id 뒤의 나머지 전체");
    g.World().Step(0.05);
    check(g.World().At(0).fwd == 1, "입력이 들어갔다");
    check(!g.Command("input 5 1 0"), "모자란 인자는 거절");
    check(!g.Command("nonsense"), "모르는 명령");
    check(!g.Command("enter 5"), "문 앞이 아니면 enter 는 false");
    check(g.Command("leave 5") && g.World().Count() == 0, "leave 명령");
    g.World().SetMaxPlayers(2);
    g.Command("join 1 a"); g.Command("join 2 b");
    check(!g.Command("join 3 c"), "가득 차면 못 들어온다");
}


// 건물 내부: 문 앞에서 B 로 방에 들어가고 나온다. 방 안은 벽과 가구에 막힌다.
const char* kPortals =
    "room h1 house 0 -1 0 100 0 3 3 2.7 1 fff0d6 c9a37a\n"
    "furn h1 100 1.5 1 1 0.75 8a5a3c 1 0\n"
    "furn h1 100 0 2 2 0.03 e86a5f 0 0\n";

void testRooms() {
    PlayerWorld w;
    String err;
    check(w.LoadGround(kGround, &err), "ground");
    check(w.ParsePortals(kPortals, &err), "portals 읽기");
    check(w.RoomCount() == 1 && w.RoomInfo(0).name == "h1", "방 하나");
    check(w.ObstacleCount() == 1, "막히는 가구만 장애물이 된다 (깔개는 아님)");
    w.SetSpawn(0, 0);
    w.Join(1, "a");
    w.Find(1)->x = -1.5; w.Find(1)->y = 1.0;   // 문 앞(0, -1) 에서 2.5 m
    check(!w.Enter(1), "문에서 멀면 아무 일도 없다");
    w.Find(1)->x = 0.5; w.Find(1)->y = -1;
    check(w.Enter(1), "문 앞에서 B");
    const Player& p = *w.Find(1);
    check(w.RoomAt(p.x, p.y) == 0, "방 안에 들어왔다");
    check(near(p.x, 100, 0.6) && near(p.y, -1.5, 0.6), "방 남쪽 문 안쪽");
    check(near(p.z, 1.0), "바닥 높이");
    check(!w.Enter(2), "없는 접속자");
    // 방 안: 문에서 멀어지면 B 가 안 먹고, 벽 너머로는 못 간다
    w.Find(1)->x = 100; w.Find(1)->y = 0;
    check(!w.Enter(1), "방 한가운데서는 나갈 수 없다");
    w.SetInput(1, 1, 0, 1, 3.14159265);   // 방 안쪽(+y) 으로 달린다
    walk(w, 6);
    check(w.Find(1)->y < 3.0 - 0.4, "북쪽 벽에 막힌다");
    check(fabs(w.Find(1)->x - 100) < 3.0, "방 밖으로 못 나간다");
    w.Find(1)->x = 100; w.Find(1)->y = -2.4; w.SetInput(1, 0, 0, 0, 0);
    check(w.Enter(1), "문 앞에서 B 로 나온다");
    check(w.RoomAt(w.Find(1)->x, w.Find(1)->y) < 0 && near(w.Find(1)->x, 0, 0.6) && near(w.Find(1)->y, -1, 0.6), "문 앞(밖)에 나온다");
    // 문 앞이 이미 차 있으면 가까운 빈 곳에
    w.Join(2, "b");
    w.Find(2)->x = 0.5; w.Find(2)->y = -1;
    check(w.Enter(2), "두 번째도 들어간다");
    check(w.RoomAt(w.Find(2)->x, w.Find(2)->y) == 0, "두 번째도 방 안");
    w.Find(1)->x = 0.5; w.Find(1)->y = -1;   // 1 번이 문 앞에 선 채 2 번이 나오면
    w.Find(2)->x = 100; w.Find(2)->y = -2.4;
    check(w.Enter(2), "나온다");
    check(hypot(w.Find(2)->x - w.Find(1)->x, w.Find(2)->y - w.Find(1)->y) >= 0.8, "문 앞이 차 있으면 가까운 빈 곳에 나온다 (겹치지 않는다)");
}

}  // namespace

int main() {
    testWalk();
    testBlocked();
    testGround();
    testChat();
    testCommands();
    testRooms();
    if (g_fail == 0) printf("OK: test_cplayers\n");
    return g_fail == 0 ? 0 : 1;
}
