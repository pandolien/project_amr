// CSimul 주민 대화 테스트: 설정 읽기, 만나서 멈추기, 번갈아 말하기, 다시 걷기, 정해 둔 대사, LLM 요청/응답 글 다루기
#include <math.h>
#include <stdio.h>

#include "CSimul/Chat.h"
#include "FakeSimBackend.h"
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

const char* kPersonas =
    "# 주석\n"
    "[a]\n"
    "name 한솔\n"
    "persona 느긋한 농부\n"
    "\n"
    "[b]\n"
    "persona 호기심 많은 소녀\n";

dhVector posOf(Engine& e, const char* name) {
    ObjectInfo info;
    e.Info(e.Find(name), &info);
    return info.pose.position;
}

void testPersonas() {
    Chat c;
    String err;
    check(c.ParsePersonas(kPersonas, &err) && c.PersonaCount() == 2, "설정을 읽는다");
    check(c.GetPersona(0).name == "한솔" && c.GetPersona(0).text == "느긋한 농부", "이름과 성격");
    check(c.GetPersona(1).name == "b", "name 이 없으면 모델 이름");
    check(!c.ParsePersonas("name 가나\n", &err) && err.find("1번째 줄") != String::npos, "[이름] 보다 먼저 나오면 오류");
    check(!c.ParsePersonas("[a]\nfoo bar\n", &err), "모르는 줄은 오류");
    check(!c.ParsePersonas("[]\n", &err), "이름이 비면 오류");
    check(c.PersonaCount() == 2, "실패하면 이전 내용을 유지");

    Patrol p;
    p.Parse("[a]\n0 0 0\n1 0 0\n[z]\n0 0 0\n1 0 0\n", &err);
    c.AddMissingPersonas(p);
    check(c.PersonaCount() == 3 && c.GetPersona(2).id == "z", "설정에 없는 주민은 모델 이름으로 채운다");
}

void testText() {
    String out;
    check(Chat::ExtractResponse("{\"model\":\"m\",\"response\":\"안녕\\n하세요 \\\"휴\\\" \\u00e9\\ud83d\\ude00\",\"done\":true}", &out), "response 를 뽑는다");
    check(out == String("안녕\n하세요 \"휴\" \xC3\xA9\xF0\x9F\x98\x80"), "이스케이프와 서로게이트 쌍을 푼다");
    check(Chat::ExtractResponse("{\"content\": \"반가워요\"}", &out) && out == "반가워요", "content 도 읽는다 (llama.cpp 서버)");
    check(Chat::ExtractResponse("{\"a\":{\"response\" : \"깊은\"}}", &out) && out == "깊은", "공백과 중첩");
    check(!Chat::ExtractResponse("{\"error\":\"x\"}", &out), "없으면 false");
    check(!Chat::ExtractResponse("{\"response\":\"닫히지 않음", &out), "닫는 따옴표가 없으면 false");

    check(Chat::JsonEscape("a\"b\\c\n\t\x01") == "a\\\"b\\\\c\\n\\t\\u0001", "JSON 이스케이프");
    const String body = Chat::BuildOllamaBody("m1", "질문 \"x\"\n끝", 50);
    check(body.find("\"model\":\"m1\"") != String::npos && body.find("\"stream\":false") != String::npos && body.find("\"num_predict\":50") != String::npos, "요청 본문");
    check(body.find("질문 \\\"x\\\"\\n끝") != String::npos, "프롬프트가 이스케이프돼 들어간다");

    check(Chat::CleanLine("\n  \"안녕하세요!\"  \n둘째 줄") == "안녕하세요!", "첫 줄만, 따옴표 제거");
    check(Chat::CleanLine("한솔: 오늘 날씨 좋네요") == "오늘 날씨 좋네요", "앞의 이름 제거");
    check(Chat::CleanLine("\xE3\x80\x8C" "좋은 아침\xE3\x80\x8D") == "좋은 아침", "「」 제거");
    check(Chat::CleanLine("시각은 12:30 입니다") == "시각은 12:30 입니다", "문장 속 시각의 콜론은 그대로 (이름이 아니다)");
    check(Chat::CleanLine("   ") == "", "빈 글");
    const String longText = "가나다라마바사아자차카타파하가나다라마바사아자차카타파하가나다라마바사아자차카타파하가나다라마바사아자차카타파하";
    const String cutLong = Chat::CleanLine(longText);
    check(cutLong.size() == 40 * 3 + 3 && cutLong.substr(cutLong.size() - 3) == "…", "긴 글은 40 글자에서 자르고 …");
    check(Chat::CleanLine("안녕하세요! 오늘은 날씨가 정말 좋아서 산책을 나왔는데 길에서 만나서 반갑습니다 정말로") == "안녕하세요!", "길면 문장 끝에서 자른다");

    ChatRequest r;
    r.speaker = "한솔";
    r.listener = "마리";
    r.persona = "느긋한 농부.";
    r.turns = 4;
    String prompt = Chat::BuildPrompt(r);
    check(prompt.find("\"한솔\"") != String::npos && prompt.find("느긋한 농부") != String::npos && prompt.find("먼저 말을 거세요") != String::npos && prompt.find("농부..") == String::npos, "첫 말 프롬프트 (마침표가 겹치지 않는다)");
    r.turn = 3;
    r.history.push_back("한솔: 안녕");
    prompt = Chat::BuildPrompt(r);
    check(prompt.find("한솔: 안녕") != String::npos && prompt.find("마지막 말") != String::npos, "대화 기록과 마무리 지시");

    ChatRequest g0, gm, gl;
    g0.turn = 0; g0.turns = 4; g0.id = 1; g0.speaker = "a";
    gm.turn = 1; gm.turns = 4; gm.id = 2; gm.speaker = "a";
    gl.turn = 3; gl.turns = 4; gl.id = 3; gl.speaker = "a";
    check(!Chat::Fallback(g0).empty() && !Chat::Fallback(gm).empty() && !Chat::Fallback(gl).empty(), "정해 둔 대사가 있다");
    check(Chat::Fallback(gl).find("봐요") != String::npos || Chat::Fallback(gl).find("가세요") != String::npos || Chat::Fallback(gl).find("안녕히") != String::npos ||
              Chat::Fallback(gl).find("이야기해요") != String::npos,
          "마지막 말은 작별 인사");
}

struct World {
    FakeBackend b;
    Patrol p;
    Chat c;
    Engine e;
    World() : e(&b) {
        b.add("a", false);
        b.add("b", false);
        String err;
        p.Parse("[a]\nspeed 1\n0 0 0\n12 0 0\n[b]\nspeed 1\n12 0 0\n0 0 0\n", &err);
        c.ParsePersonas(kPersonas, &err);
        c.SetPatrol(&p);
        e.SetListener(&p);
        e.Start();
    }
    void step() {
        e.Step(0.02);
        c.Tick(e, e.Time());
    }
};

void testConversation() {
    World w;
    double t = 0;
    while (w.c.Started() == 0 && t < 20.0) {
        w.step();
        t += 0.02;
    }
    check(w.c.Started() == 1 && t > 2.0 && t < 8.0, "마주 오다가 가까워지면 대화가 시작된다");
    check(w.p.Held("a") && w.p.Held("b") && w.c.Talking("a") && w.c.Talking("b"), "둘 다 세워진다");
    const dhVector a0 = posOf(w.e, "a"), b0 = posOf(w.e, "b");
    for (int i = 0; i < 100; i++) w.step();
    const dhVector a1 = posOf(w.e, "a"), b1 = posOf(w.e, "b");
    check(hypot(static_cast<double>(a1.x - a0.x), static_cast<double>(a1.y - a0.y)) < 0.05 && hypot(static_cast<double>(b1.x - b0.x), static_cast<double>(b1.y - b0.y)) < 0.05, "대화 중에는 제자리에 서 있다");
    ObjectInfo ia;
    w.e.Info(w.e.Find("a"), &ia);
    const dhVector front = ia.pose.orientation.Rotate(dhVector(0, -1, 0));   // 모델 앞쪽
    const double toB = atan2(static_cast<double>(b1.y - a1.y), static_cast<double>(b1.x - a1.x));
    check(cos(atan2(static_cast<double>(front.y), static_cast<double>(front.x)) - toB) > 0.98, "서로 마주 본다");

    // 요청에 답을 채운다: 첫 요청은 a(한솔), 다음은 b, 번갈아
    String order;
    int answered = 0;
    for (int guard = 0; guard < 4000 && w.c.Finished() == 0; guard++) {
        ChatRequest r;
        while (w.c.PopRequest(&r)) {
            order += r.speaker + ",";
            check(r.turns == 4 && r.turn == answered, "요청의 차례 번호");
            if (answered == 1) {
                check(r.history.size() == 1 && r.history[0].find("한솔: ") == 0, "두 번째 요청에는 앞의 대사가 들어 있다");
                check(Chat::BuildPrompt(r).find("한솔: 하나") != String::npos, "프롬프트에 대화 기록");
            }
            if (answered == 2) w.c.Fail(r.id);   // 한 번은 실패 -> 정해 둔 대사
            else w.c.Deliver(r.id, answered == 0 ? "한솔: \"하나\"" : "둘");
            answered++;
            check(w.c.Bubbles().size() >= 1, "말풍선이 올라간다");
        }
        w.step();
    }
    check(order == "한솔,b,한솔,b,", "번갈아 말한다 (한솔, b, 한솔, b)");
    check(w.c.Finished() == 1 && w.c.Lines() == 4 && w.c.FallbackLines() == 1, "4마디 후 끝, 정해 둔 대사 1개");
    check(!w.p.Held("a") && !w.p.Held("b") && !w.c.Talking("a"), "끝나면 다시 걷는다");
    Array<String> log = w.c.TakeLog();
    check(log.size() >= 6 && log[1].find("한솔 → b: 하나") == 0, "기록에는 이름 접두어가 없는 대사");

    const dhVector a2 = posOf(w.e, "a");
    for (int i = 0; i < 100; i++) w.step();
    check(fabs(static_cast<double>(posOf(w.e, "a").x - a2.x)) > 0.5, "다시 걸어간다");
    // 쿨다운 동안은 같은 둘이 또 말을 걸지 않는다
    for (int i = 0; i < 400 && w.c.Started() < 2; i++) w.step();
    check(w.c.Started() == 1, "쿨다운 중에는 다시 시작하지 않는다");
}

void testNoAnswerFallsBack() {
    World w;
    w.c.SetTurns(2);
    for (int i = 0; i < 20000 && w.c.Finished() == 0; i++) w.step();   // 답을 안 주면 40 초 뒤 정해 둔 대사로 넘어간다
    check(w.c.Finished() == 1 && w.c.Lines() == 2 && w.c.FallbackLines() == 2, "답이 없으면 정해 둔 대사로 끝낸다");
    check(!w.p.Held("a") && !w.p.Held("b"), "그래도 다시 걷는다");
}

void testDisabled() {
    World w;
    w.c.SetEnabled(false);
    for (int i = 0; i < 400; i++) w.step();
    check(w.c.Started() == 0, "끄면 대화가 시작되지 않는다");
}

}  // namespace

int main() {
    testPersonas();
    testText();
    testConversation();
    testNoAnswerFallsBack();
    testDisabled();
    if (g_fail) {
        printf("test_cchat: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_cchat: all passed\n");
    return 0;
}
