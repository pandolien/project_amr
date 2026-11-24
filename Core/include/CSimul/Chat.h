// 주민 대화: 순찰하던 두 주민이 마주치면 멈춰서 서로를 보고, 번갈아 몇 마디 나눈 뒤 다시 걷는다. (Qt 와 네트워크에 의존하지 않는다)
//
//   이 클래스는 "누가 언제 무슨 요청을 하는지"만 정한다. 대사 자체는 밖에서 채운다:
//     1. Tick() 이 두 주민이 가까워진 것을 보면 Patrol::Hold 로 세우고, 첫 대사 요청(ChatRequest)을 쌓는다.
//     2. 바깥(앱)이 PopRequest() 로 요청을 꺼내서 LLM 에 묻고 (BuildPrompt / BuildOllamaBody), 답이 오면 Deliver(id, 글),
//        실패하거나 LLM 을 안 쓰면 Fail(id) 를 부른다 (Fail 은 정해 둔 대사로 대신한다. 그래서 LLM 이 없어도 말은 한다).
//     3. 대사는 말풍선(Bubbles)으로 잠깐 보이고, 다 읽을 시간이 지나면 상대의 차례가 된다.
//     4. 정한 횟수만큼 주고받으면 Patrol::Release 로 다시 걷게 하고, 한동안 둘이 또 대화하지 않는다.
//
//   주민 설정 파일 (personas.txt):
//     # 주석
//     [villager_1]              <- 모델 이름 (순찰 지점 파일의 [이름] 과 같다)
//     name 한솔                 <- 대사에 쓰는 이름 (생략하면 모델 이름)
//     persona 느긋하고 정이 많은 농부. 날씨와 농사 이야기를 즐긴다.
#ifndef CORE_CSIMUL_CHAT_H
#define CORE_CSIMUL_CHAT_H

#include "CBase/Array.h"
#include "CBase/String.h"
#include "CSimul/Engine.h"
#include "CSimul/Patrol.h"

namespace simul {
namespace sim {

struct Persona {
    String id;     // 모델 이름
    String name;   // 대사에 쓰는 이름
    String text;   // 성격과 말투
};

// LLM 에 물어볼 한 마디
struct ChatRequest {
    int id;
    String speaker, listener;                 // 표시 이름
    String persona, listenerPersona;          // 각자의 성격
    Array<String> history;                    // 지금까지 나눈 말 ("이름: 대사")
    int turn, turns;                          // 몇 번째 말인지 (0 부터), 전체 몇 마디인지
    ChatRequest() : id(0), turn(0), turns(0) {}
};

// 주민 머리 위에 잠깐 보이는 말풍선
struct Bubble {
    String id;     // 말하는 주민의 모델 이름
    String text;
    double until;  // 시뮬레이션 시각 (s) 까지 보인다
    Bubble() : until(0.0) {}
};

class Chat {
public:
    Chat();

    // 주민 설정 파일을 읽는다. 틀리면 false 와 이유(줄 번호 포함). 실패하면 이전 내용은 그대로 둔다.
    bool ParsePersonas(const String& text, String* error);
    // 순찰 길은 있는데 설정이 없는 주민은 이름 = 모델 이름, 성격 = "마을 주민" 으로 채운다.
    void AddMissingPersonas(const Patrol& patrol);
    int PersonaCount() const { return static_cast<int>(personas_.size()); }
    const Persona& GetPersona(int i) const { return personas_[static_cast<size_t>(i)]; }

    void SetPatrol(Patrol* patrol) { patrol_ = patrol; }
    void SetMeetDistance(double meters) { meet_ = meters; }      // 이만큼 가까워지면 말을 건다 (기본 2.2 m)
    void SetTurns(int turns) { turns_ = turns < 1 ? 1 : turns; } // 한 번 만나서 주고받는 말 수 (기본 4)
    void SetCooldown(double seconds) { cooldown_ = seconds; }    // 같은 둘이 다시 대화하기까지 (기본 30 s)
    void SetEnabled(bool on) { enabled_ = on; }                  // 끄면 새 대화를 시작하지 않는다 (하던 대화는 끝낸다)

    // 시뮬레이션 시각 time 까지 진행한다. 만남을 찾고, 대화를 한 단계씩 나아가게 한다.
    void Tick(Engine& e, double time);

    bool PopRequest(ChatRequest* out);          // 새 요청이 있으면 꺼낸다
    bool Deliver(int id, const String& text);   // 요청 id 의 답. 정리해서 말풍선으로 올린다. 비었거나 이상하면 정해 둔 대사로 대신한다.
    bool Fail(int id);                          // 요청 id 에 답을 못 받았다: 정해 둔 대사로 대신한다
    const Array<Bubble>& Bubbles() const { return bubbles_; }
    Array<String> TakeLog();                    // 쌓인 한 줄 기록 ("한솔 → 마리: 안녕!")을 꺼내고 비운다
    bool Talking(const String& id) const;       // 지금 대화 중인 주민인가

    int Started() const { return started_; }       // 시작한 대화 수
    int Finished() const { return finished_; }     // 끝난 대화 수
    int Lines() const { return lines_; }           // 말풍선으로 올라간 대사 수
    int FallbackLines() const { return fallbackLines_; }   // 그중 정해 둔 대사로 대신한 수

    // ---- 글 다루기 (정적, 네트워크 없음) ----
    static String BuildPrompt(const ChatRequest& r);                                  // LLM 에 보낼 한국어 지시문
    static String BuildOllamaBody(const String& model, const String& prompt, int maxTokens = 64);   // Ollama /api/generate 요청 본문 (JSON)
    static bool ExtractResponse(const String& json, String* out);                     // 응답 JSON 의 "response" (없으면 "content") 글
    static String JsonEscape(const String& s);
    static String CleanLine(const String& raw);                                       // 첫 줄만, 따옴표와 "이름:" 을 떼고, 40자 안으로 줄인다
    static String Fallback(const ChatRequest& r);                                     // 정해 둔 대사 (LLM 이 없을 때)

private:
    struct Conv {
        int a, b;           // personas_ 번호
        int turn, turns;
        int phase;          // 0 요청할 차례, 1 답을 기다림, 2 말풍선을 보여 주는 중
        int reqId;
        double started, reqTime, until;
        ChatRequest req;    // 마지막 요청 (실패하면 정해 둔 대사를 만드는 데 쓴다)
        Array<String> history;
        Conv() : a(0), b(0), turn(0), turns(0), phase(0), reqId(0), started(0), reqTime(0), until(0) {}
    };
    struct Cool {
        int a, b;
        double until;
    };

    int convOf(int persona) const;
    int convByRequest(int id) const;
    bool coolingDown(int a, int b, double time) const;
    void makeRequest(Conv& c, double time);
    void say(Conv& c, const String& line, bool fallback);
    void finish(Engine& e, size_t index, double time);

    Array<Persona> personas_;
    Patrol* patrol_;
    Array<Conv> convs_;
    Array<Cool> cools_;
    Array<ChatRequest> pending_;
    Array<Bubble> bubbles_;
    Array<String> log_;
    double meet_, cooldown_, time_;
    int turns_, nextId_, started_, finished_, lines_, fallbackLines_;
    bool enabled_;
};

}  // namespace sim
}  // namespace simul

#endif  // CORE_CSIMUL_CHAT_H
