#include "CSimul/Chat.h"

#include <math.h>

#include "CBase/Text.h"

namespace simul {
namespace sim {

namespace {

// UTF-8 한 글자의 바이트 수 (잘못된 바이트는 1)
size_t CharLen(unsigned char c) {
    if (c < 0x80) return 1;
    if ((c >> 5) == 0x6) return 2;
    if ((c >> 4) == 0xE) return 3;
    if ((c >> 3) == 0x1E) return 4;
    return 1;
}

int CountChars(const String& s) {
    int n = 0;
    for (size_t i = 0; i < s.size(); i += CharLen(static_cast<unsigned char>(s[i]))) n++;
    return n;
}

bool IsSentenceEnd(const String& s, size_t i) {
    const char c = s[i];
    if (c == '.' || c == '!' || c == '?') return true;
    // 。(E3 80 82) ！(EF BC 81) ？(EF BC 9F)
    if (i + 2 < s.size()) {
        const unsigned char a = static_cast<unsigned char>(s[i]), b = static_cast<unsigned char>(s[i + 1]), d = static_cast<unsigned char>(s[i + 2]);
        if (a == 0xE3 && b == 0x80 && d == 0x82) return true;
        if (a == 0xEF && b == 0xBC && (d == 0x81 || d == 0x9F)) return true;
    }
    return false;
}

bool StartsAt(const String& s, size_t i, const char* lit) {
    for (size_t k = 0; lit[k]; k++)
        if (i + k >= s.size() || s[i + k] != lit[k]) return false;
    return true;
}

// 앞뒤의 따옴표류를 뗀다: " ' “ ” ‘ ’ 「 」 『 』 ` 그리고 공백
String StripQuotes(String s) {
    static const char* const kQuotes[] = {"\"", "'", "`", "\xE2\x80\x9C", "\xE2\x80\x9D", "\xE2\x80\x98", "\xE2\x80\x99", "\xE3\x80\x8C", "\xE3\x80\x8D", "\xE3\x80\x8E", "\xE3\x80\x8F"};
    bool again = true;
    while (again) {
        again = false;
        s = Trim(s);
        for (size_t q = 0; q < sizeof(kQuotes) / sizeof(kQuotes[0]); q++) {
            const size_t n = String(kQuotes[q]).size();
            if (s.size() >= n && StartsAt(s, 0, kQuotes[q])) {
                s = s.substr(n);
                again = true;
                break;
            }
            if (s.size() >= n && StartsAt(s, s.size() - n, kQuotes[q])) {
                s = s.substr(0, s.size() - n);
                again = true;
                break;
            }
        }
    }
    return s;
}

void AppendUtf8(String* out, unsigned long cp) {
    if (cp < 0x80) {
        out->push_back(static_cast<char>(cp));
    } else if (cp < 0x800) {
        out->push_back(static_cast<char>(0xC0 | (cp >> 6)));
        out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp < 0x10000) {
        out->push_back(static_cast<char>(0xE0 | (cp >> 12)));
        out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
        out->push_back(static_cast<char>(0xF0 | (cp >> 18)));
        out->push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
        out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
        out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
}

int Hex4(const String& s, size_t i, unsigned long* v) {
    if (i + 4 > s.size()) return 0;
    unsigned long r = 0;
    for (size_t k = 0; k < 4; k++) {
        const char c = s[i + k];
        r <<= 4;
        if (c >= '0' && c <= '9') r |= static_cast<unsigned long>(c - '0');
        else if (c >= 'a' && c <= 'f') r |= static_cast<unsigned long>(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') r |= static_cast<unsigned long>(c - 'A' + 10);
        else return 0;
    }
    *v = r;
    return 1;
}

// json 안에서 "key": "문자열" 을 찾아 풀어 쓴다 (\n \" \\ \uXXXX 와 서로게이트 쌍 포함)
bool JsonString(const String& json, const char* key, String* out) {
    const String pat = String("\"") + key + "\"";
    size_t at = 0;
    for (;;) {
        at = json.find(pat, at);
        if (at == String::npos) return false;
        size_t i = at + pat.size();
        while (i < json.size() && (json[i] == ' ' || json[i] == '\t' || json[i] == '\n' || json[i] == '\r')) i++;
        if (i < json.size() && json[i] == ':') {
            i++;
            while (i < json.size() && (json[i] == ' ' || json[i] == '\t' || json[i] == '\n' || json[i] == '\r')) i++;
            if (i < json.size() && json[i] == '"') {
                i++;
                String r;
                while (i < json.size() && json[i] != '"') {
                    if (json[i] != '\\') {
                        r.push_back(json[i++]);
                        continue;
                    }
                    i++;
                    if (i >= json.size()) return false;
                    const char e = json[i++];
                    switch (e) {
                        case 'n': r.push_back('\n'); break;
                        case 'r': r.push_back('\r'); break;
                        case 't': r.push_back('\t'); break;
                        case 'b': case 'f': break;
                        case 'u': {
                            unsigned long cp = 0;
                            if (!Hex4(json, i, &cp)) return false;
                            i += 4;
                            if (cp >= 0xD800 && cp < 0xDC00 && i + 6 <= json.size() && json[i] == '\\' && json[i + 1] == 'u') {
                                unsigned long lo = 0;
                                if (Hex4(json, i + 2, &lo) && lo >= 0xDC00 && lo < 0xE000) {
                                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                                    i += 6;
                                }
                            }
                            AppendUtf8(&r, cp);
                            break;
                        }
                        default: r.push_back(e); break;   // \" \\ \/
                    }
                }
                if (i >= json.size()) return false;   // 닫는 따옴표가 없다
                *out = r;
                return true;
            }
        }
        at += pat.size();
    }
}

const char* const kGreet[] = {"안녕하세요! 오늘도 좋은 날이네요.", "어머, 만나서 반가워요!", "이런 데서 다 만나네요. 안녕하세요!", "반가워요! 산책 중이세요?"};
const char* const kMiddle[] = {"그러게요, 마을이 참 평화로워요.", "저도 그렇게 생각해요.", "요즘 날씨가 정말 좋죠?", "아, 그 얘기 저도 들었어요!", "하하, 정말 재미있네요.", "길가의 꽃이 예쁘게 피었어요."};
const char* const kBye[] = {"그럼 다음에 또 봐요!", "즐거웠어요. 조심히 가세요!", "저는 이만 가 볼게요. 안녕히!", "다음에 또 이야기해요!"};

String NoPeriod(String s) {   // 문장 끝의 마침표를 뗀다 (프롬프트에서 다시 붙인다)
    s = Trim(s);
    while (!s.empty() && s[s.size() - 1] == '.') s.erase(s.size() - 1);
    return s;
}

size_t HashStr(const String& s) {
    size_t h = 1469598103u;
    for (size_t i = 0; i < s.size(); i++) h = (h ^ static_cast<unsigned char>(s[i])) * 16777619u;
    return h;
}

}  // namespace

Chat::Chat() : patrol_(nullptr), meet_(2.2), cooldown_(30.0), time_(0.0), turns_(4), nextId_(1), started_(0), finished_(0), lines_(0), fallbackLines_(0), enabled_(true) {}

bool Chat::ParsePersonas(const String& text, String* error) {
    Array<Persona> list;
    simul::Lines lines(text);
    String raw;
    int no = 0;
    while (lines.Next(&raw)) {
        no++;
        const String line = Trim(raw);
        if (line.empty() || line[0] == '#') continue;
        const String where = "주민 설정 파일 " + ToString(no) + "번째 줄: ";
        if (line[0] == '[') {
            const size_t close = line.find(']');
            if (close == String::npos || close < 2) {
                if (error) *error = where + "[이름] 형식이 아닙니다";
                return false;
            }
            Persona p;
            p.id = Trim(line.substr(1, close - 1));
            list.push_back(p);
            continue;
        }
        if (list.empty()) {
            if (error) *error = where + "[이름] 보다 먼저 설정이 나왔습니다";
            return false;
        }
        Persona& p = list[list.size() - 1];
        if (StartsWith(line, "name")) {
            p.name = Trim(line.substr(4));
        } else if (StartsWith(line, "persona")) {
            p.text = Trim(line.substr(7));
        } else {
            if (error) *error = where + "name 이나 persona 로 시작해야 합니다";
            return false;
        }
    }
    for (size_t i = 0; i < list.size(); i++) {
        if (list[i].id.empty()) {
            if (error) *error = "이름이 빈 [ ] 가 있습니다";
            return false;
        }
        if (list[i].name.empty()) list[i].name = list[i].id;
    }
    personas_ = list;
    return true;
}

void Chat::AddMissingPersonas(const Patrol& patrol) {
    for (int r = 0; r < patrol.RouteCount(); r++) {
        const String& id = patrol.Route(r).name;
        bool have = false;
        for (size_t i = 0; i < personas_.size(); i++) have = have || personas_[i].id == id;
        if (have) continue;
        Persona p;
        p.id = id;
        p.name = id;
        p.text = "마을 주민";
        personas_.push_back(p);
    }
}

int Chat::convOf(int persona) const {
    for (size_t i = 0; i < convs_.size(); i++)
        if (convs_[i].a == persona || convs_[i].b == persona) return static_cast<int>(i);
    return -1;
}

bool Chat::Talking(const String& id) const {
    for (size_t i = 0; i < personas_.size(); i++)
        if (personas_[i].id == id) return convOf(static_cast<int>(i)) >= 0;
    return false;
}

int Chat::convByRequest(int id) const {
    for (size_t i = 0; i < convs_.size(); i++)
        if (convs_[i].phase == 1 && convs_[i].reqId == id) return static_cast<int>(i);
    return -1;
}

bool Chat::coolingDown(int a, int b, double time) const {
    for (size_t i = 0; i < cools_.size(); i++)
        if (((cools_[i].a == a && cools_[i].b == b) || (cools_[i].a == b && cools_[i].b == a)) && time < cools_[i].until) return true;
    return false;
}

void Chat::makeRequest(Conv& c, double time) {
    const bool aFirst = (c.turn % 2) == 0;
    const Persona& sp = personas_[static_cast<size_t>(aFirst ? c.a : c.b)];
    const Persona& li = personas_[static_cast<size_t>(aFirst ? c.b : c.a)];
    ChatRequest r;
    r.id = nextId_++;
    r.speaker = sp.name;
    r.listener = li.name;
    r.persona = sp.text;
    r.listenerPersona = li.text;
    r.history = c.history;
    r.turn = c.turn;
    r.turns = c.turns;
    c.req = r;
    c.reqId = r.id;
    c.reqTime = time;
    c.phase = 1;
    pending_.push_back(r);
}

void Chat::say(Conv& c, const String& rawLine, bool fallback) {
    String line = CleanLine(rawLine);
    if (line.empty()) {
        line = Fallback(c.req);
        fallback = true;
    }
    const bool aFirst = (c.turn % 2) == 0;
    const Persona& sp = personas_[static_cast<size_t>(aFirst ? c.a : c.b)];
    const Persona& li = personas_[static_cast<size_t>(aFirst ? c.b : c.a)];
    double dur = 2.0 + 0.18 * CountChars(line);
    if (dur < 3.0) dur = 3.0;
    if (dur > 7.0) dur = 7.0;
    Bubble b;
    b.id = sp.id;
    b.text = line;
    b.until = time_ + dur;
    for (size_t i = 0; i < bubbles_.size(); i++)
        if (bubbles_[i].id == sp.id) {
            bubbles_.erase(bubbles_.begin() + static_cast<long>(i));
            break;
        }
    bubbles_.push_back(b);
    c.history.push_back(sp.name + ": " + line);
    log_.push_back(sp.name + " → " + li.name + ": " + line + (fallback ? "  (정해 둔 대사)" : ""));
    c.until = time_ + dur + 0.6;   // 다 읽은 뒤 잠깐 쉬었다가 상대 차례
    c.phase = 2;
    lines_++;
    if (fallback) fallbackLines_++;
}

bool Chat::Deliver(int id, const String& text) {
    const int ci = convByRequest(id);
    if (ci < 0) return false;
    say(convs_[static_cast<size_t>(ci)], text, false);
    return true;
}

bool Chat::Fail(int id) {
    const int ci = convByRequest(id);
    if (ci < 0) return false;
    say(convs_[static_cast<size_t>(ci)], Fallback(convs_[static_cast<size_t>(ci)].req), true);
    return true;
}

bool Chat::PopRequest(ChatRequest* out) {
    if (pending_.empty()) return false;
    *out = pending_[0];
    pending_.erase(pending_.begin());
    return true;
}

Array<String> Chat::TakeLog() {
    Array<String> r = log_;
    log_.clear();
    return r;
}

void Chat::finish(Engine& e, size_t index, double time) {
    const Conv c = convs_[index];
    if (patrol_) {
        patrol_->Release(e, personas_[static_cast<size_t>(c.a)].id);
        patrol_->Release(e, personas_[static_cast<size_t>(c.b)].id);
    }
    Cool k;
    k.a = c.a;
    k.b = c.b;
    k.until = time + cooldown_;
    cools_.push_back(k);
    log_.push_back(personas_[static_cast<size_t>(c.a)].name + " 와(과) " + personas_[static_cast<size_t>(c.b)].name + " 의 대화가 끝났다");
    convs_.erase(convs_.begin() + static_cast<long>(index));
    finished_++;
}

void Chat::Tick(Engine& e, double time) {
    time_ = time;
    for (size_t i = bubbles_.size(); i > 0; i--)
        if (bubbles_[i - 1].until <= time) bubbles_.erase(bubbles_.begin() + static_cast<long>(i - 1));
    for (size_t i = cools_.size(); i > 0; i--)
        if (cools_[i - 1].until <= time) cools_.erase(cools_.begin() + static_cast<long>(i - 1));

    // 진행 중인 대화
    for (size_t i = 0; i < convs_.size();) {
        Conv& c = convs_[i];
        if (time - c.started > 150.0) {   // 너무 길어지면 끊는다
            finish(e, i, time);
            continue;
        }
        if (c.phase == 0) {
            makeRequest(c, time);
        } else if (c.phase == 1) {
            if (time - c.reqTime > 40.0) Fail(c.reqId);   // 답이 안 오면 정해 둔 대사로
        } else if (time >= c.until) {
            c.turn++;
            if (c.turn >= c.turns) {
                finish(e, i, time);
                continue;
            }
            c.phase = 0;
        }
        i++;
    }

    // 새 만남
    if (!enabled_ || !patrol_) return;
    const size_t n = personas_.size();
    Array<dhVector> pos(n);
    Array<char> ok(n, 0);
    for (size_t i = 0; i < n; i++) {
        if (convOf(static_cast<int>(i)) >= 0 || !patrol_->Active(personas_[i].id) || patrol_->Held(personas_[i].id)) continue;
        ObjectInfo info;
        const Handle h = e.Find(personas_[i].id);
        if (h.slot >= 0 && e.Info(h, &info)) {
            pos[i] = info.pose.position;
            ok[i] = 1;
        }
    }
    for (size_t i = 0; i < n; i++) {
        for (size_t j = i + 1; j < n; j++) {
            if (!ok[i] || !ok[j] || convOf(static_cast<int>(i)) >= 0 || convOf(static_cast<int>(j)) >= 0) continue;
            const double d = hypot(static_cast<double>(pos[i].x) - static_cast<double>(pos[j].x), static_cast<double>(pos[i].y) - static_cast<double>(pos[j].y));
            if (d > meet_ || coolingDown(static_cast<int>(i), static_cast<int>(j), time)) continue;
            Conv c;
            c.a = static_cast<int>(i);
            c.b = static_cast<int>(j);
            c.turns = turns_;
            c.started = time;
            patrol_->Hold(e, personas_[i].id, static_cast<double>(pos[j].x), static_cast<double>(pos[j].y));
            patrol_->Hold(e, personas_[j].id, static_cast<double>(pos[i].x), static_cast<double>(pos[i].y));
            convs_.push_back(c);
            started_++;
            log_.push_back(personas_[i].name + " 와(과) " + personas_[j].name + " 이(가) 마주쳐서 이야기를 시작한다");
        }
    }
}

// ---------------------------------------------------------------------------------------------------- 글 다루기

String Chat::BuildPrompt(const ChatRequest& r) {
    String p;
    p += "당신은 동화 속 마을의 주민 \"" + r.speaker + "\" 입니다.";
    if (!r.persona.empty()) p += " 성격: " + NoPeriod(r.persona) + ".";
    p += "\n지금 마을 길에서 \"" + r.listener + "\"";
    if (!r.listenerPersona.empty()) p += " (" + NoPeriod(r.listenerPersona) + ")";
    p += " 을(를) 만나 이야기하는 중입니다.\n";
    if (r.history.empty()) {
        p += "아직 아무 말도 나누지 않았습니다. 먼저 말을 거세요.\n";
    } else {
        p += "지금까지의 대화:\n";
        for (size_t i = 0; i < r.history.size(); i++) p += r.history[i] + "\n";
    }
    if (r.turn + 1 >= r.turns) p += "이번이 마지막 말입니다. 인사하며 대화를 마무리하세요.\n";
    p += "\"" + r.speaker + "\" 의 다음 말을 한국어 한 문장(30자 이내)으로만 쓰세요. 이름, 따옴표, 설명 없이 대사만 쓰세요.";
    return p;
}

String Chat::JsonEscape(const String& s) {
    String r;
    for (size_t i = 0; i < s.size(); i++) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        switch (c) {
            case '"': r += "\\\""; break;
            case '\\': r += "\\\\"; break;
            case '\n': r += "\\n"; break;
            case '\r': r += "\\r"; break;
            case '\t': r += "\\t"; break;
            default:
                if (c < 0x20) {
                    static const char* const kHex = "0123456789abcdef";
                    r += "\\u00";
                    r.push_back(kHex[c >> 4]);
                    r.push_back(kHex[c & 15]);
                } else {
                    r.push_back(static_cast<char>(c));
                }
        }
    }
    return r;
}

String Chat::BuildOllamaBody(const String& model, const String& prompt, int maxTokens) {
    return "{\"model\":\"" + JsonEscape(model) + "\",\"prompt\":\"" + JsonEscape(prompt) + "\",\"stream\":false,\"options\":{\"num_predict\":" + ToString(maxTokens) +
           ",\"temperature\":0.9}}";
}

bool Chat::ExtractResponse(const String& json, String* out) {
    if (JsonString(json, "response", out)) return true;
    return JsonString(json, "content", out);
}

String Chat::CleanLine(const String& raw) {
    String line;
    simul::Lines lines(raw);
    String l;
    while (lines.Next(&l)) {
        l = Trim(l);
        if (!l.empty()) {
            line = l;
            break;
        }
    }
    line = StripQuotes(line);
    // "한솔: 안녕" / "[한솔] 안녕" 처럼 이름이 앞에 붙어 오면 뗀다
    const size_t colon = line.find_first_of(":");
    const size_t fcolon = line.find("\xEF\xBC\x9A");   // ：
    size_t cut = String::npos, clen = 1;
    if (colon != String::npos && colon <= 24) cut = colon;
    if (fcolon != String::npos && fcolon <= 24 && (cut == String::npos || fcolon < cut)) {
        cut = fcolon;
        clen = 3;
    }
    if (cut != String::npos) {
        bool notName = CountChars(line.substr(0, cut)) > 10;   // 이름 접두어는 짧고 숫자와 문장부호가 없다
        for (size_t i = 0; i < cut; i++) notName = notName || IsSentenceEnd(line, i) || (line[i] >= '0' && line[i] <= '9');
        if (!notName && cut + clen < line.size()) line = line.substr(cut + clen);
    }
    line = StripQuotes(line);
    // 40 글자 안으로: 넘으면 그 안의 마지막 문장 끝에서 자르고, 없으면 40 글자에서 자르고 …
    int count = 0;
    size_t i = 0, lastEnd = 0;
    int lastEndCount = 0;
    while (i < line.size()) {
        const size_t len = CharLen(static_cast<unsigned char>(line[i]));
        if (count >= 40) break;
        if (IsSentenceEnd(line, i)) {
            const size_t elen = static_cast<unsigned char>(line[i]) >= 0x80 ? 3 : 1;
            lastEnd = i + elen;
            lastEndCount = count + 1;
        }
        i += len;
        count++;
    }
    if (i < line.size()) {   // 40 글자를 넘는다
        if (lastEnd > 0 && lastEndCount >= 4) line = line.substr(0, lastEnd);
        else line = line.substr(0, i) + "…";
    }
    return Trim(line);
}

String Chat::Fallback(const ChatRequest& r) {
    const size_t h = HashStr(r.speaker) + static_cast<size_t>(r.id) * 7u + static_cast<size_t>(r.turn) * 3u;
    if (r.turn == 0) return kGreet[h % (sizeof(kGreet) / sizeof(kGreet[0]))];
    if (r.turn + 1 >= r.turns) return kBye[h % (sizeof(kBye) / sizeof(kBye[0]))];
    return kMiddle[h % (sizeof(kMiddle) / sizeof(kMiddle[0]))];
}

}  // namespace sim
}  // namespace simul
