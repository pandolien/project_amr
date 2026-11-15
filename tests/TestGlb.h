// 테스트용 GLB 생성 도우미: JSON 문자열과 바이너리 버퍼를 GLB 로 묶는다.
#ifndef TESTS_TESTGLB_H
#define TESTS_TESTGLB_H

#include <stdint.h>

#include <cstring>
#include <string>
#include <vector>

namespace testglb {

inline void put32(std::vector<char>* out, uint32_t v) {
    for (int i = 0; i < 4; i++) out->push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}

inline std::vector<char> MakeGlb(const std::string& jsonIn, const std::vector<char>& bin) {
    std::string json = jsonIn;
    while (json.size() % 4) json += ' ';
    std::vector<char> b = bin;
    while (b.size() % 4) b.push_back(0);

    const uint32_t total = 12 + 8 + static_cast<uint32_t>(json.size()) + (b.empty() ? 0 : 8 + static_cast<uint32_t>(b.size()));
    std::vector<char> out;
    put32(&out, 0x46546C67u);  // "glTF"
    put32(&out, 2);
    put32(&out, total);
    put32(&out, static_cast<uint32_t>(json.size()));
    put32(&out, 0x4E4F534Au);  // "JSON"
    out.insert(out.end(), json.begin(), json.end());
    if (!b.empty()) {
        put32(&out, static_cast<uint32_t>(b.size()));
        put32(&out, 0x004E4942u);  // "BIN\0"
        out.insert(out.end(), b.begin(), b.end());
    }
    return out;
}

inline void putf(std::vector<char>* out, float f) {
    char t[4];
    std::memcpy(t, &f, 4);
    out->insert(out->end(), t, t + 4);
}

inline void putu16(std::vector<char>* out, uint16_t v) {
    out->push_back(static_cast<char>(v & 0xFF));
    out->push_back(static_cast<char>(v >> 8));
}

}  // namespace testglb

#endif  // TESTS_TESTGLB_H
