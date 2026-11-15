// STL 로더 (바이너리/ASCII). 정점은 용접하지 않고, 법선은 정점에서 다시 계산한다.
#include <string.h>

#include <locale>
#include <sstream>

#include "CMesh/MeshLoader.h"
#include "CMesh/Primitives.h"
#include "FileUtil.h"

namespace amr {

namespace {

bool LooksBinary(const unsigned char* d, size_t size) {
    if (size < 84) return false;
    uint32_t count = 0;
    memcpy(&count, d + 80, 4);  // 리틀 엔디언 (x86/ARM 대상)
    // 일부 ASCII 파일도 "solid" 로 시작하므로, 크기가 정확히 맞으면 바이너리로 본다
    return static_cast<uint64_t>(84) + static_cast<uint64_t>(count) * 50 == size;
}

bool ParseBinary(const unsigned char* d, size_t size, Primitive* p, std::string* error) {
    uint32_t count = 0;
    memcpy(&count, d + 80, 4);
    if (count == 0) {
        detail::SetError(error, "STL 에 삼각형이 없습니다");
        return false;
    }
    p->positions.resize(static_cast<size_t>(count) * 9);
    p->indices.resize(static_cast<size_t>(count) * 3);
    for (size_t t = 0; t < count; t++) {
        const unsigned char* rec = d + 84 + t * 50 + 12;  // 면 법선(12B)은 건너뛴다
        memcpy(&p->positions[t * 9], rec, 36);
        for (size_t k = 0; k < 9; k++) {
            const float f = p->positions[t * 9 + k];
            if (!(f == f) || f > 1e30f || f < -1e30f) {  // NaN/Inf 방지
                detail::SetError(error, "STL 좌표가 올바르지 않습니다");
                return false;
            }
        }
        for (size_t k = 0; k < 3; k++) p->indices[t * 3 + k] = static_cast<uint32_t>(t * 3 + k);
    }
    (void)size;
    return true;
}

bool ParseAscii(const std::string& text, Primitive* p, std::string* error) {
    std::istringstream in(text);
    in.imbue(std::locale::classic());
    std::string word;
    size_t pending = 0;
    while (in >> word) {
        if (word != "vertex") continue;
        float x, y, z;
        if (!(in >> x >> y >> z) || !(x == x) || !(y == y) || !(z == z)) {
            detail::SetError(error, "ASCII STL 의 vertex 줄이 올바르지 않습니다");
            return false;
        }
        p->positions.push_back(x);
        p->positions.push_back(y);
        p->positions.push_back(z);
        pending++;
    }
    if (pending == 0 || pending % 3 != 0) {
        detail::SetError(error, pending == 0 ? "STL 에 삼각형이 없습니다" : "ASCII STL 의 정점 수가 3의 배수가 아닙니다");
        return false;
    }
    p->indices.resize(pending);
    for (size_t i = 0; i < pending; i++) p->indices[i] = static_cast<uint32_t>(i);
    return true;
}

}  // namespace

bool LoadStlFromMemory(const void* data, size_t size, Scene* out, std::string* error) {
    if (!out) return false;
    out->Clear();
    if (!data || size == 0) {
        detail::SetError(error, "데이터가 비어 있습니다");
        return false;
    }
    const unsigned char* d = static_cast<const unsigned char*>(data);
    Primitive prim;
    bool ok;
    if (LooksBinary(d, size)) {
        ok = ParseBinary(d, size, &prim, error);
    } else {
        // "solid" 로 시작하는 텍스트만 ASCII 로 인정한다
        size_t i = 0;
        while (i < size && (d[i] == ' ' || d[i] == '\t' || d[i] == '\r' || d[i] == '\n')) i++;
        if (size - i < 5 || memcmp(d + i, "solid", 5) != 0) {
            detail::SetError(error, "STL 형식이 아닙니다 (바이너리 크기 불일치, ASCII 헤더 없음)");
            return false;
        }
        ok = ParseAscii(std::string(reinterpret_cast<const char*>(d), size), &prim, error);
    }
    if (!ok) return false;
    prim.ComputeNormals();
    *out = SceneFromPrimitive(prim, "stl");
    return true;
}

bool LoadStl(const std::string& path, Scene* out, std::string* error) {
    if (!out) return false;
    out->Clear();
    std::vector<char> bytes;
    if (!detail::ReadFileBytes(path, &bytes, error)) return false;
    if (bytes.empty()) {
        detail::SetError(error, "빈 파일입니다: " + path);
        return false;
    }
    return LoadStlFromMemory(&bytes[0], bytes.size(), out, error);
}

}  // namespace amr
