#ifndef CORE_INTERNAL_VERTEXKEY_H
#define CORE_INTERNAL_VERTEXKEY_H

#include <stddef.h>

// 메시 로더가 같은 정점을 합칠 때 쓰는 키: (위치, uv, 법선) 인덱스 세 개 (없으면 -1). HashMap 의 키로 쓴다.
namespace amr {
namespace detail {

struct VertexKey {
    int v[3];
    VertexKey() { v[0] = v[1] = v[2] = 0; }
    int& operator[](int i) { return v[i]; }
    const int& operator[](int i) const { return v[i]; }
    bool operator==(const VertexKey& o) const { return v[0] == o.v[0] && v[1] == o.v[1] && v[2] == o.v[2]; }
    size_t Hash() const {
        size_t h = 1469598103u;
        for (int i = 0; i < 3; i++) h = (h ^ static_cast<size_t>(static_cast<unsigned>(v[i]))) * 16777619u;
        return h ^ (h >> 15);
    }
};

}  // namespace detail
}  // namespace amr

#endif  // CORE_INTERNAL_VERTEXKEY_H
