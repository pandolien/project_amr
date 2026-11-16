#ifndef CORE_CCOLLISION_BVH_H
#define CORE_CCOLLISION_BVH_H

#include <stddef.h>
#include <stdint.h>

#include "3d/dhVector.h"
#include "CBase/Array.h"

// 삼각형 메시의 BVH (경계 상자 계층). 광선이 닿지 않는 삼각형 무리를 통째로 건너뛰어서, 삼각형이 많아도 광선 하나가
// 대략 O(log n) 로 끝난다. 메시 좌표에서 한 번 만들어 두고 (메시가 바뀌지 않으면) 계속 쓴다.
// 분할은 binned SAH (표면적 휴리스틱, 구간 16개). 교차 판정은 RaycastTriangles 와 똑같다 (양면, 평행/퇴화/NaN 삼각형은 건너뜀,
// 정점 범위를 벗어난 인덱스의 삼각형은 건너뜀).
namespace amr {

class TriangleBvh {
public:
    TriangleBvh();

    // positions: 정점마다 xyz 가 stride 개(float)씩 간격으로 놓인 배열, indices: 삼각형 인덱스 (3개씩). 배열은 복사하지 않으므로
    // Raycast 를 부르는 동안 살아 있고 바뀌지 않아야 한다.
    void Build(const float* positions, size_t vertexCount, const uint32_t* indices, size_t indexCount, size_t stride = 3);

    // 광선 origin + t * dir (0 <= t <= tMax) 와 가장 가까운 교차. RaycastTriangles 와 같은 규약.
    bool Raycast(const dhVector& origin, const dhVector& dir, float tMax, float* tHit, size_t* triangle = NULL) const;

    bool empty() const { return nodes_.empty(); }
    size_t TriangleCount() const { return tris_.size(); }  // 트리에 들어간 (유효한) 삼각형 수
    size_t NodeCount() const { return nodes_.size(); }
    size_t Depth() const { return depth_; }

private:
    struct Node {
        float bmin[3], bmax[3];
        uint32_t leftFirst;  // 안쪽 노드: 왼쪽 자식 번호 (오른쪽 자식은 +1). 잎: tris_ 에서의 첫 번호
        uint32_t count;      // 0 이면 안쪽 노드, 아니면 잎의 삼각형 수
    };
    struct BuildCtx;
    void buildNode(BuildCtx& c, uint32_t node, size_t begin, size_t end, size_t depth);

    const float* positions_;
    size_t vertexCount_;
    const uint32_t* indices_;
    size_t stride_;
    Array<Node> nodes_;
    Array<uint32_t> tris_;  // 삼각형 번호 (indices 에서 3 개씩 묶은 순번), 잎 순서대로
    size_t depth_;
};

}  // namespace amr

#endif  // CORE_CCOLLISION_BVH_H
