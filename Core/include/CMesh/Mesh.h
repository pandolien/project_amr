#ifndef CORE_CMESH_MESH_H
#define CORE_CMESH_MESH_H

#include <stdint.h>

#include "CBase/String.h"
#include "CBase/Array.h"

#include "3d/dhMat.h"
#include "3d/dhVector.h"

// 렌더링과 분리된 메시/장면 데이터. GLB, STL, OBJ, DAE 로더가 모두 이 구조로 채운다.
// 좌표는 파일의 원래 좌표계를 그대로 유지한다 (GLB: Y-up, DAE: up_axis 에 따라 Z-up 으로 변환).
// 변환 행렬은 dhMat(열 우선, 이동 v[12..14]) 이며 OpenGL 행렬과 배치가 같다.
namespace simul {

struct Color {
    float r, g, b, a;
    Color() : r(0.8f), g(0.8f), b(0.8f), a(1.0f) {}
    Color(float pr, float pg, float pb, float pa) : r(pr), g(pg), b(pb), a(pa) {}
};

struct Material {
    simul::String name;
    Color baseColor;
    float metallic;
    float roughness;
    simul::String baseColorTexture;  // 외부 이미지 경로. 없거나 임베디드이면 빈 문자열
    bool doubleSided;
    Material() : metallic(0.0f), roughness(1.0f), doubleSided(false) {}
};

// 삼각형 메시 하나 (glTF 의 primitive 에 해당)
struct Primitive {
    simul::Array<float> positions;   // xyz * 정점 수
    simul::Array<float> normals;     // xyz * 정점 수. 비어 있으면 ComputeNormals() 로 만든다
    simul::Array<float> uvs;         // uv * 정점 수. 없으면 비어 있음
    simul::Array<uint32_t> indices;  // 삼각형 인덱스 (3개씩)
    int material;                   // Scene::materials 인덱스, 없으면 -1

    Primitive() : material(-1) {}
    size_t VertexCount() const { return positions.size() / 3; }
    size_t TriangleCount() const { return indices.size() / 3; }

    // 면적 가중 평균으로 정점 법선을 계산한다 (정점을 공유하지 않으면 면 법선이 된다)
    void ComputeNormals();
    // 배열 크기와 인덱스 범위가 올바른지 검사한다
    bool IsValid() const;
};

struct Mesh {
    simul::String name;
    simul::Array<Primitive> primitives;
};

struct SceneNode {
    simul::String name;
    dhMat local;                 // 부모 기준 변환
    int mesh;                    // Scene::meshes 인덱스, 없으면 -1
    int parent;                  // 루트이면 -1
    simul::Array<int> children;
    SceneNode() : mesh(-1), parent(-1) {}
};

struct Scene {
    simul::String name;
    simul::Array<Material> materials;
    simul::Array<Mesh> meshes;
    simul::Array<SceneNode> nodes;
    simul::Array<int> roots;
    simul::Array<simul::String> warnings;  // 지원하지 않아 무시한 항목 등

    void Clear();
    // 각 노드의 월드 변환 (nodes 와 같은 순서)
    simul::Array<dhMat> WorldTransforms() const;
    // 모든 루트 노드의 변환 앞에 t 를 곱한다 (좌표축 변환, 단위 변환 등)
    void ApplyRootTransform(const dhMat& t);
    // 월드 좌표 기준 경계 상자. 정점이 하나도 없으면 false
    bool ComputeBounds(dhVector* outMin, dhVector* outMax) const;
    size_t TriangleCount() const;
    // 모든 인덱스/참조가 범위 안에 있고 부모-자식 관계가 일관되는지 검사한다
    bool IsValid() const;
};

}  // namespace simul

#endif  // CORE_CMESH_MESH_H
