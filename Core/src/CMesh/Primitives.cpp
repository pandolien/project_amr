#include "CMesh/Primitives.h"

#include "CBase/Math.h"

namespace simul {

namespace {

const float kPi = 3.14159265358979f;

void AddVertex(Primitive* p, float x, float y, float z, float nx, float ny, float nz, float u, float v) {
    p->positions.push_back(x);
    p->positions.push_back(y);
    p->positions.push_back(z);
    p->normals.push_back(nx);
    p->normals.push_back(ny);
    p->normals.push_back(nz);
    p->uvs.push_back(u);
    p->uvs.push_back(v);
}

void AddTriangle(Primitive* p, size_t a, size_t b, size_t c) {
    p->indices.push_back(static_cast<uint32_t>(a));
    p->indices.push_back(static_cast<uint32_t>(b));
    p->indices.push_back(static_cast<uint32_t>(c));
}

}  // namespace

Primitive MakeBox(float sx, float sy, float sz) {
    // 면마다 정점 4개 (법선이 면마다 달라야 하므로 공유하지 않는다). 반시계 방향이 바깥쪽.
    static const float kFaces[6][4][3] = {
        {{1, -1, -1}, {1, 1, -1}, {1, 1, 1}, {1, -1, 1}},        // +X
        {{-1, -1, 1}, {-1, 1, 1}, {-1, 1, -1}, {-1, -1, -1}},    // -X
        {{-1, 1, -1}, {-1, 1, 1}, {1, 1, 1}, {1, 1, -1}},        // +Y
        {{-1, -1, 1}, {-1, -1, -1}, {1, -1, -1}, {1, -1, 1}},    // -Y
        {{-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}},        // +Z
        {{-1, 1, -1}, {1, 1, -1}, {1, -1, -1}, {-1, -1, -1}},    // -Z
    };
    static const float kNormals[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    static const float kUv[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};

    const float hx = sx * 0.5f, hy = sy * 0.5f, hz = sz * 0.5f;
    Primitive p;
    for (int f = 0; f < 6; f++) {
        const size_t base = p.VertexCount();
        for (int k = 0; k < 4; k++) {
            AddVertex(&p, kFaces[f][k][0] * hx, kFaces[f][k][1] * hy, kFaces[f][k][2] * hz,
                      kNormals[f][0], kNormals[f][1], kNormals[f][2], kUv[k][0], kUv[k][1]);
        }
        AddTriangle(&p, base, base + 1, base + 2);
        AddTriangle(&p, base, base + 2, base + 3);
    }
    return p;
}

Primitive MakeCylinder(float radius, float length, int segments) {
    if (segments < 3) segments = 3;
    const size_t n = static_cast<size_t>(segments);
    const float h = length * 0.5f;
    Primitive p;

    // 옆면: 아래/위 고리를 번갈아 배치 (이음매 정점은 uv 때문에 한 번 더 만든다)
    for (size_t i = 0; i <= n; i++) {
        const float t = static_cast<float>(i) / static_cast<float>(n);
        const float c = simul::Cos(2.0f * kPi * t), s = simul::Sin(2.0f * kPi * t);
        AddVertex(&p, radius * c, radius * s, -h, c, s, 0, t, 0);
        AddVertex(&p, radius * c, radius * s, h, c, s, 0, t, 1);
    }
    for (size_t i = 0; i < n; i++) {
        const size_t b0 = 2 * i, t0 = 2 * i + 1, b1 = 2 * (i + 1), t1 = 2 * (i + 1) + 1;
        AddTriangle(&p, b0, b1, t1);
        AddTriangle(&p, b0, t1, t0);
    }

    // 윗면(+Z), 아랫면(-Z)
    for (int cap = 0; cap < 2; cap++) {
        const float z = cap == 0 ? h : -h;
        const float nz = cap == 0 ? 1.0f : -1.0f;
        const size_t center = p.VertexCount();
        AddVertex(&p, 0, 0, z, 0, 0, nz, 0.5f, 0.5f);
        for (size_t i = 0; i <= n; i++) {
            const float t = static_cast<float>(i) / static_cast<float>(n);
            const float c = simul::Cos(2.0f * kPi * t), s = simul::Sin(2.0f * kPi * t);
            AddVertex(&p, radius * c, radius * s, z, 0, 0, nz, 0.5f + 0.5f * c, 0.5f + 0.5f * s);
        }
        for (size_t i = 0; i < n; i++) {
            if (cap == 0) AddTriangle(&p, center, center + 1 + i, center + 2 + i);
            else AddTriangle(&p, center, center + 2 + i, center + 1 + i);
        }
    }
    return p;
}

Primitive MakeSphere(float radius, int segments, int rings) {
    if (segments < 3) segments = 3;
    if (rings < 2) rings = 2;
    const size_t ns = static_cast<size_t>(segments), nr = static_cast<size_t>(rings);
    Primitive p;
    for (size_t r = 0; r <= nr; r++) {
        const float v = static_cast<float>(r) / static_cast<float>(nr);
        const float phi = kPi * v;  // 0 = +Z 극
        for (size_t s = 0; s <= ns; s++) {
            const float u = static_cast<float>(s) / static_cast<float>(ns);
            const float theta = 2.0f * kPi * u;
            const float nx = simul::Sin(phi) * simul::Cos(theta);
            const float ny = simul::Sin(phi) * simul::Sin(theta);
            const float nz = simul::Cos(phi);
            AddVertex(&p, radius * nx, radius * ny, radius * nz, nx, ny, nz, u, v);
        }
    }
    for (size_t r = 0; r < nr; r++) {
        for (size_t s = 0; s < ns; s++) {
            const size_t v00 = r * (ns + 1) + s, v01 = v00 + 1, v10 = v00 + (ns + 1), v11 = v10 + 1;
            AddTriangle(&p, v00, v10, v01);
            AddTriangle(&p, v01, v10, v11);
        }
    }
    return p;
}

Scene SceneFromPrimitive(const Primitive& p, const simul::String& name) {
    Scene scene;
    scene.name = name;
    Mesh mesh;
    mesh.name = name;
    mesh.primitives.push_back(p);
    scene.meshes.push_back(mesh);
    SceneNode node;
    node.name = name;
    node.mesh = 0;
    scene.nodes.push_back(node);
    scene.roots.push_back(0);
    return scene;
}

}  // namespace simul
