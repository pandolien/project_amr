#include "CMesh/MeshWriter.h"

#include <math.h>
#include <string.h>

#include "CBase/File.h"

namespace simul {

namespace {

void PutF(unsigned char* dst, float f) { memcpy(dst, &f, 4); }   // 리틀 엔디언 (x86/ARM 대상, 로더와 같다)

}  // namespace

bool StlBytes(const Scene& scene, Array<unsigned char>* out) {
    out->clear();
    const Array<dhMat> world = scene.WorldTransforms();
    uint32_t count = 0;
    for (size_t n = 0; n < scene.nodes.size(); n++) {
        const SceneNode& node = scene.nodes[n];
        if (node.mesh < 0 || static_cast<size_t>(node.mesh) >= scene.meshes.size()) continue;
        const Mesh& m = scene.meshes[static_cast<size_t>(node.mesh)];
        for (size_t p = 0; p < m.primitives.size(); p++) count += static_cast<uint32_t>(m.primitives[p].TriangleCount());
    }
    if (count == 0) return false;
    out->resize(84 + static_cast<size_t>(count) * 50, 0);
    unsigned char* d = &(*out)[0];
    const char* head = "Simul binary STL";
    memcpy(d, head, strlen(head));
    memcpy(d + 80, &count, 4);

    unsigned char* rec = d + 84;
    for (size_t n = 0; n < scene.nodes.size(); n++) {
        const SceneNode& node = scene.nodes[n];
        if (node.mesh < 0 || static_cast<size_t>(node.mesh) >= scene.meshes.size()) continue;
        const Mesh& m = scene.meshes[static_cast<size_t>(node.mesh)];
        for (size_t p = 0; p < m.primitives.size(); p++) {
            const Primitive& pr = m.primitives[p];
            for (size_t t = 0; t < pr.TriangleCount(); t++) {
                dhVector v[3];
                for (int k = 0; k < 3; k++) {
                    const size_t i = pr.indices[t * 3 + static_cast<size_t>(k)] * 3;
                    v[k] = world[n] * dhVector(pr.positions[i], pr.positions[i + 1], pr.positions[i + 2]);
                }
                dhVector nrm = (v[1] - v[0]) * (v[2] - v[0]);
                const float len = sqrtf(nrm.x * nrm.x + nrm.y * nrm.y + nrm.z * nrm.z);
                if (len > 1e-20f) nrm = nrm / len;
                else nrm = dhVector(0, 0, 0);
                PutF(rec, nrm.x); PutF(rec + 4, nrm.y); PutF(rec + 8, nrm.z);
                for (int k = 0; k < 3; k++) {
                    PutF(rec + 12 + 12 * k, v[k].x);
                    PutF(rec + 16 + 12 * k, v[k].y);
                    PutF(rec + 20 + 12 * k, v[k].z);
                }
                rec += 50;
            }
        }
    }
    return true;
}

bool SaveStl(const Scene& scene, const String& path, String* error) {
    Array<unsigned char> bytes;
    if (!StlBytes(scene, &bytes)) {
        if (error) *error = "STL 로 쓸 삼각형이 없습니다: " + path;
        return false;
    }
    if (!WriteFileBytes(path, &bytes[0], bytes.size())) {
        if (error) *error = "파일을 쓸 수 없습니다: " + path;
        return false;
    }
    return true;
}

}  // namespace simul
