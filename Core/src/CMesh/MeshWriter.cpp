#include "CMesh/MeshWriter.h"

#include <math.h>
#include <stdio.h>
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

namespace {

// 큰 메시를 위해 한 덩어리로 모아 쓰는 문자열 버퍼
struct Out {
    Array<char> buf;
    void Str(const char* s) { buf.insert(buf.end(), s, s + strlen(s)); }
    void Str(const String& s) { buf.insert(buf.end(), s.c_str(), s.c_str() + s.size()); }
    void Fmt(const char* f, double a, double b) {
        char t[128];
        const int n = snprintf(t, sizeof t, f, a, b);
        if (n > 0) buf.insert(buf.end(), t, t + n);
    }
    void Fmt(const char* f, double a) {
        char t[128];
        const int n = snprintf(t, sizeof t, f, a);
        if (n > 0) buf.insert(buf.end(), t, t + n);
    }
    void Fmt(const char* f, double a, double b, double c) {
        char t[128];
        const int n = snprintf(t, sizeof t, f, a, b, c);
        if (n > 0) buf.insert(buf.end(), t, t + n);
    }
};

String BaseOf(const String& p) {
    const size_t s = p.find_last_of("/\\");
    return s == String::npos ? p : p.substr(s + 1);
}

String StemOf(const String& p) {
    const String b = BaseOf(p);
    const size_t d = b.rfind('.');
    return d == String::npos || d == 0 ? b : b.substr(0, d);
}

String DirOf(const String& p) {
    const size_t s = p.find_last_of("/\\");
    return s == String::npos ? String() : p.substr(0, s + 1);
}

String ExtOf(const String& p) {
    const size_t d = p.rfind('.');
    const size_t s = p.find_last_of("/\\");
    if (d == String::npos || (s != String::npos && d < s)) return String();
    String e = p.substr(d + 1);
    for (size_t i = 0; i < e.size(); i++)
        if (e[i] >= 'A' && e[i] <= 'Z') e[i] = static_cast<char>(e[i] - 'A' + 'a');
    return e;
}

}  // namespace

bool SaveObj(const Scene& scene, const String& path, String* error, Array<String>* warnings) {
    const Array<dhMat> world = scene.WorldTransforms();
    const String stem = StemOf(path), dir = DirOf(path);
    const String mtlName = stem + ".mtl";
    Out obj, mtl;
    obj.Str("# Project Simul\nmtllib " + mtlName + "\n");
    mtl.Str("# Project Simul\n");
    Array<char> used(scene.materials.size(), 0);   // 쓴 재질
    size_t base = 0, tris = 0, uvBase = 0;
    bool anyColor = false;
    for (size_t mi = 0; mi < scene.meshes.size(); mi++)
        for (size_t pi = 0; pi < scene.meshes[mi].primitives.size(); pi++)
            if (!scene.meshes[mi].primitives[pi].colors.empty() && scene.meshes[mi].primitives[pi].colors.size() == scene.meshes[mi].primitives[pi].VertexCount() * 4) anyColor = true;
    for (size_t n = 0; n < scene.nodes.size(); n++) {
        const SceneNode& node = scene.nodes[n];
        if (node.mesh < 0 || static_cast<size_t>(node.mesh) >= scene.meshes.size()) continue;
        const dhMat& m = world[n];
        // 법선 변환: 역행렬의 전치 = 열의 외적 (비균등 크기에도 맞다). 행렬식이 음수(거울)이면 면 방향을 뒤집는다.
        const dhVector c0(m.v[0], m.v[1], m.v[2]), c1(m.v[4], m.v[5], m.v[6]), c2(m.v[8], m.v[9], m.v[10]);
        const dhVector n0 = c1 * c2, n1 = c2 * c0, n2 = c0 * c1;
        const float det = c0.x * n0.x + c0.y * n0.y + c0.z * n0.z;
        const bool flip = det < 0.0f;
        for (size_t p = 0; p < scene.meshes[static_cast<size_t>(node.mesh)].primitives.size(); p++) {
            Primitive pr = scene.meshes[static_cast<size_t>(node.mesh)].primitives[p];
            if (pr.TriangleCount() == 0) continue;
            if (pr.normals.size() != pr.positions.size()) {
                pr.normals.clear();
                pr.ComputeNormals();
            }
            const size_t nv = pr.VertexCount();
            const bool hasUv = pr.uvs.size() == nv * 2, hasCol = anyColor;   // 정점 색이 한 곳이라도 있으면 모든 v 에 쓴다 (로더는 일부만 있으면 버린다)
            const bool ownCol = pr.colors.size() == nv * 4;
            obj.Str("o " + (scene.meshes[static_cast<size_t>(node.mesh)].name.empty() ? String("mesh") : scene.meshes[static_cast<size_t>(node.mesh)].name) + "_" + ToString(p) + "\n");
            for (size_t i = 0; i < nv; i++) {
                const dhVector w = m * dhVector(pr.positions[i * 3], pr.positions[i * 3 + 1], pr.positions[i * 3 + 2]);
                char t[160];
                const int len = hasCol ? snprintf(t, sizeof t, "v %.7g %.7g %.7g %.4g %.4g %.4g\n", static_cast<double>(w.x), static_cast<double>(w.y), static_cast<double>(w.z),
                                                  ownCol ? static_cast<double>(pr.colors[i * 4]) : 1.0, ownCol ? static_cast<double>(pr.colors[i * 4 + 1]) : 1.0, ownCol ? static_cast<double>(pr.colors[i * 4 + 2]) : 1.0)
                                       : snprintf(t, sizeof t, "v %.7g %.7g %.7g\n", static_cast<double>(w.x), static_cast<double>(w.y), static_cast<double>(w.z));
                obj.buf.insert(obj.buf.end(), t, t + len);
            }
            if (hasUv)
                for (size_t i = 0; i < nv; i++) obj.Fmt("vt %.6g %.6g\n", static_cast<double>(pr.uvs[i * 2]), 1.0 - static_cast<double>(pr.uvs[i * 2 + 1]));
            for (size_t i = 0; i < nv; i++) {
                const float x = pr.normals[i * 3], y = pr.normals[i * 3 + 1], z = pr.normals[i * 3 + 2];
                dhVector nn(n0.x * x + n1.x * y + n2.x * z, n0.y * x + n1.y * y + n2.y * z, n0.z * x + n1.z * y + n2.z * z);
                const float len = sqrtf(nn.x * nn.x + nn.y * nn.y + nn.z * nn.z);
                if (len > 1e-20f) nn = nn / len;
                if (flip) nn = dhVector(-nn.x, -nn.y, -nn.z);
                obj.Fmt("vn %.5g %.5g %.5g\n", static_cast<double>(nn.x), static_cast<double>(nn.y), static_cast<double>(nn.z));
            }
            if (pr.material >= 0 && static_cast<size_t>(pr.material) < scene.materials.size()) {
                obj.Str("usemtl m" + ToString(pr.material) + "\n");
                used[static_cast<size_t>(pr.material)] = 1;
            } else {
                obj.Str("usemtl none\n");
            }
            for (size_t t = 0; t < pr.TriangleCount(); t++) {
                size_t id[3];
                for (int k = 0; k < 3; k++) id[k] = static_cast<size_t>(pr.indices[t * 3 + static_cast<size_t>(flip ? 2 - k : k)]);
                char line[160];
                int len;
                if (hasUv)
                    len = snprintf(line, sizeof line, "f %zu/%zu/%zu %zu/%zu/%zu %zu/%zu/%zu\n", base + id[0] + 1, uvBase + id[0] + 1, base + id[0] + 1, base + id[1] + 1, uvBase + id[1] + 1, base + id[1] + 1,
                                   base + id[2] + 1, uvBase + id[2] + 1, base + id[2] + 1);
                else
                    len = snprintf(line, sizeof line, "f %zu//%zu %zu//%zu %zu//%zu\n", base + id[0] + 1, base + id[0] + 1, base + id[1] + 1, base + id[1] + 1, base + id[2] + 1, base + id[2] + 1);
                obj.buf.insert(obj.buf.end(), line, line + len);
                tris++;
            }
            base += nv;
            if (hasUv) uvBase += nv;
        }
    }
    if (tris == 0) {
        if (error) *error = "OBJ 로 쓸 삼각형이 없습니다: " + path;
        return false;
    }
    // 재질 파일과 텍스처
    mtl.Str("newmtl none\nKd 0.8 0.8 0.8\n\n");
    for (size_t i = 0; i < scene.materials.size(); i++) {
        if (!used[i]) continue;
        const Material& mt = scene.materials[i];
        mtl.Str("newmtl m" + ToString(i) + "\n");
        mtl.Fmt("Kd %.5g %.5g %.5g\n", static_cast<double>(mt.baseColor.r), static_cast<double>(mt.baseColor.g), static_cast<double>(mt.baseColor.b));
        if (mt.alphaMode == 2 && mt.baseColor.a < 1.0f) mtl.Fmt("d %.4g\n", static_cast<double>(mt.baseColor.a));
        String texFile;
        if (!mt.baseColorImage.empty()) {
            const unsigned char* d = &mt.baseColorImage[0];
            const size_t sz = mt.baseColorImage.size();
            const char* ext = sz > 8 && d[0] == 0x89 && d[1] == 'P' && d[2] == 'N' && d[3] == 'G' ? "png" : sz > 3 && d[0] == 0xFF && d[1] == 0xD8 ? "jpg" : NULL;
            if (!ext) {
                if (warnings) warnings->push_back("알 수 없는 이미지 형식이라 텍스처를 쓰지 못했습니다 (재질 " + ToString(i) + ")");
            } else {
                texFile = stem + "_tex" + ToString(i) + "." + ext;
                if (!WriteFileBytes(dir + texFile, d, sz)) {
                    if (error) *error = "파일을 쓸 수 없습니다: " + dir + texFile;
                    return false;
                }
            }
        } else if (!mt.baseColorTexture.empty()) {
            Array<unsigned char> bytes;
            String ext = ExtOf(mt.baseColorTexture);
            if (ext.empty()) ext = "png";
            if (!ReadAllBytes(mt.baseColorTexture, &bytes, 0) || bytes.empty()) {
                if (warnings) warnings->push_back("텍스처 파일을 읽지 못했습니다: " + mt.baseColorTexture);
            } else {
                texFile = stem + "_tex" + ToString(i) + "." + ext;
                if (!WriteFileBytes(dir + texFile, &bytes[0], bytes.size())) {
                    if (error) *error = "파일을 쓸 수 없습니다: " + dir + texFile;
                    return false;
                }
            }
        }
        if (!texFile.empty()) mtl.Str("map_Kd " + texFile + "\n");
        mtl.Str("\n");
    }
    if (!WriteFileBytes(path, &obj.buf[0], obj.buf.size()) || !WriteFileBytes(dir + mtlName, &mtl.buf[0], mtl.buf.size())) {
        if (error) *error = "파일을 쓸 수 없습니다: " + path;
        return false;
    }
    return true;
}

}  // namespace simul
