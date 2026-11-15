// Wavefront OBJ (+MTL) 로더. 다각형은 팬으로 삼각형화하고, 재질별로 primitive 를 나눈다.
#include <stdlib.h>

#include "CBase/HashMap.h"
#include "CBase/Map.h"
#include "CBase/Text.h"
#include "CMesh/MeshLoader.h"
#include "FileUtil.h"
#include "VertexKey.h"

namespace amr {

namespace {

typedef detail::VertexKey Key;  // (위치, uv, 법선) 인덱스, 없으면 -1

struct Group {
    Primitive prim;
    amr::HashMap<Key, uint32_t> dedup;
};

// "3", "3/1", "3//2", "3/1/2" -> 0 기반 인덱스 (음수는 현재 개수 기준 상대값). 실패 시 false.
bool ParseFaceVertex(const amr::String& tok, size_t nv, size_t nt, size_t nn, Key* key) {
    int vals[3] = {0, 0, 0};
    bool has[3] = {false, false, false};
    size_t part = 0, start = 0;
    for (size_t i = 0; i <= tok.size() && part < 3; i++) {
        if (i == tok.size() || tok[i] == '/') {
            const amr::String s = tok.substr(start, i - start);
            if (!s.empty()) {
                char* end = NULL;
                const long long v = strtoll(s.c_str(), &end, 10);
                if (*end != '\0' || v > 0x7FFFFFFFLL || v < -0x7FFFFFFFLL) return false;
                vals[part] = static_cast<int>(v);
                has[part] = true;
            }
            part++;
            start = i + 1;
        }
    }
    const size_t counts[3] = {nv, nt, nn};
    for (int k = 0; k < 3; k++) {
        if (!has[k]) {
            if (k == 0) return false;
            (*key)[k] = -1;
            continue;
        }
        long long idx = vals[k];
        if (idx < 0) idx = static_cast<long long>(counts[k]) + idx;  // 상대 인덱스
        else idx = idx - 1;
        if (idx < 0 || static_cast<size_t>(idx) >= counts[k]) return false;
        (*key)[k] = static_cast<int>(idx);
    }
    return true;
}

void ParseMtl(const amr::String& text, Scene* scene, amr::Map<amr::String, int>* matIndex) {
    Lines in(text);
    amr::String line;
    Material* cur = NULL;
    while (in.Next(&line)) {
        Tokens ls(line);
        amr::String key;
        if (!ls.Next(&key)) continue;
        if (key == "newmtl") {
            const amr::String name = ls.Rest();
            amr::Map<amr::String, int>::iterator it = matIndex->find(name);
            cur = it != matIndex->end() ? &scene->materials[static_cast<size_t>(it->second)] : NULL;
        } else if (!cur) {
            continue;
        } else if (key == "Kd") {
            float r, g, b;
            if (ls.NextFloat(&r) && ls.NextFloat(&g) && ls.NextFloat(&b)) {
                cur->baseColor.r = r;
                cur->baseColor.g = g;
                cur->baseColor.b = b;
            }
        } else if (key == "d") {
            float d;
            if (ls.NextFloat(&d)) cur->baseColor.a = d;
        } else if (key == "Tr") {
            float t;
            if (ls.NextFloat(&t)) cur->baseColor.a = 1.0f - t;
        } else if (key == "map_Kd") {
            cur->baseColorTexture = ls.Rest();
        }
    }
}

bool ParseObj(const amr::String& text, const amr::String& baseDir, bool loadMtl, Scene* out, amr::String* error) {
    amr::Array<float> pos, uv, nor;
    amr::Array<Group> groups;
    amr::Array<amr::String> groupMat;           // groups 와 같은 순서의 재질 이름 ("" = 없음)
    amr::Map<amr::String, size_t> groupByMat;
    amr::Array<amr::String> mtllibs;
    amr::String objName;

    const amr::String noMat;
    groupByMat[noMat] = static_cast<size_t>(-1);  // 지연 생성 표시
    size_t current = static_cast<size_t>(-1);
    amr::String currentMat;

    Lines in(text);
    amr::String line;
    size_t lineNo = 0;
    size_t faceNo = 0;
    while (in.Next(&line)) {
        lineNo++;
        if (line.empty() || line[0] == '#') continue;
        Tokens ls(line);
        amr::String key;
        if (!ls.Next(&key)) continue;
        if (key == "v") {
            float x, y, z;
            if (!(ls.NextFloat(&x) && ls.NextFloat(&y) && ls.NextFloat(&z))) {
                detail::SetError(error, "OBJ " + amr::ToString(lineNo) + "번째 줄: v 좌표가 올바르지 않습니다");
                return false;
            }
            pos.push_back(x); pos.push_back(y); pos.push_back(z);
        } else if (key == "vt") {
            float u = 0, v = 0;
            ls.NextFloat(&u) && ls.NextFloat(&v);
            uv.push_back(u); uv.push_back(v);
        } else if (key == "vn") {
            float x = 0, y = 0, z = 0;
            ls.NextFloat(&x) && ls.NextFloat(&y) && ls.NextFloat(&z);
            nor.push_back(x); nor.push_back(y); nor.push_back(z);
        } else if (key == "o" || key == "g") {
            if (objName.empty()) {
                objName = ls.Rest();
            }
        } else if (key == "mtllib") {
            const amr::String rest = ls.Rest();
            if (!rest.empty()) mtllibs.push_back(rest);
        } else if (key == "usemtl") {
            currentMat = ls.Rest();
            current = static_cast<size_t>(-1);
        } else if (key == "f") {
            amr::Array<Key> verts;
            amr::String tok;
            while (ls.Next(&tok)) {
                Key k;
                if (!ParseFaceVertex(tok, pos.size() / 3, uv.size() / 2, nor.size() / 3, &k)) {
                    detail::SetError(error, "OBJ " + amr::ToString(lineNo) + "번째 줄: 면의 인덱스가 올바르지 않습니다 (" + tok + ")");
                    return false;
                }
                verts.push_back(k);
            }
            if (verts.size() < 3) continue;  // 점/선 취급, 무시
            // 법선이 없는 정점은 면마다 따로 만들어(공유하지 않아) 평평한(flat) 음영이 되게 한다.
            // 위치만 공유하면 모서리가 뭉개져 상자 같은 모델이 둥글게 보인다.
            faceNo++;
            for (size_t i = 0; i < verts.size(); i++) if (verts[i][2] < 0) verts[i][2] = -static_cast<int>(faceNo % 1000000000u) - 2;
            if (current == static_cast<size_t>(-1)) {
                amr::Map<amr::String, size_t>::iterator it = groupByMat.find(currentMat);
                if (it == groupByMat.end() || it->second == static_cast<size_t>(-1)) {
                    groups.push_back(Group());
                    groupMat.push_back(currentMat);
                    groupByMat[currentMat] = groups.size() - 1;
                    current = groups.size() - 1;
                } else {
                    current = it->second;
                }
            }
            Group& g = groups[current];
            amr::Array<uint32_t> ids;
            for (size_t i = 0; i < verts.size(); i++) {
                const Key& k = verts[i];
                const uint32_t* found = g.dedup.Find(k);
                if (found) {
                    ids.push_back(*found);
                    continue;
                }
                const uint32_t id = static_cast<uint32_t>(g.prim.VertexCount());
                for (int c = 0; c < 3; c++) g.prim.positions.push_back(pos[static_cast<size_t>(k[0]) * 3 + static_cast<size_t>(c)]);
                g.dedup.Insert(k, id);
                ids.push_back(id);
                // 법선/uv 는 모든 정점이 가진 경우에만 유지한다 (아래에서 길이로 검증)
                if (k[2] >= 0) for (int c = 0; c < 3; c++) g.prim.normals.push_back(nor[static_cast<size_t>(k[2]) * 3 + static_cast<size_t>(c)]);
                if (k[1] >= 0) for (int c = 0; c < 2; c++) g.prim.uvs.push_back(uv[static_cast<size_t>(k[1]) * 2 + static_cast<size_t>(c)]);
            }
            for (size_t i = 1; i + 1 < ids.size(); i++) {
                g.prim.indices.push_back(ids[0]);
                g.prim.indices.push_back(ids[i]);
                g.prim.indices.push_back(ids[i + 1]);
            }
        }
    }

    if (groups.empty()) {
        detail::SetError(error, "OBJ 에 면(f)이 없습니다");
        return false;
    }

    out->Clear();
    amr::Map<amr::String, int> matIndex;
    Mesh mesh;
    mesh.name = objName.empty() ? "obj" : objName;
    for (size_t gi = 0; gi < groups.size(); gi++) {
        Primitive& p = groups[gi].prim;
        // 일부 정점에만 법선/uv 가 있으면 일관성이 없으므로 버린다
        if (p.normals.size() != p.positions.size()) {
            if (!p.normals.empty()) out->warnings.push_back("일부 정점에 법선이 없어 법선을 새로 계산했습니다");
            p.normals.clear();
            p.ComputeNormals();
        }
        if (p.uvs.size() != p.VertexCount() * 2) p.uvs.clear();
        if (!groupMat[gi].empty()) {
            amr::Map<amr::String, int>::iterator it = matIndex.find(groupMat[gi]);
            if (it == matIndex.end()) {
                Material m;
                m.name = groupMat[gi];
                out->materials.push_back(m);
                it = matIndex.insert(amr::MakePair(groupMat[gi], static_cast<int>(out->materials.size() - 1))).first;
            }
            p.material = it->second;
        }
        mesh.primitives.push_back(p);
    }
    out->meshes.push_back(mesh);
    out->name = mesh.name;
    SceneNode node;
    node.name = mesh.name;
    node.mesh = 0;
    out->nodes.push_back(node);
    out->roots.push_back(0);

    if (!mtllibs.empty()) {
        if (!loadMtl) {
            out->warnings.push_back("mtllib 은 파일 경로로 읽을 때만 적용됩니다");
        } else {
            for (size_t i = 0; i < mtllibs.size(); i++) {
                amr::Array<char> bytes;
                amr::String ignored;
                if (!detail::ReadFileBytes(detail::JoinPath(baseDir, mtllibs[i]), &bytes, &ignored)) {
                    out->warnings.push_back("MTL 파일을 읽을 수 없습니다: " + mtllibs[i]);
                    continue;
                }
                ParseMtl(amr::String(bytes.begin(), bytes.end()), out, &matIndex);
            }
        }
    }
    if (!out->IsValid()) {
        out->Clear();
        detail::SetError(error, "변환된 장면의 구조가 올바르지 않습니다");
        return false;
    }
    return true;
}

}  // namespace

bool LoadObjFromString(const amr::String& text, Scene* out, amr::String* error) {
    if (!out) return false;
    out->Clear();
    const bool ok = ParseObj(text, amr::String(), false, out, error);
    if (!ok) out->Clear();
    return ok;
}

bool LoadObj(const amr::String& path, Scene* out, amr::String* error) {
    if (!out) return false;
    out->Clear();
    amr::Array<char> bytes;
    if (!detail::ReadFileBytes(path, &bytes, error)) return false;
    const bool ok = ParseObj(amr::String(bytes.begin(), bytes.end()), detail::DirName(path), true, out, error);
    if (!ok) out->Clear();
    return ok;
}

}  // namespace amr
