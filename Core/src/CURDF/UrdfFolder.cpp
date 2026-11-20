// 폴더로 저장: <dir>/<이름>.urdf + <dir>/obj/ (메시)
#include <stdio.h>
#include <string.h>

#include "CBase/File.h"
#include "CBase/Map.h"
#include "CMesh/MeshLoader.h"
#include "CMesh/MeshWriter.h"
#include "CURDF/UrdfWriter.h"
#include "FileUtil.h"

namespace simul {
namespace urdf {

namespace {

simul::String Slashes(simul::String p) {
    for (size_t i = 0; i < p.size(); i++)
        if (p[i] == '\\') p[i] = '/';
    return p;
}

simul::String BaseName(const simul::String& p) {
    const size_t s = p.find_last_of("/\\");
    return s == simul::String::npos ? p : p.substr(s + 1);
}

simul::String Stem(const simul::String& file) {
    const simul::String b = BaseName(file);
    const size_t d = b.find_last_of('.');
    return d == simul::String::npos || d == 0 ? b : b.substr(0, d);
}

// 파일 이름에 쓸 수 있는 글자만 남긴다 (공백, 한글 등은 그대로 두되 경로 구분자와 Windows 금지 문자는 '_')
simul::String Safe(const simul::String& s) {
    simul::String o;
    for (size_t i = 0; i < s.size(); i++) {
        const char c = s[i];
        if (c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' || c == '"' || c == '<' || c == '>' || c == '|') o += '_';
        else o += c;
    }
    return o.empty() ? simul::String("mesh") : o;
}

bool HasScheme(const simul::String& p) { return p.find("://") != simul::String::npos; }

bool IsAbs(const simul::String& p) {
    if (p.empty()) return false;
    if (p[0] == '/' || p[0] == '\\') return true;
    return p.size() > 2 && p[1] == ':';
}

simul::String Lower(simul::String s) {
    for (size_t i = 0; i < s.size(); i++)
        if (s[i] >= 'A' && s[i] <= 'Z') s[i] = static_cast<char>(s[i] - 'A' + 'a');
    return s;
}

bool CopyFileBytes(const simul::String& from, const simul::String& to, simul::String* err) {
    Array<unsigned char> bytes;
    if (!ReadAllBytes(from, &bytes, err)) return false;
    if (!WriteFileBytes(to, bytes.empty() ? static_cast<const void*>("") : static_cast<const void*>(&bytes[0]), bytes.size())) {
        if (err) *err = "파일을 쓸 수 없습니다: " + to;
        return false;
    }
    return true;
}

// 상대 경로 file 을 srcDir 에서 dstDir 로 (같은 하위 경로로) 복사한다. 절대 경로, "..", 접두사(scheme)는 건너뛰고 false.
bool CopyCompanion(const simul::String& srcDir, const simul::String& dstDir, const simul::String& fileIn) {
    simul::String file = Slashes(fileIn);
    if (file.empty() || IsAbs(file) || HasScheme(file) || file.find("..") != simul::String::npos) return false;
    const simul::String from = detail::JoinPath(srcDir, file);
    const simul::String to = detail::JoinPath(dstDir, file);
    const simul::String sub = detail::DirName(to);
    if (!sub.empty() && !MakeDirectories(sub)) return false;
    simul::String err;
    return CopyFileBytes(from, to, &err);
}

bool IsImageExt(const simul::String& file) {
    const simul::String e = detail::LowerExtension(file);
    return e == "png" || e == "jpg" || e == "jpeg" || e == "tga" || e == "bmp" || e == "gif" || e == "tif" || e == "tiff";
}

simul::String Trim(const simul::String& s) {
    size_t a = 0, b = s.size();
    while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n')) a++;
    while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n')) b--;
    return s.substr(a, b - a);
}

// OBJ 의 mtllib 과 그 mtl 안의 map_* 텍스처를 복사한다. 복사하지 못한 것은 warnings 에 쓴다.
void CopyObjCompanions(const simul::String& objPath, const simul::String& dstDir, simul::Array<simul::String>* warnings) {
    simul::Array<unsigned char> bytes;
    if (!ReadAllBytes(objPath, &bytes, 0) || bytes.empty()) return;
    const simul::String text(reinterpret_cast<const char*>(&bytes[0]), bytes.size());
    const simul::String srcDir = detail::DirName(objPath);
    size_t pos = 0;
    while (pos < text.size()) {
        size_t end = text.find('\n', pos);
        if (end == simul::String::npos) end = text.size();
        const simul::String line = Trim(text.substr(pos, end - pos));
        pos = end + 1;
        if (line.compare(0, 7, "mtllib ") != 0) continue;
        const simul::String mtl = Trim(line.substr(7));
        if (!CopyCompanion(srcDir, dstDir, mtl)) {
            warnings->push_back("OBJ 의 재질 파일을 복사하지 못했습니다: " + mtl + " (" + BaseName(objPath) + ")");
            continue;
        }
        simul::Array<unsigned char> mb;
        if (!ReadAllBytes(detail::JoinPath(srcDir, mtl), &mb, 0) || mb.empty()) continue;
        const simul::String mt(reinterpret_cast<const char*>(&mb[0]), mb.size());
        const simul::String mtlDir = detail::DirName(detail::JoinPath(srcDir, mtl));
        const simul::String mtlDst = detail::DirName(detail::JoinPath(dstDir, mtl));
        size_t p2 = 0;
        while (p2 < mt.size()) {
            size_t e2 = mt.find('\n', p2);
            if (e2 == simul::String::npos) e2 = mt.size();
            const simul::String l2 = Trim(mt.substr(p2, e2 - p2));
            p2 = e2 + 1;
            if (l2.compare(0, 4, "map_") != 0 && l2.compare(0, 4, "bump") != 0) continue;
            const size_t sp = l2.find_last_of(' ');   // 옵션 뒤의 마지막 토큰이 파일 이름이다
            const simul::String tex = Trim(sp == simul::String::npos ? l2 : l2.substr(sp + 1));
            if (tex.empty() || !IsImageExt(tex)) continue;
            if (!CopyCompanion(mtlDir, mtlDst, tex)) warnings->push_back("OBJ 재질의 텍스처를 복사하지 못했습니다: " + tex);
        }
    }
}

// DAE 의 <init_from> 이미지를 복사한다
void CopyDaeCompanions(const simul::String& daePath, const simul::String& dstDir, simul::Array<simul::String>* warnings) {
    simul::Array<unsigned char> bytes;
    if (!ReadAllBytes(daePath, &bytes, 0) || bytes.empty()) return;
    const simul::String text(reinterpret_cast<const char*>(&bytes[0]), bytes.size());
    const simul::String srcDir = detail::DirName(daePath);
    size_t pos = 0;
    while ((pos = text.find("<init_from>", pos)) != simul::String::npos) {
        pos += 11;
        const size_t end = text.find('<', pos);
        if (end == simul::String::npos) break;
        simul::String f = Trim(text.substr(pos, end - pos));
        pos = end;
        if (f.compare(0, 7, "file://") == 0) f = f.substr(7);
        if (f.empty() || !IsImageExt(f)) continue;
        if (!CopyCompanion(srcDir, dstDir, f)) warnings->push_back("DAE 의 텍스처를 복사하지 못했습니다: " + f + " (" + BaseName(daePath) + ")");
    }
}

struct Exporter {
    const Robot& robot;
    const LoadOptions& load;
    simul::String dir;      // 저장 폴더 (슬래시, 끝 슬래시 없음)
    simul::String objDir;   // dir/obj
    FolderSaveResult* res;
    bool toStl;
    simul::Map<simul::String, simul::String> nameOfKey;   // 원본 키 -> obj/ 아래 상대 경로 ("a.stl", "dir/x.obj")
    simul::Map<simul::String, int> used;                   // 쓴 이름(소문자) -> 개수
    simul::String error;

    Exporter(const Robot& r, const LoadOptions& l, FolderSaveResult* result, bool stl) : robot(r), load(l), res(result), toStl(stl) {}

    void Warn(const simul::String& w) { res->warnings.push_back(w); }

    // 겹치지 않는 이름 (확장자 포함). 같은 이름이 있으면 stem_2.ext 로 한다.
    simul::String Unique(const simul::String& stem, const simul::String& ext) {
        simul::String name = Safe(stem);
        int n = 1;
        for (;;) {
            const simul::String cand = n == 1 ? name + ext : name + "_" + simul::ToString(n) + ext;
            const simul::String key = Lower(cand);
            if (!used.count(key)) {
                used[key] = 1;
                return cand;
            }
            n++;
        }
    }

    bool EnsureObjDir() { return MakeDirectories(objDir); }

    // 메시 geometry 하나를 폴더 안으로 옮기고 g 를 새 경로로 고친다. color 가 NULL 이 아니면 GLB 의 기본 색을 알려 준다 (STL 로 바꿀 때).
    bool Process(Geometry* g, bool wantColor, bool* gotColor, Color* color, const simul::String& where) {
        if (g->type != kGeomMesh || g->filename.empty()) return true;
        simul::String src;
        if (!ResolveMeshPath(g->filename, robot.baseDir, load, &src)) {
            Warn(where + ": 메시 파일을 찾을 수 없어 경로를 그대로 둡니다 (" + g->filename + ")");
            if (!IsAbs(g->filename) && !HasScheme(g->filename) && !robot.baseDir.empty()) g->filename = detail::JoinPath(Slashes(robot.baseDir), Slashes(g->filename));
            return true;
        }
        src = Slashes(src);
        const simul::String ext = detail::LowerExtension(src);
        if (ext == "glb" || ext == "gltf") return ConvertGlb(g, src, wantColor, gotColor, color, where);
        if (ext == "stl" || ext == "obj" || ext == "dae") return CopyMesh(g, src, ext, where);
        Warn(where + ": 지원하지 않는 형식이라 경로를 그대로 둡니다 (" + src + ")");
        g->filename = src;
        return true;
    }

    bool ConvertGlb(Geometry* g, const simul::String& src, bool wantColor, bool* gotColor, Color* color, const simul::String& where) {
        Scene sc;
        simul::String err;
        if (!LoadGeometryScene(*g, robot.baseDir, load, &sc, &err)) {   // scale 과 Y-up 회전이 적용된 모양 = 화면에 보이는 모양
            Warn(where + ": " + err);
            g->filename = src;
            return true;
        }
        const simul::String key = (toStl ? "glb-stl|" : "glb-obj|") + src + "|" + simul::ToString(g->scale[0]) + "," + simul::ToString(g->scale[1]) + "," + simul::ToString(g->scale[2]);
        simul::Map<simul::String, simul::String>::iterator it = nameOfKey.find(key);
        simul::String rel;
        if (it != nameOfKey.end()) {
            rel = it->second;
        } else {
            if (!EnsureObjDir()) {
                error = "폴더를 만들 수 없습니다: " + objDir;
                return false;
            }
            if (toStl) {
                rel = Unique(Stem(src), ".stl");
                if (!SaveStl(sc, detail::JoinPath(objDir, rel), &err)) {
                    Warn(where + ": " + err);
                    g->filename = src;
                    return true;
                }
            } else {   // OBJ + MTL + 텍스처는 한 폴더에 모은다
                const simul::String sub = Unique(Stem(src), "");
                if (!MakeDirectories(detail::JoinPath(objDir, sub))) {
                    error = "폴더를 만들 수 없습니다: " + detail::JoinPath(objDir, sub);
                    return false;
                }
                rel = sub + "/" + sub + ".obj";
                simul::Array<simul::String> w;
                if (!SaveObj(sc, detail::JoinPath(objDir, rel), &err, &w)) {
                    Warn(where + ": " + err);
                    g->filename = src;
                    return true;
                }
                for (size_t i = 0; i < w.size(); i++) Warn(where + ": " + w[i]);
            }
            nameOfKey[key] = rel;
            res->converted++;
        }
        g->filename = "obj/" + rel;
        g->scale[0] = g->scale[1] = g->scale[2] = 1.0f;
        if (!toStl) return true;   // OBJ 는 색과 텍스처를 파일이 가진다
        // STL 에 담지 못하는 정보
        bool textured = false, vertexColors = false;
        for (size_t m = 0; m < sc.materials.size(); m++)
            if (!sc.materials[m].baseColorTexture.empty() || !sc.materials[m].baseColorImage.empty()) textured = true;
        for (size_t m = 0; m < sc.meshes.size(); m++)
            for (size_t p = 0; p < sc.meshes[m].primitives.size(); p++)
                if (!sc.meshes[m].primitives[p].colors.empty()) vertexColors = true;
        if (textured) Warn(where + ": STL 에는 텍스처를 담을 수 없어 빠집니다 (" + BaseName(src) + ")");
        if (vertexColors) Warn(where + ": STL 에는 정점 색을 담을 수 없어 빠집니다 (" + BaseName(src) + ")");
        if (wantColor && !sc.materials.empty()) {   // 기본 색 하나는 URDF 의 <material> 로 옮긴다
            int first = -1;
            bool several = false;
            for (size_t m = 0; m < sc.meshes.size(); m++)
                for (size_t p = 0; p < sc.meshes[m].primitives.size(); p++) {
                    const int mi = sc.meshes[m].primitives[p].material;
                    if (mi < 0 || static_cast<size_t>(mi) >= sc.materials.size()) continue;
                    if (first < 0) first = mi;
                    else if (memcmp(&sc.materials[static_cast<size_t>(mi)].baseColor, &sc.materials[static_cast<size_t>(first)].baseColor, sizeof(Color)) != 0) several = true;
                }
            if (first >= 0) {
                *gotColor = true;
                *color = sc.materials[static_cast<size_t>(first)].baseColor;
                if (several) Warn(where + ": 재질이 여러 색이라 첫 번째 색 하나만 남깁니다 (" + BaseName(src) + ")");
            }
        }
        return true;
    }

    bool CopyMesh(Geometry* g, const simul::String& src, const simul::String& ext, const simul::String& where) {
        const simul::String key = "copy|" + src;
        simul::Map<simul::String, simul::String>::iterator it = nameOfKey.find(key);
        if (it != nameOfKey.end()) {
            g->filename = "obj/" + it->second;
            return true;
        }
        if (!EnsureObjDir()) {
            error = "폴더를 만들 수 없습니다: " + objDir;
            return false;
        }
        simul::String rel, err;
        if (ext == "stl") {
            rel = Unique(Stem(src), ".stl");
            if (!CopyFileBytes(src, detail::JoinPath(objDir, rel), &err)) {
                Warn(where + ": " + err);
                g->filename = src;
                return true;
            }
        } else {   // obj, dae: 부속 파일(재질, 텍스처)이 상대 경로로 붙어 있으므로 폴더 하나에 모은다
            const simul::String sub = Unique(Stem(src), "");
            const simul::String subDir = detail::JoinPath(objDir, sub);
            if (!MakeDirectories(subDir) || !CopyFileBytes(src, detail::JoinPath(subDir, BaseName(src)), &err)) {
                Warn(where + ": " + (err.empty() ? simul::String("폴더를 만들 수 없습니다") : err));
                g->filename = src;
                return true;
            }
            rel = sub + "/" + BaseName(src);
            if (ext == "obj") CopyObjCompanions(src, subDir, &res->warnings);
            else CopyDaeCompanions(src, subDir, &res->warnings);
        }
        nameOfKey[key] = rel;
        res->copied++;
        g->filename = "obj/" + rel;
        return true;
    }

    // <material><texture> 이미지
    void Texture(MaterialDef* m, const simul::String& where) {
        if (m->texture.empty()) return;
        simul::String src;
        if (!ResolveMeshPath(m->texture, robot.baseDir, load, &src)) {
            Warn(where + ": 텍스처 파일을 찾을 수 없어 경로를 그대로 둡니다 (" + m->texture + ")");
            return;
        }
        src = Slashes(src);
        const simul::String key = "tex|" + src;
        simul::Map<simul::String, simul::String>::iterator it = nameOfKey.find(key);
        if (it != nameOfKey.end()) {
            m->texture = "obj/" + it->second;
            return;
        }
        if (!EnsureObjDir()) return;
        const simul::String rel = Unique(Stem(src), simul::String(".") + detail::LowerExtension(src));
        simul::String err;
        if (!CopyFileBytes(src, detail::JoinPath(objDir, rel), &err)) {
            Warn(where + ": " + err);
            return;
        }
        nameOfKey[key] = rel;
        m->texture = "obj/" + rel;
    }
};

}  // namespace

bool SaveUrdfFolder(const Robot& robot, const simul::String& dirIn, const LoadOptions& load, simul::String* error,
                    FolderSaveResult* result, const WriteOptions& opt, const FolderOptions& folder) {
    FolderSaveResult local;
    FolderSaveResult* res = result ? result : &local;
    res->warnings.clear();
    res->converted = res->copied = 0;
    simul::String dir = Slashes(dirIn);
    while (dir.size() > 1 && dir[dir.size() - 1] == '/') dir.erase(dir.size() - 1);
    const simul::String name = BaseName(dir);
    if (dir.empty() || name.empty() || name == "." || name == "..") {
        if (error) *error = "저장할 폴더 이름이 올바르지 않습니다: " + dirIn;
        return false;
    }
    if (!MakeDirectories(dir)) {
        if (error) *error = "폴더를 만들 수 없습니다: " + dir;
        return false;
    }

    Robot r = robot;
    Exporter ex(robot, load, res, folder.glbToStl);
    ex.dir = dir;
    ex.objDir = dir + "/obj";
    for (size_t i = 0; i < r.links.size(); i++) {
        Link& l = r.links[i];
        for (size_t v = 0; v < l.visuals.size(); v++) {
            Visual& vis = l.visuals[v];
            const simul::String where = "link '" + l.name + "' visual " + simul::ToString(v);
            bool got = false;
            Color c;
            const bool wantColor = !(vis.hasMaterial && vis.material.hasColor);
            if (!ex.Process(&vis.geometry, wantColor, &got, &c, where)) {
                if (error) *error = ex.error;
                return false;
            }
            if (got) {
                vis.hasMaterial = true;
                vis.material.hasColor = true;
                vis.material.color = c;
            }
            if (vis.hasMaterial) ex.Texture(&vis.material, where);
        }
        for (size_t k = 0; k < l.collisions.size(); k++) {
            bool got = false;
            Color c;
            if (!ex.Process(&l.collisions[k].geometry, false, &got, &c, "link '" + l.name + "' collision " + simul::ToString(k))) {
                if (error) *error = ex.error;
                return false;
            }
        }
    }
    r.baseDir = dir;   // 이제 모든 메시 경로는 "obj/..." (dir 기준) 이거나 절대 경로다

    WriteOptions wo = opt;
    if (wo.robotName.empty()) wo.robotName = name;
    res->urdfPath = dir + "/" + name + ".urdf";
    const simul::String text = UrdfToString(r, dir, wo);
    if (!WriteFileText(res->urdfPath, text)) {
        if (error) *error = "파일을 쓸 수 없습니다: " + res->urdfPath;
        return false;
    }
    return true;
}

}  // namespace urdf
}  // namespace simul
