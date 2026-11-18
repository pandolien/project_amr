// URDF 의 메시 경로 해석과 visual 로딩
#include <stdio.h>
#include "CBase/Map.h"

#include "3d/dhMat.h"
#include "CMesh/MeshLoader.h"
#include "CMesh/Primitives.h"
#include "CURDF/Urdf.h"
#include "FileUtil.h"

namespace simul {
namespace urdf {

namespace {

bool FileExists(const simul::String& p) {
    FILE* f = fopen(p.c_str(), "rb");
    if (!f) return false;
    fclose(f);
    return true;
}

simul::String ParentDir(const simul::String& dir) {
    if (dir.empty()) return "..";
    const simul::String d = detail::DirName(dir);
    if (!d.empty() && d != dir) return d;
    // 마지막 요소가 ".." 이거나 단일 요소이면 ".." 를 덧붙인다
    return detail::JoinPath(dir, "..");
}

bool TryCandidates(const simul::Array<simul::String>& cands, simul::String* resolved) {
    for (size_t i = 0; i < cands.size(); i++) {
        if (FileExists(cands[i])) {
            *resolved = cands[i];
            return true;
        }
    }
    return false;
}

}  // namespace

bool ResolveMeshPath(const simul::String& filename, const simul::String& baseDir, const LoadOptions& opt, simul::String* resolved) {
    if (!resolved) return false;
    resolved->clear();
    if (filename.empty()) return false;
    simul::Array<simul::String> cands;

    const simul::String pkgPrefix = "package://";
    const simul::String filePrefix = "file://";
    if (filename.compare(0, pkgPrefix.size(), pkgPrefix) == 0) {
        const simul::String rest = filename.substr(pkgPrefix.size());
        const size_t slash = rest.find('/');
        const simul::String pkg = slash == simul::String::npos ? rest : rest.substr(0, slash);
        const simul::String sub = slash == simul::String::npos ? simul::String() : rest.substr(slash + 1);
        for (size_t i = 0; i < opt.packagePaths.size(); i++)
            if (opt.packagePaths[i].first == pkg) cands.push_back(detail::JoinPath(opt.packagePaths[i].second, sub));
        for (size_t i = 0; i < opt.searchDirs.size(); i++) {
            cands.push_back(detail::JoinPath(detail::JoinPath(opt.searchDirs[i], pkg), sub));
            cands.push_back(detail::JoinPath(opt.searchDirs[i], sub));
        }
        // URDF 폴더에서 위로 올라가며 찾는다 (pkg/urdf/robot.urdf -> pkg/meshes/..)
        simul::String dir = baseDir.empty() ? simul::String(".") : baseDir;
        for (int up = 0; up < 6; up++) {
            cands.push_back(detail::JoinPath(dir, sub));
            cands.push_back(detail::JoinPath(detail::JoinPath(dir, pkg), sub));
            dir = ParentDir(dir);
        }
    } else if (filename.compare(0, filePrefix.size(), filePrefix) == 0) {
        cands.push_back(filename.substr(filePrefix.size()));
    } else if (filename[0] == '/' || filename[0] == '\\' || (filename.size() > 2 && filename[1] == ':')) {
        cands.push_back(filename);
    } else {
        cands.push_back(detail::JoinPath(baseDir, filename));
        for (size_t i = 0; i < opt.searchDirs.size(); i++) cands.push_back(detail::JoinPath(opt.searchDirs[i], filename));
    }
    return TryCandidates(cands, resolved);
}

bool LoadGeometryScene(const Geometry& g, const simul::String& baseDir, const LoadOptions& opt, Scene* out,
                       simul::String* error, simul::Map<simul::String, Scene>* cache) {
    (void)cache;
    if (error) error->clear();
    if (!out) return false;
    if (g.type == kGeomBox) {
        *out = SceneFromPrimitive(MakeBox(g.size[0], g.size[1], g.size[2]), "box");
        return true;
    }
    if (g.type == kGeomCylinder) {
        *out = SceneFromPrimitive(MakeCylinder(g.radius, g.length), "cylinder");
        return true;
    }
    if (g.type == kGeomSphere) {
        *out = SceneFromPrimitive(MakeSphere(g.radius), "sphere");
        return true;
    }
    if (g.type != kGeomMesh) return false;  // geometry 없음

    simul::String path;
    if (!ResolveMeshPath(g.filename, baseDir, opt, &path)) {
        detail::SetError(error, "메시 파일을 찾을 수 없습니다 (" + g.filename + ")");
        return false;
    }
    // 읽은 파일은 LoadMeshCached 가 기억해 두므로 같은 파일을 다시 읽어도 파싱하지 않는다 (cache 인자는 쓰지 않는다)
    Scene& sc = *out;
    {
        simul::String err;
        if (!LoadMeshCached(path, &sc, &err)) {
            detail::SetError(error, err);
            return false;
        }
    }
    // URDF 의 scale 은 파일 좌표축 기준이므로 축 변환보다 먼저 적용한다
    if (g.scale[0] != 1.0f || g.scale[1] != 1.0f || g.scale[2] != 1.0f)
        sc.ApplyRootTransform(dhMat::Scale(g.scale[0], g.scale[1], g.scale[2]));
    const simul::String ext = detail::LowerExtension(path);
    if (opt.glbYUpToZUp && (ext == "glb" || ext == "gltf")) sc.ApplyRootTransform(dhMat::RotX(static_cast<float>(dhRad(90.0))));
    return true;
}

bool LoadRobotMeshes(const Robot& robot, const LoadOptions& opt, RobotMeshes* out) {
    if (!out) return false;
    out->visuals.clear();
    out->warnings.clear();
    bool allOk = true;
    simul::Map<simul::String, Scene> cache;  // 해석된 경로 -> 읽은 메시 (scale 적용 전)

    for (size_t li = 0; li < robot.links.size(); li++) {
        const Link& link = robot.links[li];
        for (size_t vi = 0; vi < link.visuals.size(); vi++) {
            const Visual& v = link.visuals[vi];
            if (v.geometry.type == kGeomNone) continue;
            LinkVisual lv;
            lv.link = static_cast<int>(li);
            lv.visual = static_cast<int>(vi);
            lv.local = v.origin;
            if (v.hasMaterial && v.material.hasColor) {
                lv.hasColor = true;
                lv.color = v.material.color;
            }
            simul::String err;
            if (!LoadGeometryScene(v.geometry, robot.baseDir, opt, &lv.scene, &err, &cache)) {
                out->warnings.push_back("link '" + link.name + "' visual " + simul::ToString(vi) + ": " + err);
                allOk = false;
                continue;
            }
            out->visuals.push_back(simul::Move(lv));   // 큰 장면을 복사하지 않고 넘긴다
        }
    }
    return allOk;
}

}  // namespace urdf
}  // namespace simul
