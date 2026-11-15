// URDF 의 메시 경로 해석과 visual 로딩
#include <fstream>
#include <map>

#include "3d/dhMat.h"
#include "CMesh/MeshLoader.h"
#include "CMesh/Primitives.h"
#include "CURDF/Urdf.h"
#include "FileUtil.h"

namespace amr {
namespace urdf {

namespace {

bool FileExists(const std::string& p) {
    std::ifstream f(p.c_str(), std::ios::in | std::ios::binary);
    return f.good();
}

std::string ParentDir(const std::string& dir) {
    if (dir.empty()) return "..";
    const std::string d = detail::DirName(dir);
    if (!d.empty() && d != dir) return d;
    // 마지막 요소가 ".." 이거나 단일 요소이면 ".." 를 덧붙인다
    return detail::JoinPath(dir, "..");
}

bool TryCandidates(const std::vector<std::string>& cands, std::string* resolved) {
    for (size_t i = 0; i < cands.size(); i++) {
        if (FileExists(cands[i])) {
            *resolved = cands[i];
            return true;
        }
    }
    return false;
}

}  // namespace

bool ResolveMeshPath(const std::string& filename, const std::string& baseDir, const LoadOptions& opt, std::string* resolved) {
    if (!resolved) return false;
    resolved->clear();
    if (filename.empty()) return false;
    std::vector<std::string> cands;

    const std::string pkgPrefix = "package://";
    const std::string filePrefix = "file://";
    if (filename.compare(0, pkgPrefix.size(), pkgPrefix) == 0) {
        const std::string rest = filename.substr(pkgPrefix.size());
        const size_t slash = rest.find('/');
        const std::string pkg = slash == std::string::npos ? rest : rest.substr(0, slash);
        const std::string sub = slash == std::string::npos ? std::string() : rest.substr(slash + 1);
        for (size_t i = 0; i < opt.packagePaths.size(); i++)
            if (opt.packagePaths[i].first == pkg) cands.push_back(detail::JoinPath(opt.packagePaths[i].second, sub));
        for (size_t i = 0; i < opt.searchDirs.size(); i++) {
            cands.push_back(detail::JoinPath(detail::JoinPath(opt.searchDirs[i], pkg), sub));
            cands.push_back(detail::JoinPath(opt.searchDirs[i], sub));
        }
        // URDF 폴더에서 위로 올라가며 찾는다 (pkg/urdf/robot.urdf -> pkg/meshes/..)
        std::string dir = baseDir.empty() ? std::string(".") : baseDir;
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

bool LoadGeometryScene(const Geometry& g, const std::string& baseDir, const LoadOptions& opt, Scene* out,
                       std::string* error, std::map<std::string, Scene>* cache) {
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

    std::string path;
    if (!ResolveMeshPath(g.filename, baseDir, opt, &path)) {
        detail::SetError(error, "메시 파일을 찾을 수 없습니다 (" + g.filename + ")");
        return false;
    }
    Scene sc;
    std::map<std::string, Scene>::iterator it;
    if (cache && (it = cache->find(path)) != cache->end()) {
        sc = it->second;
    } else {
        std::string err;
        if (!LoadMesh(path, &sc, &err)) {
            detail::SetError(error, err);
            return false;
        }
        if (cache) cache->insert(std::make_pair(path, sc));
    }
    // URDF 의 scale 은 파일 좌표축 기준이므로 축 변환보다 먼저 적용한다
    if (g.scale[0] != 1.0f || g.scale[1] != 1.0f || g.scale[2] != 1.0f)
        sc.ApplyRootTransform(dhMat::Scale(g.scale[0], g.scale[1], g.scale[2]));
    const std::string ext = detail::LowerExtension(path);
    if (opt.glbYUpToZUp && (ext == "glb" || ext == "gltf")) sc.ApplyRootTransform(dhMat::RotX(static_cast<float>(dhRad(90.0))));
    *out = sc;
    return true;
}

bool LoadRobotMeshes(const Robot& robot, const LoadOptions& opt, RobotMeshes* out) {
    if (!out) return false;
    out->visuals.clear();
    out->warnings.clear();
    bool allOk = true;
    std::map<std::string, Scene> cache;  // 해석된 경로 -> 읽은 메시 (scale 적용 전)

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
            std::string err;
            if (!LoadGeometryScene(v.geometry, robot.baseDir, opt, &lv.scene, &err, &cache)) {
                out->warnings.push_back("link '" + link.name + "' visual " + std::to_string(vi) + ": " + err);
                allOk = false;
                continue;
            }
            out->visuals.push_back(lv);
        }
    }
    return allOk;
}

}  // namespace urdf
}  // namespace amr
