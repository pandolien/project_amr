#include "CCollision/RobotCollision.h"

#include <math.h>

#include <algorithm>
#include <map>
#include <set>

namespace amr {

namespace {

inline float Abs(float a) { return a < 0 ? -a : a; }

}  // namespace

Aabb AabbOfPrimitive(const Primitive& p, const dhMat& t) {
    Aabb box;
    const size_t n = p.VertexCount();
    for (size_t i = 0; i < n; i++)
        box.Expand(t * dhVector(p.positions[i * 3], p.positions[i * 3 + 1], p.positions[i * 3 + 2]));
    return box;
}

Aabb AabbOfScene(const Scene& scene, const dhMat& t) {
    Aabb box;
    const std::vector<dhMat> world = scene.WorldTransforms();
    for (size_t i = 0; i < scene.nodes.size(); i++) {
        const int mi = scene.nodes[i].mesh;
        if (mi < 0 || static_cast<size_t>(mi) >= scene.meshes.size()) continue;
        const dhMat m = t * world[i];
        const Mesh& mesh = scene.meshes[static_cast<size_t>(mi)];
        for (size_t pi = 0; pi < mesh.primitives.size(); pi++) box.Merge(AabbOfPrimitive(mesh.primitives[pi], m));
    }
    return box;
}

Aabb AabbOfGeometry(const urdf::Geometry& g, const dhMat& world, const Aabb* meshLocal) {
    const dhVector c = world.O();
    switch (g.type) {
        case urdf::kGeomBox: {
            const dhVector h(g.size[0] * 0.5f, g.size[1] * 0.5f, g.size[2] * 0.5f);
            return Aabb::FromCenterHalfExtents(dhVector(0, 0, 0), h).Transformed(world);
        }
        case urdf::kGeomSphere: {
            const float r = g.radius;
            return Aabb::FromCenterHalfExtents(c, dhVector(r, r, r));
        }
        case urdf::kGeomCylinder: {
            // 축 = 도형의 Z. 월드 축 i 로의 반크기 = |축의 i 성분| * L/2 + r * sqrt(1 - (축의 i 성분)^2)
            const float halfLen = g.length * 0.5f;
            float h[3];
            for (int i = 0; i < 3; i++) {
                const float a = world.v[8 + i];
                const float s = 1.0f - a * a;
                h[i] = Abs(a) * halfLen + g.radius * (s > 0.0f ? sqrtf(s) : 0.0f);
            }
            return Aabb::FromCenterHalfExtents(c, dhVector(h[0], h[1], h[2]));
        }
        case urdf::kGeomMesh:
            if (!meshLocal) return Aabb();
            return meshLocal->Transformed(world);
        default:
            return Aabb();
    }
}

bool BuildCollisionModel(const urdf::Robot& robot, const CollisionModelOptions& opt, RobotCollisionModel* out) {
    if (!out) return false;
    out->shapes.clear();
    out->warnings.clear();
    bool allOk = true;
    std::map<std::string, Scene> cache;

    for (size_t li = 0; li < robot.links.size(); li++) {
        const urdf::Link& link = robot.links[li];
        // 이 링크에 쓸 (origin, geometry) 목록
        std::vector<std::pair<dhMat, urdf::Geometry> > geoms;
        bool fromVisual = false;
        for (size_t ci = 0; ci < link.collisions.size(); ci++)
            if (link.collisions[ci].geometry.type != urdf::kGeomNone)
                geoms.push_back(std::make_pair(link.collisions[ci].origin, link.collisions[ci].geometry));
        if (link.collisions.empty() && opt.fallbackToVisual) {
            fromVisual = true;
            for (size_t vi = 0; vi < link.visuals.size(); vi++)
                if (link.visuals[vi].geometry.type != urdf::kGeomNone)
                    geoms.push_back(std::make_pair(link.visuals[vi].origin, link.visuals[vi].geometry));
        }

        for (size_t gi = 0; gi < geoms.size(); gi++) {
            CollisionShape sh;
            sh.link = static_cast<int>(li);
            sh.origin = geoms[gi].first;
            sh.geometry = geoms[gi].second;
            sh.fromVisual = fromVisual;
            if (sh.geometry.type == urdf::kGeomMesh) {
                Scene sc;
                std::string err;
                if (!urdf::LoadGeometryScene(sh.geometry, robot.baseDir, opt.load, &sc, &err, &cache)) {
                    out->warnings.push_back("link '" + link.name + "' collision " + std::to_string(gi) + ": " + err);
                    allOk = false;
                    continue;
                }
                sh.meshBounds = AabbOfScene(sc, dhMat());
                if (sh.meshBounds.IsEmpty()) {
                    out->warnings.push_back("link '" + link.name + "' collision " + std::to_string(gi) + ": 메시에 정점이 없습니다");
                    allOk = false;
                    continue;
                }
            }
            out->shapes.push_back(sh);
        }
    }
    return allOk;
}

std::vector<Aabb> ComputeLinkAabbs(const urdf::Robot& robot, const RobotCollisionModel& model, const urdf::JointValues& q,
                                   bool clampToLimits) {
    std::vector<Aabb> boxes(robot.links.size());
    const std::vector<dhMat> T = robot.LinkTransforms(q, clampToLimits);
    for (size_t i = 0; i < model.shapes.size(); i++) {
        const CollisionShape& s = model.shapes[i];
        if (s.link < 0 || static_cast<size_t>(s.link) >= boxes.size() || static_cast<size_t>(s.link) >= T.size()) continue;
        const dhMat world = T[static_cast<size_t>(s.link)] * s.origin;
        boxes[static_cast<size_t>(s.link)].Merge(AabbOfGeometry(s.geometry, world, &s.meshBounds));
    }
    return boxes;
}

std::vector<CollisionPair> FindSelfCollisions(const urdf::Robot& robot, const std::vector<Aabb>& linkAabbs,
                                              const SelfCollisionOptions& opt) {
    // margin 만큼 가까운 쌍도 찾도록 상자마다 margin/2 씩 넓힌다
    std::vector<Aabb> boxes(linkAabbs);
    if (opt.margin > 0.0f)
        for (size_t i = 0; i < boxes.size(); i++) boxes[i] = boxes[i].Inflated(opt.margin * 0.5f);

    std::set<std::pair<int, int> > skip;
    if (opt.ignoreAdjacent) {
        for (size_t ji = 0; ji < robot.joints.size(); ji++) {
            const int p = robot.FindLink(robot.joints[ji].parent);
            const int c = robot.FindLink(robot.joints[ji].child);
            if (p >= 0 && c >= 0) skip.insert(std::make_pair(std::min(p, c), std::max(p, c)));
        }
    }
    for (size_t i = 0; i < opt.ignoredPairs.size(); i++) {
        const int a = robot.FindLink(opt.ignoredPairs[i].first);
        const int b = robot.FindLink(opt.ignoredPairs[i].second);
        if (a >= 0 && b >= 0) skip.insert(std::make_pair(std::min(a, b), std::max(a, b)));
    }

    std::vector<CollisionPair> pairs = FindOverlappingPairs(boxes);
    std::vector<CollisionPair> out;
    for (size_t i = 0; i < pairs.size(); i++)
        if (skip.find(std::make_pair(pairs[i].a, pairs[i].b)) == skip.end()) out.push_back(pairs[i]);
    return out;
}

}  // namespace amr
