#include "CMesh/Mesh.h"

#include "CBase/Math.h"

namespace simul {

void Primitive::ComputeNormals() {
    const size_t n = VertexCount();
    normals.assign(n * 3, 0.0f);
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        const uint32_t i0 = indices[t], i1 = indices[t + 1], i2 = indices[t + 2];
        if (i0 >= n || i1 >= n || i2 >= n) continue;
        const dhVector p0(positions[i0 * 3], positions[i0 * 3 + 1], positions[i0 * 3 + 2]);
        const dhVector p1(positions[i1 * 3], positions[i1 * 3 + 1], positions[i1 * 3 + 2]);
        const dhVector p2(positions[i2 * 3], positions[i2 * 3 + 1], positions[i2 * 3 + 2]);
        // 외적의 크기가 삼각형 면적의 2배이므로 그대로 더하면 면적 가중 평균이 된다
        const dhVector fn = (p1 - p0) * (p2 - p0);
        const uint32_t ids[3] = {i0, i1, i2};
        for (int k = 0; k < 3; k++) {
            normals[ids[k] * 3] += fn.x;
            normals[ids[k] * 3 + 1] += fn.y;
            normals[ids[k] * 3 + 2] += fn.z;
        }
    }
    for (size_t i = 0; i < n; i++) {
        float* nv = &normals[i * 3];
        const float len = simul::Sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
        if (len > 1e-12f) {
            nv[0] /= len;
            nv[1] /= len;
            nv[2] /= len;
        } else {  // 면에 속하지 않았거나 넓이가 0인 정점
            nv[0] = 0.0f;
            nv[1] = 0.0f;
            nv[2] = 1.0f;
        }
    }
}

bool Primitive::IsValid() const {
    if (positions.size() % 3 != 0) return false;
    const size_t n = VertexCount();
    if (!normals.empty() && normals.size() != n * 3) return false;
    if (!uvs.empty() && uvs.size() != n * 2) return false;
    if (indices.size() % 3 != 0) return false;
    for (size_t i = 0; i < indices.size(); i++) {
        if (indices[i] >= n) return false;
    }
    return true;
}

void Scene::Clear() {
    name.clear();
    materials.clear();
    meshes.clear();
    nodes.clear();
    roots.clear();
    warnings.clear();
}

simul::Array<dhMat> Scene::WorldTransforms() const {
    simul::Array<dhMat> world(nodes.size());
    simul::Array<char> done(nodes.size(), 0);
    simul::Array<int> stack;
    for (size_t r = 0; r < roots.size(); r++) {
        const int root = roots[r];
        if (root < 0 || static_cast<size_t>(root) >= nodes.size() || done[static_cast<size_t>(root)]) continue;
        world[static_cast<size_t>(root)] = nodes[static_cast<size_t>(root)].local;
        done[static_cast<size_t>(root)] = 1;
        stack.push_back(root);
        while (!stack.empty()) {
            const size_t cur = static_cast<size_t>(stack.back());
            stack.pop_back();
            for (size_t c = 0; c < nodes[cur].children.size(); c++) {
                const int child = nodes[cur].children[c];
                if (child < 0 || static_cast<size_t>(child) >= nodes.size() || done[static_cast<size_t>(child)]) continue;
                world[static_cast<size_t>(child)] = world[cur] * nodes[static_cast<size_t>(child)].local;
                done[static_cast<size_t>(child)] = 1;
                stack.push_back(child);
            }
        }
    }
    // 루트에서 닿지 않는 노드는 자신의 변환만 사용한다
    for (size_t i = 0; i < nodes.size(); i++) {
        if (!done[i]) world[i] = nodes[i].local;
    }
    return world;
}

void Scene::ApplyRootTransform(const dhMat& t) {
    for (size_t r = 0; r < roots.size(); r++) {
        const int root = roots[r];
        if (root < 0 || static_cast<size_t>(root) >= nodes.size()) continue;
        nodes[static_cast<size_t>(root)].local = t * nodes[static_cast<size_t>(root)].local;
    }
}

bool Scene::ComputeBounds(dhVector* outMin, dhVector* outMax) const {
    const simul::Array<dhMat> world = WorldTransforms();
    bool any = false;
    dhVector lo, hi;
    for (size_t i = 0; i < nodes.size(); i++) {
        const int mi = nodes[i].mesh;
        if (mi < 0 || static_cast<size_t>(mi) >= meshes.size()) continue;
        const Mesh& mesh = meshes[static_cast<size_t>(mi)];
        for (size_t p = 0; p < mesh.primitives.size(); p++) {
            const simul::Array<float>& pos = mesh.primitives[p].positions;
            for (size_t v = 0; v + 2 < pos.size(); v += 3) {
                const dhVector w = world[i] * dhVector(pos[v], pos[v + 1], pos[v + 2]);
                if (!any) {
                    lo = w;
                    hi = w;
                    any = true;
                } else {
                    if (w.x < lo.x) lo.x = w.x;
                    if (w.y < lo.y) lo.y = w.y;
                    if (w.z < lo.z) lo.z = w.z;
                    if (w.x > hi.x) hi.x = w.x;
                    if (w.y > hi.y) hi.y = w.y;
                    if (w.z > hi.z) hi.z = w.z;
                }
            }
        }
    }
    if (!any) return false;
    if (outMin) *outMin = lo;
    if (outMax) *outMax = hi;
    return true;
}

size_t Scene::TriangleCount() const {
    size_t count = 0;
    for (size_t i = 0; i < nodes.size(); i++) {
        const int mi = nodes[i].mesh;
        if (mi < 0 || static_cast<size_t>(mi) >= meshes.size()) continue;
        const Mesh& mesh = meshes[static_cast<size_t>(mi)];
        for (size_t p = 0; p < mesh.primitives.size(); p++) count += mesh.primitives[p].TriangleCount();
    }
    return count;
}

bool Scene::IsValid() const {
    const size_t nn = nodes.size();
    for (size_t m = 0; m < meshes.size(); m++) {
        for (size_t p = 0; p < meshes[m].primitives.size(); p++) {
            const Primitive& prim = meshes[m].primitives[p];
            if (!prim.IsValid()) return false;
            if (prim.material < -1 || prim.material >= static_cast<int>(materials.size())) return false;
        }
    }
    for (size_t i = 0; i < nn; i++) {
        const SceneNode& n = nodes[i];
        if (n.mesh < -1 || n.mesh >= static_cast<int>(meshes.size())) return false;
        if (n.parent < -1 || n.parent >= static_cast<int>(nn)) return false;
        for (size_t c = 0; c < n.children.size(); c++) {
            const int child = n.children[c];
            if (child < 0 || static_cast<size_t>(child) >= nn) return false;
            if (nodes[static_cast<size_t>(child)].parent != static_cast<int>(i)) return false;
        }
    }
    for (size_t r = 0; r < roots.size(); r++) {
        if (roots[r] < 0 || static_cast<size_t>(roots[r]) >= nn) return false;
        if (nodes[static_cast<size_t>(roots[r])].parent != -1) return false;
    }
    return true;
}

}  // namespace simul
