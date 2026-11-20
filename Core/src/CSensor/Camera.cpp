#include "CSensor/Camera.h"

#include <math.h>

#include "CBase/Math.h"
#include "CCollision/MeshRaycast.h"

namespace simul {
namespace sensor {

bool CameraImage::PixelPoint(int u, int v, dhVector* p) const {
    if (u < 0 || v < 0 || u >= width || v >= height) return false;
    const float d = Depth(u, v);
    if (!(d > 0.0f)) return false;
    *p = dhVector(d, -(static_cast<float>(u) + 0.5f - intrinsics.cx) / intrinsics.fx * d, -(static_cast<float>(v) + 0.5f - intrinsics.cy) / intrinsics.fy * d);
    return true;
}

Camera::Camera(const CameraSettings& settings) : settings_(settings), rng_(settings.seed) {}

void Camera::SetSettings(const CameraSettings& s) {
    settings_ = s;
    rng_ = s.seed;
}

bool Camera::Valid() const {
    return settings_.width > 0 && settings_.height > 0 && settings_.width <= 16384 && settings_.height <= 16384 &&
           settings_.horizontalFov > 0.0f && settings_.horizontalFov < static_cast<float>(dhPI) - 0.01f && settings_.nearClip >= 0.0f && settings_.farClip > settings_.nearClip &&
           IsFinite(settings_.farClip);
}

CameraIntrinsics Camera::Intrinsics() const {
    CameraIntrinsics k;
    k.fx = k.fy = (static_cast<float>(settings_.width) * 0.5f) / tanf(settings_.horizontalFov * 0.5f);
    k.cx = static_cast<float>(settings_.width) * 0.5f;
    k.cy = static_cast<float>(settings_.height) * 0.5f;
    return k;
}

float Camera::VerticalFov() const {
    const CameraIntrinsics k = Intrinsics();
    return 2.0f * atanf(static_cast<float>(settings_.height) * 0.5f / k.fy);
}

dhVector Camera::PixelDirection(float u, float v) const {
    const CameraIntrinsics k = Intrinsics();
    return dhVector(1.0f, -(u - k.cx) / k.fx, -(v - k.cy) / k.fy);
}

unsigned Camera::nextRandom() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return rng_ >> 8;
}

float Camera::gaussian() {
    const float u1 = (static_cast<float>(nextRandom() & 0xFFFFFF) + 1.0f) / 16777217.0f;
    const float u2 = static_cast<float>(nextRandom() & 0xFFFFFF) / 16777216.0f;
    return Sqrt(-2.0f * Log(u1)) * Cos(static_cast<float>(2.0 * dhPI) * u2);
}

namespace {

uint8_t ToByte(float v) {
    const float c = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    return static_cast<uint8_t>(c * 255.0f + 0.5f);
}

}  // namespace

void Camera::Render(const dhMat& sensorPose, const CameraTarget* targets, size_t targetCount, CameraImage* out) {
    out->width = out->height = 0;
    out->bgr.clear();
    out->depth.clear();
    out->targetIds.clear();
    if (!Valid()) return;
    const size_t w = static_cast<size_t>(settings_.width), h = static_cast<size_t>(settings_.height), n = w * h;
    out->width = settings_.width;
    out->height = settings_.height;
    out->intrinsics = Intrinsics();
    out->depth.assign(n, 0.0f);
    out->targetIds.assign(n, -1);
    out->bgr.assign(n * 3, 0);
    const uint8_t bgB = ToByte(settings_.background[2]), bgG = ToByte(settings_.background[1]), bgR = ToByte(settings_.background[0]);
    for (size_t i = 0; i < n; i++) {
        out->bgr[i * 3] = bgB;
        out->bgr[i * 3 + 1] = bgG;
        out->bgr[i * 3 + 2] = bgR;
    }
    Array<float> best(n, HUGE_VALF);   // 지금까지 가장 가까운 깊이
    Array<float> shade(n, 1.0f);       // 밝기
    Array<int> who(n, -1);             // 맞은 targets 번호

    // 광선 방향(센서)은 (1, a_u, a_v) 꼴이라 열마다/행마다 한 번만 계산한다
    Array<float> dy(w), dz(h);
    for (size_t u = 0; u < w; u++) dy[u] = PixelDirection(static_cast<float>(u) + 0.5f, 0.0f).y;
    for (size_t v = 0; v < h; v++) dz[v] = PixelDirection(0.0f, static_cast<float>(v) + 0.5f).z;

    for (size_t k = 0; k < targetCount; k++) {
        const CameraTarget& ct = targets[k];
        const LidarTarget& tg = ct.mesh;
        if (!tg.positions || !tg.indices || tg.vertexCount == 0 || tg.indexCount < 3) continue;
        const dhMat M = tg.world.Inv() * sensorPose;  // 센서 -> 메시 좌표
        const float o[3] = {M.v[12], M.v[13], M.v[14]};
        const bool useBvh = tg.bvh && !tg.bvh->empty();
        for (size_t v = 0; v < h; v++) {
            for (size_t u = 0; u < w; u++) {
                const size_t i = v * w + u;
                const float sd[3] = {1.0f, dy[u], dz[v]};
                const float d[3] = {M.v[0] * sd[0] + M.v[4] * sd[1] + M.v[8] * sd[2], M.v[1] * sd[0] + M.v[5] * sd[1] + M.v[9] * sd[2],
                                    M.v[2] * sd[0] + M.v[6] * sd[1] + M.v[10] * sd[2]};
                const float limit = IsFinite(best[i]) ? best[i] : settings_.farClip;  // t 는 곧 깊이 (sd.x = 1, M 은 강체변환)
                float t = 0.0f;
                size_t tri = 0;
                bool hit;
                if (useBvh) hit = tg.bvh->Raycast(dhVector(o[0], o[1], o[2]), dhVector(d[0], d[1], d[2]), limit, &t, &tri);
                else hit = RaycastTriangles(tg.positions, tg.vertexCount, tg.indices, tg.indexCount, dhVector(o[0], o[1], o[2]), dhVector(d[0], d[1], d[2]), limit, &t, &tri, tg.stride);
                if (!hit || t < settings_.nearClip || t > settings_.farClip) continue;
                if (!(t < best[i])) continue;
                // 맞은 삼각형의 면 법선 (메시 좌표) 과 광선의 각도로 밝기를 정한다
                float s = 1.0f;
                if (tri * 3 + 2 < tg.indexCount) {
                    const uint32_t ia = tg.indices[tri * 3], ib = tg.indices[tri * 3 + 1], ic = tg.indices[tri * 3 + 2];
                    if (ia < tg.vertexCount && ib < tg.vertexCount && ic < tg.vertexCount) {
                        const float* a = tg.positions + static_cast<size_t>(ia) * tg.stride;
                        const float* b = tg.positions + static_cast<size_t>(ib) * tg.stride;
                        const float* c = tg.positions + static_cast<size_t>(ic) * tg.stride;
                        const float e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
                        const float nx = e1[1] * e2[2] - e1[2] * e2[1], ny = e1[2] * e2[0] - e1[0] * e2[2], nz = e1[0] * e2[1] - e1[1] * e2[0];
                        const float nl = sqrtf(nx * nx + ny * ny + nz * nz), dl = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                        if (nl > 1e-20f && dl > 1e-20f) s = 0.3f + 0.7f * fabsf((nx * d[0] + ny * d[1] + nz * d[2]) / (nl * dl));
                    }
                }
                best[i] = t;
                shade[i] = s;
                who[i] = static_cast<int>(k);
            }
        }
    }
    for (size_t i = 0; i < n; i++) {
        if (who[i] < 0) continue;
        const CameraTarget& ct = targets[static_cast<size_t>(who[i])];
        float depth = best[i];
        if (settings_.noiseStdDev > 0.0f) depth += settings_.noiseStdDev * gaussian();
        out->depth[i] = depth > 0.0f ? depth : 0.0f;
        out->targetIds[i] = ct.mesh.id;
        out->bgr[i * 3] = ToByte(ct.color[2] * shade[i]);
        out->bgr[i * 3 + 1] = ToByte(ct.color[1] * shade[i]);
        out->bgr[i * 3 + 2] = ToByte(ct.color[0] * shade[i]);
    }
}

}  // namespace sensor
}  // namespace simul
