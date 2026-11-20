#include "CSensor/Camera.h"

#include <math.h>

#include "CBase/Math.h"
#include "CCollision/MeshRaycast.h"

namespace simul {
namespace sensor {

void DistortNormalized(const float dist[5], float x, float y, float* xd, float* yd) {
    const float k1 = dist[0], k2 = dist[1], p1 = dist[2], p2 = dist[3], k3 = dist[4];
    const float r2 = x * x + y * y;
    const float radial = 1.0f + r2 * (k1 + r2 * (k2 + r2 * k3));
    *xd = x * radial + 2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x);
    *yd = y * radial + p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y;
}

bool UndistortNormalized(const float dist[5], float xd, float yd, float* x, float* y) {
    const float k1 = dist[0], k2 = dist[1], p1 = dist[2], p2 = dist[3], k3 = dist[4];
    float ux = xd, uy = yd;
    for (int it = 0; it < 50; it++) {   // 고정점 반복: 왜곡 없는 좌표 = (왜곡된 좌표 - 접선 성분) / 방사 성분
        const float r2 = ux * ux + uy * uy;
        const float radial = 1.0f + r2 * (k1 + r2 * (k2 + r2 * k3));
        if (!(radial > 1e-3f) || !IsFinite(radial)) return false;
        const float dx = 2.0f * p1 * ux * uy + p2 * (r2 + 2.0f * ux * ux);
        const float dy = p1 * (r2 + 2.0f * uy * uy) + 2.0f * p2 * ux * uy;
        const float nx = (xd - dx) / radial, ny = (yd - dy) / radial;
        if (!IsFinite(nx) || !IsFinite(ny)) return false;
        const bool done = fabsf(nx - ux) < 1e-7f && fabsf(ny - uy) < 1e-7f;
        ux = nx;
        uy = ny;
        if (done) break;
    }
    // 되돌린 값을 다시 왜곡해서 원래 값과 맞는지 확인 (반복이 발산했거나 다른 해로 갔는지 거른다)
    float cx, cy;
    DistortNormalized(dist, ux, uy, &cx, &cy);
    const float tol = 1e-4f * (1.0f + fabsf(xd) + fabsf(yd));
    if (!(fabsf(cx - xd) <= tol && fabsf(cy - yd) <= tol)) return false;
    *x = ux;
    *y = uy;
    return true;
}

bool CameraImage::PixelPoint(int u, int v, dhVector* p) const {
    if (u < 0 || v < 0 || u >= width || v >= height) return false;
    const float d = Depth(u, v);
    if (!(d > 0.0f)) return false;
    float x = (static_cast<float>(u) + 0.5f - intrinsics.cx) / intrinsics.fx, y = (static_cast<float>(v) + 0.5f - intrinsics.cy) / intrinsics.fy;
    const bool distorted = distortion[0] != 0.0f || distortion[1] != 0.0f || distortion[2] != 0.0f || distortion[3] != 0.0f || distortion[4] != 0.0f;
    if (distorted && !UndistortNormalized(distortion, x, y, &x, &y)) return false;
    *p = dhVector(d, -x * d, -y * d);
    return true;
}

Camera::Camera(const CameraSettings& settings) : settings_(settings), rng_(settings.seed) {}

void Camera::SetSettings(const CameraSettings& s) {
    settings_ = s;
    rng_ = s.seed;
}

static bool AllFinite5(const float* v) {
    for (int i = 0; i < 5; i++)
        if (!IsFinite(v[i])) return false;
    return true;
}

bool Camera::Valid() const {
    return settings_.width > 0 && settings_.height > 0 && settings_.width <= 16384 && settings_.height <= 16384 &&
           settings_.antialias >= 1 && settings_.antialias <= 4 && settings_.width * settings_.antialias <= 16384 && settings_.height * settings_.antialias <= 16384 &&
           settings_.horizontalFov > 0.0f && settings_.horizontalFov < static_cast<float>(dhPI) - 0.01f && settings_.nearClip >= 0.0f && settings_.farClip > settings_.nearClip &&
           IsFinite(settings_.farClip) && AllFinite5(settings_.distortion);
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

bool Camera::PixelRay(float u, float v, dhVector* dir) const {
    const CameraIntrinsics k = Intrinsics();
    float x = (u - k.cx) / k.fx, y = (v - k.cy) / k.fy;
    if (settings_.HasDistortion() && !UndistortNormalized(settings_.distortion, x, y, &x, &y)) {
        *dir = dhVector(1.0f, -x, -y);
        return false;
    }
    *dir = dhVector(1.0f, -x, -y);
    return true;
}

dhVector Camera::PixelDirection(float u, float v) const {
    dhVector d;
    PixelRay(u, v, &d);
    return d;
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
    if (settings_.antialias > 1) {   // 큰 영상으로 찍은 뒤 줄인다
        const int k = settings_.antialias;
        CameraSettings hi = settings_;
        hi.width *= k;
        hi.height *= k;
        hi.antialias = 1;
        hi.noiseStdDev = 0.0f;   // 잡음은 줄인 영상에 한 번만 더한다
        hi.colorNoiseStdDev = 0.0f;
        Camera big(hi);
        big.sun_ = sun_;
        CameraImage src;
        big.Render(sensorPose, targets, targetCount, &src);
        const size_t w = static_cast<size_t>(settings_.width), h = static_cast<size_t>(settings_.height), K = static_cast<size_t>(k), W = static_cast<size_t>(hi.width);
        out->width = settings_.width;
        out->height = settings_.height;
        out->intrinsics = Intrinsics();
        for (int i = 0; i < 5; i++) out->distortion[i] = settings_.distortion[i];
        out->bgr.assign(w * h * 3, 0);
        out->depth.assign(w * h, 0.0f);
        out->targetIds.assign(w * h, -1);
        const size_t c = K / 2;   // 가운데에 가장 가까운 하위 픽셀 (K 가 짝수면 아래쪽 오른쪽)
        for (size_t y = 0; y < h; y++)
            for (size_t x = 0; x < w; x++) {
                unsigned sum[3] = {0, 0, 0};
                for (size_t sy = 0; sy < K; sy++)
                    for (size_t sx = 0; sx < K; sx++) {
                        const size_t si = ((y * K + sy) * W + (x * K + sx)) * 3;
                        for (int ch = 0; ch < 3; ch++) sum[ch] += src.bgr[si + static_cast<size_t>(ch)];
                    }
                const size_t i = y * w + x, ci = (y * K + c) * W + (x * K + c);
                for (int ch = 0; ch < 3; ch++) out->bgr[i * 3 + static_cast<size_t>(ch)] = static_cast<uint8_t>((sum[ch] + K * K / 2) / (K * K));
                out->depth[i] = src.depth[ci];
                out->targetIds[i] = src.targetIds[ci];
                if (settings_.noiseStdDev > 0.0f && out->targetIds[i] >= 0) {
                    const float d = out->depth[i] + settings_.noiseStdDev * gaussian();
                    out->depth[i] = d > 0.0f ? d : 0.0f;
                }
            }
        addColorNoise(out);
        return;
    }
    const size_t w = static_cast<size_t>(settings_.width), h = static_cast<size_t>(settings_.height), n = w * h;
    out->width = settings_.width;
    out->height = settings_.height;
    out->intrinsics = Intrinsics();
    for (int i = 0; i < 5; i++) out->distortion[i] = settings_.distortion[i];
    out->depth.assign(n, 0.0f);
    out->targetIds.assign(n, -1);
    out->bgr.assign(n * 3, 0);
    const float* bgc = sun_.enabled ? sun_.sky : settings_.background;
    const uint8_t bgB = ToByte(bgc[2]), bgG = ToByte(bgc[1]), bgR = ToByte(bgc[0]);
    for (size_t i = 0; i < n; i++) {
        out->bgr[i * 3] = bgB;
        out->bgr[i * 3 + 1] = bgG;
        out->bgr[i * 3 + 2] = bgR;
    }
    Array<float> best(n, HUGE_VALF);   // 지금까지 가장 가까운 깊이
    Array<int> who(n, -1);             // 맞은 targets 번호
    Array<size_t> hitTri(n, 0);        // 맞은 삼각형 번호

    // 광선 방향(센서)은 (1, dy, dz). 왜곡이 없으면 dy 는 열마다, dz 는 행마다 같지만, 왜곡이 있으면 픽셀마다 다르다.
    // 그래서 픽셀마다 배열로 둔다 (dy[i], dz[i], i = v * w + u). 왜곡을 되돌리지 못한 픽셀은 ray[i] = 0 으로 표시하고 광선을 쏘지 않는다.
    Array<float> dy(n), dz(n);
    Array<unsigned char> ray(n, 1);
    if (!settings_.HasDistortion()) {
        for (size_t v = 0; v < h; v++)
            for (size_t u = 0; u < w; u++) {
                dy[v * w + u] = PixelDirection(static_cast<float>(u) + 0.5f, 0.0f).y;
                dz[v * w + u] = PixelDirection(0.0f, static_cast<float>(v) + 0.5f).z;
            }
    } else {
        for (size_t v = 0; v < h; v++)
            for (size_t u = 0; u < w; u++) {
                dhVector d;
                const size_t i = v * w + u;
                ray[i] = PixelRay(static_cast<float>(u) + 0.5f, static_cast<float>(v) + 0.5f, &d) ? 1 : 0;
                dy[i] = d.y;
                dz[i] = d.z;
            }
    }

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
                if (!ray[i]) continue;
                const float sd[3] = {1.0f, dy[i], dz[i]};
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
                best[i] = t;
                hitTri[i] = tri;
                who[i] = static_cast<int>(k);
            }
        }
    }

    // 태양: 월드 방향을 정규화하고, 그림자 광선을 위해 메시마다 월드 -> 메시 변환을 한 번만 만든다
    float L[3] = {sun_.direction[0], sun_.direction[1], sun_.direction[2]};
    const float ll = sqrtf(L[0] * L[0] + L[1] * L[1] + L[2] * L[2]);
    const bool sunOn = sun_.enabled && ll > 1e-12f;
    if (sunOn) {
        L[0] /= ll;
        L[1] /= ll;
        L[2] /= ll;
    }
    Array<dhMat> worldToMesh;
    if (sunOn && sun_.shadows && sun_.intensity > 0.0f)
        for (size_t k = 0; k < targetCount; k++) worldToMesh.push_back(targets[k].mesh.world.Inv());

    for (size_t i = 0; i < n; i++) {
        if (who[i] < 0) continue;
        const CameraTarget& ct = targets[static_cast<size_t>(who[i])];
        const LidarTarget& tg = ct.mesh;
        const float t = best[i];
        // 면 법선 (메시 좌표) -> 월드. 광선 방향(월드)과 같은 쪽이면 뒤집어 카메라를 향하게 한다 (양면)
        float nm[3] = {0.0f, 0.0f, 1.0f};
        bool haveN = false;
        const size_t tri = hitTri[i];
        if (tri * 3 + 2 < tg.indexCount) {
            const uint32_t ia = tg.indices[tri * 3], ib = tg.indices[tri * 3 + 1], ic = tg.indices[tri * 3 + 2];
            if (ia < tg.vertexCount && ib < tg.vertexCount && ic < tg.vertexCount) {
                const float* a = tg.positions + static_cast<size_t>(ia) * tg.stride;
                const float* b = tg.positions + static_cast<size_t>(ib) * tg.stride;
                const float* c = tg.positions + static_cast<size_t>(ic) * tg.stride;
                const float e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
                nm[0] = e1[1] * e2[2] - e1[2] * e2[1];
                nm[1] = e1[2] * e2[0] - e1[0] * e2[2];
                nm[2] = e1[0] * e2[1] - e1[1] * e2[0];
                const float nl = sqrtf(nm[0] * nm[0] + nm[1] * nm[1] + nm[2] * nm[2]);
                if (nl > 1e-20f) {
                    nm[0] /= nl; nm[1] /= nl; nm[2] /= nl;
                    haveN = true;
                }
            }
        }
        const dhMat& W = tg.world;   // 메시 -> 월드 (강체)
        float nw[3] = {W.v[0] * nm[0] + W.v[4] * nm[1] + W.v[8] * nm[2], W.v[1] * nm[0] + W.v[5] * nm[1] + W.v[9] * nm[2], W.v[2] * nm[0] + W.v[6] * nm[1] + W.v[10] * nm[2]};
        // 센서 좌표의 광선 방향 -> 월드
        const float sdv[3] = {1.0f, dy[i], dz[i]};
        float dwv[3] = {sensorPose.v[0] * sdv[0] + sensorPose.v[4] * sdv[1] + sensorPose.v[8] * sdv[2], sensorPose.v[1] * sdv[0] + sensorPose.v[5] * sdv[1] + sensorPose.v[9] * sdv[2],
                        sensorPose.v[2] * sdv[0] + sensorPose.v[6] * sdv[1] + sensorPose.v[10] * sdv[2]};
        const float dl = sqrtf(dwv[0] * dwv[0] + dwv[1] * dwv[1] + dwv[2] * dwv[2]);
        if (dl > 1e-20f) { dwv[0] /= dl; dwv[1] /= dl; dwv[2] /= dl; }
        if (nw[0] * dwv[0] + nw[1] * dwv[1] + nw[2] * dwv[2] > 0.0f) { nw[0] = -nw[0]; nw[1] = -nw[1]; nw[2] = -nw[2]; }

        float rgb[3];
        if (!sunOn) {
            float s = 1.0f;
            if (haveN) s = 0.3f + 0.7f * fabsf(nw[0] * dwv[0] + nw[1] * dwv[1] + nw[2] * dwv[2]);
            rgb[0] = ct.color[0] * s;
            rgb[1] = ct.color[1] * s;
            rgb[2] = ct.color[2] * s;
        } else {
            float ndl = haveN ? nw[0] * L[0] + nw[1] * L[1] + nw[2] * L[2] : 1.0f;
            ndl = ndl < 0.0f ? 0.0f : ndl;
            bool lit = sun_.intensity > 0.0f && ndl > 0.0f;
            if (lit && sun_.shadows) {
                // 맞은 점(월드)에서 법선 쪽으로 조금 띄워 태양 쪽으로 광선을 쏜다. 아무 메시나 맞으면 그림자.
                const dhMat Wn = sensorPose;
                const float P[3] = {Wn.v[0] * t + Wn.v[4] * dy[i] * t + Wn.v[8] * dz[i] * t + Wn.v[12], Wn.v[1] * t + Wn.v[5] * dy[i] * t + Wn.v[9] * dz[i] * t + Wn.v[13],
                                    Wn.v[2] * t + Wn.v[6] * dy[i] * t + Wn.v[10] * dz[i] * t + Wn.v[14]};
                const float eps = 2e-3f;
                const dhVector origin(P[0] + nw[0] * eps, P[1] + nw[1] * eps, P[2] + nw[2] * eps);
                for (size_t k = 0; k < targetCount && lit; k++) {
                    const LidarTarget& sg = targets[k].mesh;
                    if (!sg.positions || !sg.indices || sg.vertexCount == 0 || sg.indexCount < 3) continue;
                    const dhMat& Mi = worldToMesh[k];
                    const dhVector om = Mi * origin;
                    const dhVector dm(Mi.v[0] * L[0] + Mi.v[4] * L[1] + Mi.v[8] * L[2], Mi.v[1] * L[0] + Mi.v[5] * L[1] + Mi.v[9] * L[2], Mi.v[2] * L[0] + Mi.v[6] * L[1] + Mi.v[10] * L[2]);
                    float th = 0.0f;
                    const float reach = 1.0e4f;
                    const bool useBvh = sg.bvh && !sg.bvh->empty();
                    const bool blocked = useBvh ? sg.bvh->Raycast(om, dm, reach, &th)
                                                : RaycastTriangles(sg.positions, sg.vertexCount, sg.indices, sg.indexCount, om, dm, reach, &th, 0, sg.stride);
                    if (blocked) lit = false;
                }
            }
            const float sunTerm = lit ? 0.85f * sun_.intensity * ndl : 0.0f;
            const float amb = 0.45f * sun_.ambient;
            for (int c = 0; c < 3; c++) rgb[c] = ct.color[c] * (amb + sunTerm * sun_.color[c]);
        }
        float depth = t;
        if (settings_.noiseStdDev > 0.0f) depth += settings_.noiseStdDev * gaussian();
        out->depth[i] = depth > 0.0f ? depth : 0.0f;
        out->targetIds[i] = ct.mesh.id;
        out->bgr[i * 3] = ToByte(rgb[2]);
        out->bgr[i * 3 + 1] = ToByte(rgb[1]);
        out->bgr[i * 3 + 2] = ToByte(rgb[0]);
    }
    addColorNoise(out);
}

// 모든 픽셀의 B, G, R 에 따로 가우시안 잡음을 더한다 (0..255 로 자른다)
void Camera::addColorNoise(CameraImage* out) {
    const float sd = settings_.colorNoiseStdDev * 255.0f;
    if (!(sd > 0.0f)) return;
    for (size_t k = 0; k < out->bgr.size(); k++) {
        const float v = static_cast<float>(out->bgr[k]) + sd * gaussian() + 0.5f;
        out->bgr[k] = static_cast<uint8_t>(v < 0.0f ? 0.0f : (v > 255.0f ? 255.0f : v));
    }
}

}  // namespace sensor
}  // namespace simul
