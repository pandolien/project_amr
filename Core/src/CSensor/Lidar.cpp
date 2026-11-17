#include "CSensor/Lidar.h"

#include <math.h>

#include "CBase/Math.h"
#include "CCollision/MeshRaycast.h"

namespace simul {
namespace sensor {

void LidarTarget::ComputeBounds() {
    hasBounds = false;
    if (!positions || vertexCount == 0) return;
    for (int k = 0; k < 3; k++) boundsMin[k] = boundsMax[k] = positions[k];
    for (size_t i = 1; i < vertexCount; i++)
        for (int k = 0; k < 3; k++) {
            const float v = positions[i * stride + static_cast<size_t>(k)];
            if (v < boundsMin[k]) boundsMin[k] = v;
            if (v > boundsMax[k]) boundsMax[k] = v;
        }
    hasBounds = true;
}

namespace {

// 선분 o + t*d (0 <= t <= tMax) 가 경계 상자와 만나는가 (slab 방법)
bool SegmentHitsBox(const float o[3], const float d[3], float tMax, const float bmin[3], const float bmax[3]) {
    float t0 = 0.0f, t1 = tMax;
    for (int k = 0; k < 3; k++) {
        if (Abs(d[k]) < 1e-12f) {
            if (o[k] < bmin[k] || o[k] > bmax[k]) return false;  // 평행인데 두 면 사이 밖
        } else {
            float a = (bmin[k] - o[k]) / d[k], b = (bmax[k] - o[k]) / d[k];
            if (a > b) Swap(a, b);
            if (a > t0) t0 = a;
            if (b < t1) t1 = b;
            if (t0 > t1) return false;
        }
    }
    return true;
}

}  // namespace

Lidar::Lidar(const LidarSettings& settings) : settings_(settings), rng_(settings.seed) {}

void Lidar::SetSettings(const LidarSettings& s) {
    settings_ = s;
    rng_ = s.seed;
}

size_t Lidar::AzimuthCount() const {
    const float span = settings_.angleMax - settings_.angleMin;
    if (!(settings_.angleIncrement > 0.0f) || !(span >= 0.0f) || !IsFinite(span)) return 0;
    return static_cast<size_t>(Floor(span / settings_.angleIncrement + 0.5f)) + 1;  // 반올림: 270 / 0.5 가 539.9999 가 돼도 540
}

size_t Lidar::RayCount() const {
    const size_t channels = settings_.channels > 0 ? static_cast<size_t>(settings_.channels) : 0;
    return AzimuthCount() * channels;
}

float Lidar::ChannelElevation(size_t channel) const {
    const size_t c = settings_.channels > 0 ? static_cast<size_t>(settings_.channels) : 1;
    if (c <= 1) return settings_.elevationMin;
    return settings_.elevationMin + (settings_.elevationMax - settings_.elevationMin) * static_cast<float>(channel) / static_cast<float>(c - 1);
}

unsigned Lidar::nextRandom() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return rng_ >> 8;
}

float Lidar::gaussian() {
    const float u1 = (static_cast<float>(nextRandom() & 0xFFFFFF) + 1.0f) / 16777217.0f;  // (0, 1]
    const float u2 = static_cast<float>(nextRandom() & 0xFFFFFF) / 16777216.0f;
    return Sqrt(-2.0f * Log(u1)) * Cos(static_cast<float>(2.0 * dhPI) * u2);
}

void Lidar::Scan(const dhMat& sensorPose, const LidarTarget* targets, size_t targetCount, LidarScan* out) {
    const size_t nAz = AzimuthCount();
    const size_t nCh = settings_.channels > 0 ? static_cast<size_t>(settings_.channels) : 0;
    const size_t n = nAz * nCh;
    out->angleMin = settings_.angleMin;
    out->angleIncrement = settings_.angleIncrement;
    out->rangeMin = settings_.rangeMin;
    out->rangeMax = settings_.rangeMax;
    out->azimuthCount = nAz;
    out->channelCount = nCh;
    out->elevationMin = settings_.elevationMin;
    out->elevationMax = settings_.elevationMax;
    out->ranges.assign(n, HUGE_VALF);
    out->targetIds.assign(n, -1);
    out->points.assign(n, dhVector());
    if (n == 0) return;

    // 광선마다 센서 좌표의 방향 (cos(phi) cos(theta), cos(phi) sin(theta), sin(phi)) 을 한 번만 계산해 둔다.
    simul::Array<float> dirs(n * 3, 0.0f);
    for (size_t ch = 0; ch < nCh; ch++) {
        const float phi = ChannelElevation(ch);
        const float cp = Cos(phi), sp = Sin(phi);
        for (size_t a = 0; a < nAz; a++) {
            const float th = RayAngle(a);
            float* d = &dirs[(ch * nAz + a) * 3];
            d[0] = cp * Cos(th);
            d[1] = cp * Sin(th);
            d[2] = sp;
        }
    }
    // 수직 채널이 하나이고 수평(phi = 0)이면 모든 광선이 센서 평면 위에 있다 (2D 라이다). 이때만 평면 가속을 쓴다.
    const bool planar = nCh == 1 && Abs(ChannelElevation(0)) < 1e-6f;

    // 메시마다 "센서 -> 메시 좌표" 변환을 한 번만 만든다. 광선을 메시 쪽으로 옮기면 정점을 월드로 옮길 필요가 없다.
    for (size_t k = 0; k < targetCount; k++) {
        const LidarTarget& tg = targets[k];
        if (!tg.positions || !tg.indices || tg.vertexCount == 0 || tg.indexCount < 3) continue;
        const dhMat M = tg.world.Inv() * sensorPose;
        const float o[3] = {M.v[12], M.v[13], M.v[14]};

        const bool useBvh = tg.bvh && !tg.bvh->empty();  // BVH 가 있으면 평면 가속과 경계 상자 대신 이것을 쓴다
        const uint32_t* triIdx = tg.indices;
        size_t triCount = tg.indexCount;
        simul::Array<uint32_t> near;
        if (planar && !useBvh) {
            // 평면(점 o, 법선 nrm = 센서의 z 축)의 한쪽에만 있는 삼각형은 어떤 광선도 맞출 수 없으므로,
            // 평면을 가로지르는 삼각형만 남긴다 (한 번에 O(정점 + 삼각형)).
            const float nrm[3] = {M.v[8], M.v[9], M.v[10]};
            const float eps = 1e-5f;
            Array<char> side(tg.vertexCount, 0);  // +1: 평면 위쪽, -1: 아래쪽, 0: 평면 위(오차 범위)
            for (size_t v = 0; v < tg.vertexCount; v++) {
                const float* p = tg.positions + v * tg.stride;
                const float dist = nrm[0] * (p[0] - o[0]) + nrm[1] * (p[1] - o[1]) + nrm[2] * (p[2] - o[2]);
                side[v] = dist > eps ? 1 : (dist < -eps ? -1 : 0);
            }
            for (size_t f = 0; f + 2 < tg.indexCount; f += 3) {
                const uint32_t a = tg.indices[f], b = tg.indices[f + 1], c = tg.indices[f + 2];
                if (a >= tg.vertexCount || b >= tg.vertexCount || c >= tg.vertexCount) continue;
                if (side[a] > 0 && side[b] > 0 && side[c] > 0) continue;
                if (side[a] < 0 && side[b] < 0 && side[c] < 0) continue;
                near.push_back(a);
                near.push_back(b);
                near.push_back(c);
            }
            if (near.empty()) continue;
            triIdx = near.data();
            triCount = near.size();
        }

        for (size_t i = 0; i < n; i++) {
            const float* sd = &dirs[i * 3];
            // 센서 방향을 메시 좌표로: 변환 M 의 회전 부분
            const float d[3] = {M.v[0] * sd[0] + M.v[4] * sd[1] + M.v[8] * sd[2], M.v[1] * sd[0] + M.v[5] * sd[1] + M.v[9] * sd[2],
                                M.v[2] * sd[0] + M.v[6] * sd[1] + M.v[10] * sd[2]};
            // 지금까지 찾은 가장 가까운 거리(또는 rangeMax)까지만 검사하면 먼 메시를 더 빨리 거른다
            const float limit = IsFinite(out->ranges[i]) ? out->ranges[i] : settings_.rangeMax;
            float t = 0.0f;
            if (useBvh) {
                if (!tg.bvh->Raycast(dhVector(o[0], o[1], o[2]), dhVector(d[0], d[1], d[2]), limit, &t)) continue;
            } else {
                if (tg.hasBounds && !SegmentHitsBox(o, d, limit, tg.boundsMin, tg.boundsMax)) continue;
                if (!RaycastTriangles(tg.positions, tg.vertexCount, triIdx, triCount, dhVector(o[0], o[1], o[2]), dhVector(d[0], d[1], d[2]), limit, &t,
                                      NULL, tg.stride))
                    continue;
            }
            if (t < out->ranges[i] || !IsFinite(out->ranges[i])) {
                out->ranges[i] = t;
                out->targetIds[i] = tg.id < 0 ? 0 : tg.id;  // id 가 음수면 0 (맞았음을 -1 과 구별하기 위해)
            }
        }
    }

    // 가장 가까운 면이 rangeMin 안쪽(사각지대)이면 그 광선은 무효다. 그 뒤의 물체가 대신 보이지는 않는다.
    // 그다음 잡음을 더하고, 맞은 점을 월드 좌표로 바꾼다.
    for (size_t i = 0; i < n; i++) {
        if (out->targetIds[i] < 0) continue;
        if (out->ranges[i] < settings_.rangeMin) {
            out->ranges[i] = HUGE_VALF;
            out->targetIds[i] = -1;
            continue;
        }
        float r = out->ranges[i];
        if (settings_.noiseStdDev > 0.0f) {
            r += settings_.noiseStdDev * gaussian();
            if (r < settings_.rangeMin) r = settings_.rangeMin;
            if (r > settings_.rangeMax) r = settings_.rangeMax;
            out->ranges[i] = r;
        }
        const float* sd = &dirs[i * 3];
        out->points[i] = sensorPose * dhVector(r * sd[0], r * sd[1], r * sd[2]);
    }
}

}  // namespace sensor
}  // namespace simul
