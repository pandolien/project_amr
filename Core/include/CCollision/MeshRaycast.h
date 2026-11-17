#ifndef CORE_CCOLLISION_MESHRAYCAST_H
#define CORE_CCOLLISION_MESHRAYCAST_H

#include <stddef.h>
#include <stdint.h>

#include "3d/dhVector.h"

// 광선과 삼각형 메시의 교차 (화면에서 마우스로 물체를 고를 때 쓴다). 양면 모두 맞는다.
namespace simul {

// positions: 정점마다 xyz 가 stride 개(float)씩 간격으로 놓인 배열 (기본 3 = 빽빽한 xyz 배열. 위치+법선이 섞인 배열이면 6),
// indices: 삼각형 인덱스 (3개씩, indexCount 는 3 의 배수).
// 광선은 origin + t * dir (t >= 0, t <= tMax). dir 은 길이 1 일 필요 없고 tHit 는 dir 길이 단위를 따른다.
// 맞으면 true, 가장 가까운 교차의 t 를 tHit 에, 그 삼각형 번호를 triangle 에 (null 가능).
// 인덱스가 정점 범위를 벗어난 삼각형, 퇴화(면적 0) 삼각형, NaN 은 건너뛴다.
bool RaycastTriangles(const float* positions, size_t vertexCount, const uint32_t* indices, size_t indexCount,
                      const dhVector& origin, const dhVector& dir, float tMax, float* tHit, size_t* triangle = nullptr, size_t stride = 3);

}  // namespace simul

#endif  // CORE_CCOLLISION_MESHRAYCAST_H
