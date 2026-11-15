#ifndef CORE_CMESH_PRIMITIVES_H
#define CORE_CMESH_PRIMITIVES_H

#include <string>

#include "CMesh/Mesh.h"

// 기본 도형 생성 (URDF 의 box / cylinder / sphere 용). 모두 원점이 중심이고 면이 바깥을 향한다.
namespace amr {

Primitive MakeBox(float sx, float sy, float sz);                         // 각 변의 전체 길이
Primitive MakeCylinder(float radius, float length, int segments = 32);   // 축이 Z (URDF 규약)
Primitive MakeSphere(float radius, int segments = 32, int rings = 16);   // 극이 ±Z

// Primitive 하나를 노드 하나짜리 Scene 으로 감싼다
Scene SceneFromPrimitive(const Primitive& p, const std::string& name);

}  // namespace amr

#endif  // CORE_CMESH_PRIMITIVES_H
