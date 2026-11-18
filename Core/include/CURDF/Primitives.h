#ifndef CORE_CURDF_PRIMITIVES_H
#define CORE_CURDF_PRIMITIVES_H

#include "CBase/String.h"
#include "CMesh/Mesh.h"
#include "CURDF/Urdf.h"

// 파일 없이 기본 도형(박스, 원통, 구)을 URDF 로봇 한 개(링크 하나)로 만든다. World/ModelViewer 에 모델로 올려서 쓴다.
// 도형의 중심이 링크 원점이다 (박스는 가로 x 세로 x 높이, 원통은 축이 Z, 구는 반지름). 질량 mass(kg) 로 관성 텐서를 계산해 넣고,
// visual 과 collision 에 같은 도형을 쓴다. 색은 visual 재질로 들어간다.
namespace simul {
namespace urdf {

struct PrimitiveSpec {
    GeometryType type;     // kGeomBox, kGeomCylinder, kGeomSphere
    float size[3];         // 박스: 전체 변 길이 x, y, z (m)
    float radius, length;  // 원통: 반지름, 길이 / 구: 반지름
    float mass;            // kg. 0 이하면 질량 없음 (<inertial> 을 넣지 않는다)
    Color color;
    PrimitiveSpec() : type(kGeomBox), radius(0.25f), length(0.5f), mass(1.0f), color(0.6f, 0.65f, 0.75f, 1.0f) {
        size[0] = size[1] = size[2] = 0.5f;
    }
};

// 값이 올바르지 않으면(크기가 0 이하 등) false 와 error. 성공하면 FinalizeRobot 까지 끝난 Robot 을 돌려준다.
bool MakePrimitiveRobot(const simul::String& name, const PrimitiveSpec& spec, Robot* out, simul::String* error);

}  // namespace urdf
}  // namespace simul

#endif  // CORE_CURDF_PRIMITIVES_H
