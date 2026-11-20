#ifndef CORE_CMESH_MESHWRITER_H
#define CORE_CMESH_MESHWRITER_H

#include "CBase/Array.h"
#include "CBase/String.h"

#include "CMesh/Mesh.h"

// 메시 파일 쓰기: 바이너리 STL, OBJ(+MTL, 텍스처 이미지).
namespace simul {

// 장면 전체(모든 노드의 월드 변환을 적용한 삼각형)를 바이너리 STL 바이트로 만든다.
// 80바이트 헤더, 삼각형 수(4바이트), 삼각형마다 50바이트(면 법선, 정점 3개, 2바이트 0). 리틀 엔디언.
// 면 법선은 정점 순서(반시계 = 바깥)에서 계산한다. 색, 텍스처, 재질은 STL 에 담을 수 없어 버린다.
// 삼각형이 하나도 없으면 false.
bool StlBytes(const Scene& scene, Array<unsigned char>* out);
bool SaveStl(const Scene& scene, const String& path, String* error);

// OBJ + MTL + 텍스처 이미지. path 는 .obj 파일이고 같은 폴더에 같은 이름의 .mtl 을 쓴다. 텍스처 이미지는 <이름>_tex<번호>.png/.jpg 로 같은 폴더에 둔다
// (GLB 안에 들어 있던 이미지는 꺼내 쓰고, 외부 파일은 복사한다).
//   - 모든 노드의 월드 변환을 정점에 적용하고, 재질별로 usemtl 을 나눈다. 법선과 uv(v 는 OBJ 규약으로 1 - v)를 쓴다.
//   - 재질은 Kd(기본 색), d(투명), map_Kd(텍스처)만 쓴다. 금속성/거칠기는 OBJ 에 없어 버린다.
//   - 정점 색은 "v x y z r g b" 확장으로 쓴다 (알파는 버린다). 우리 로더와 MeshLab, Blender 가 읽는다.
// 못 옮긴 것(알 수 없는 이미지 형식 등)은 warnings 에 쓴다. 삼각형이 없으면 false.
bool SaveObj(const Scene& scene, const String& path, String* error, Array<String>* warnings = 0);

}  // namespace simul

#endif  // CORE_CMESH_MESHWRITER_H
