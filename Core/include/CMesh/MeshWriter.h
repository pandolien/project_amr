#ifndef CORE_CMESH_MESHWRITER_H
#define CORE_CMESH_MESHWRITER_H

#include "CBase/Array.h"
#include "CBase/String.h"

#include "CMesh/Mesh.h"

// 메시 파일 쓰기. 지금은 바이너리 STL 만 있다.
namespace simul {

// 장면 전체(모든 노드의 월드 변환을 적용한 삼각형)를 바이너리 STL 바이트로 만든다.
// 80바이트 헤더, 삼각형 수(4바이트), 삼각형마다 50바이트(면 법선, 정점 3개, 2바이트 0). 리틀 엔디언.
// 면 법선은 정점 순서(반시계 = 바깥)에서 계산한다. 색, 텍스처, 재질은 STL 에 담을 수 없어 버린다.
// 삼각형이 하나도 없으면 false.
bool StlBytes(const Scene& scene, Array<unsigned char>* out);
bool SaveStl(const Scene& scene, const String& path, String* error);

}  // namespace simul

#endif  // CORE_CMESH_MESHWRITER_H
