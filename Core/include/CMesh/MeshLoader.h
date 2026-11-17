#ifndef CORE_CMESH_MESHLOADER_H
#define CORE_CMESH_MESHLOADER_H

#include <stddef.h>

#include "CBase/String.h"

#include "CMesh/Mesh.h"

// 3D 파일 로더. 모든 함수는 성공하면 true, 실패하면 false 를 반환하고
// error(NULL 가능)에 원인을 적는다. 실패한 경우 *out 의 내용은 비어 있다.
// 지원 형식: GLB / glTF, STL(ASCII, 바이너리), OBJ(+MTL), COLLADA(DAE)
namespace simul {

// ---- GLB / glTF 2.0 (.glb, .gltf) : cgltf 사용
//   - 삼각형, 삼각형 스트립, 삼각형 팬을 읽는다 (점/선은 건너뛰고 Scene::warnings 에 기록)
//   - 8/16/32비트 인덱스, 인터리브(byteStride), 정규화(normalized) 속성, sparse 접근자 지원
//   - 법선이 없으면 계산하고, UV 와 재질(baseColorFactor, metallic, roughness)을 읽는다
//   - Draco, meshopt 압축 파일은 지원하지 않는다. 스킨/애니메이션은 무시한다.
bool LoadGlb(const simul::String& path, Scene* out, simul::String* error);
bool LoadGlbFromMemory(const void* data, size_t size, Scene* out, simul::String* error);

// ---- STL (바이너리/ASCII 자동 판별). 면 법선은 파일 값이 아니라 정점에서 다시 계산한다.
bool LoadStl(const simul::String& path, Scene* out, simul::String* error);
bool LoadStlFromMemory(const void* data, size_t size, Scene* out, simul::String* error);

// ---- OBJ. LoadObj 는 mtllib 이 가리키는 .mtl 을 같은 폴더에서 읽는다 (없으면 경고).
bool LoadObj(const simul::String& path, Scene* out, simul::String* error);
bool LoadObjFromString(const simul::String& text, Scene* out, simul::String* error);

// ---- COLLADA (.dae). unit 은 미터 기준으로 환산하고, up_axis 는 Z-up 으로 변환한다.
bool LoadDae(const simul::String& path, Scene* out, simul::String* error);
bool LoadDaeFromString(const simul::String& xml, Scene* out, simul::String* error);

// 확장자(.glb .gltf .stl .obj .dae, 대소문자 무시)로 형식을 골라 읽는다
bool LoadMesh(const simul::String& path, Scene* out, simul::String* error);

}  // namespace simul

#endif  // CORE_CMESH_MESHLOADER_H
