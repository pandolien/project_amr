// CMesh 테스트: 기본 도형, Scene 유틸리티, GLB 로더 (정상/오류/잘린 입력)
#include "CBase/File.h"
#include "CBase/Math.h"
#include <stdio.h>

#include "CMesh/MeshLoader.h"
#include "CMesh/Primitives.h"
#include "TestGlb.h"
#include "TestUtil.h"

using namespace simul;

namespace {

int g_fail = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

bool nearly(double a, double b, double eps = 1e-4) { return simul::Abs(a - b) <= eps; }

// 닫힌 메시의 부호 있는 부피 (바깥 방향 와인딩이면 양수)
double SignedVolume(const Primitive& p) {
    double vol = 0;
    for (size_t t = 0; t < p.TriangleCount(); t++) {
        const float* a = &p.positions[3 * p.indices[3 * t]];
        const float* b = &p.positions[3 * p.indices[3 * t + 1]];
        const float* c = &p.positions[3 * p.indices[3 * t + 2]];
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
                a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0;
    }
    return vol;
}

// 법선이 면의 바깥을 향하는지 (정점 법선과 삼각형 법선의 내적이 양수)
bool NormalsMatchWinding(const Primitive& p) {
    for (size_t t = 0; t < p.TriangleCount(); t++) {
        const float* a = &p.positions[3 * p.indices[3 * t]];
        const float* b = &p.positions[3 * p.indices[3 * t + 1]];
        const float* c = &p.positions[3 * p.indices[3 * t + 2]];
        const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
        const double vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
        const double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const float* n = &p.normals[3 * p.indices[3 * t]];
        if (nx * n[0] + ny * n[1] + nz * n[2] < -1e-6) return false;  // 극점의 퇴화 삼각형은 오차 허용
    }
    return true;
}

void testPrimitives() {
    const Primitive box = MakeBox(2, 4, 6);
    check(box.IsValid(), "box valid");
    check(box.TriangleCount() == 12, "box 12 triangles");
    check(nearly(SignedVolume(box), 48.0), "box volume");
    check(NormalsMatchWinding(box), "box normals outward");

    const Primitive cyl = MakeCylinder(1.0f, 2.0f, 64);
    check(cyl.IsValid(), "cylinder valid");
    check(nearly(SignedVolume(cyl), 3.14159265 * 2.0, 0.05), "cylinder volume (pi r^2 h)");
    check(NormalsMatchWinding(cyl), "cylinder normals outward");

    const Primitive sph = MakeSphere(1.0f, 48, 24);
    check(sph.IsValid(), "sphere valid");
    check(nearly(SignedVolume(sph), 4.0 / 3.0 * 3.14159265, 0.1), "sphere volume");
    check(NormalsMatchWinding(sph), "sphere normals outward");

    // 비정상 인자도 안전하게 보정
    check(MakeCylinder(1, 1, 0).IsValid(), "cylinder segments clamp");
    check(MakeSphere(1, 0, 0).IsValid(), "sphere segments clamp");
}

void testSceneUtil() {
    Scene s = SceneFromPrimitive(MakeBox(2, 2, 2), "box");
    check(s.IsValid(), "scene valid");
    dhVector mn, mx;
    check(s.ComputeBounds(&mn, &mx), "bounds");
    check(nearly(mn.x, -1) && nearly(mx.z, 1), "bounds values");

    s.ApplyRootTransform(dhMat::Trans(10, 0, 0));
    check(s.ComputeBounds(&mn, &mx) && nearly(mn.x, 9) && nearly(mx.x, 11), "root transform moves bounds");

    Scene empty;
    check(!empty.ComputeBounds(&mn, &mx), "empty bounds false");

    Scene bad = SceneFromPrimitive(MakeBox(1, 1, 1), "bad");
    bad.meshes[0].primitives[0].indices[0] = 9999;
    check(!bad.IsValid(), "out of range index invalid");
}

// 삼각형 하나: 위치 3개(36B) + 인덱스 3개
simul::String TriJson(const simul::String& indexType, const simul::String& extra, const simul::String& nodeExtra,
                    size_t binLen, const simul::String& primExtra = "", int indexCount = 3, const simul::String& ext = "") {
    char buf[2048];
    snprintf(buf, sizeof(buf),
        "{\"asset\":{\"version\":\"2.0\"},%s"
        "\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
        "\"nodes\":[{\"mesh\":0%s}],%s"
        "\"meshes\":[{\"name\":\"tri\",\"primitives\":[{\"attributes\":{\"POSITION\":0},\"indices\":1%s}]}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
        "\"min\":[0,0,0],\"max\":[1,1,0]},"
        "{\"bufferView\":1,\"componentType\":%s,\"count\":%d,\"type\":\"SCALAR\"}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},"
        "{\"buffer\":0,\"byteOffset\":36,\"byteLength\":%d}],"
        "\"buffers\":[{\"byteLength\":%d}]}",
        ext.c_str(), nodeExtra.c_str(), extra.c_str(), primExtra.c_str(), indexType.c_str(), indexCount,
        static_cast<int>(binLen - 36), static_cast<int>(binLen));
    return buf;
}

simul::Array<char> TriBin16() {
    simul::Array<char> b;
    const float p[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 9; i++) testglb::putf(&b, p[i]);
    testglb::putu16(&b, 0);
    testglb::putu16(&b, 1);
    testglb::putu16(&b, 2);
    return b;  // 36 + 6
}

void testGlbBasic() {
    const simul::Array<char> bin = TriBin16();
    const simul::Array<char> glb = testglb::MakeGlb(
        TriJson("5123", "\"materials\":[{\"name\":\"red\",\"doubleSided\":true,\"pbrMetallicRoughness\":"
                        "{\"baseColorFactor\":[1,0,0,1],\"metallicFactor\":0.25,\"roughnessFactor\":0.5}}],",
                ",\"translation\":[1,2,3]", bin.size(), ",\"material\":0"),
        bin);
    Scene s;
    simul::String err;
    check(LoadGlbFromMemory(&glb[0], glb.size(), &s, &err), "basic glb loads");
    if (!s.meshes.empty()) {
        check(s.TriangleCount() == 1, "1 triangle");
        check(s.meshes[0].primitives[0].normals.size() == 9, "normals computed");
        check(nearly(s.meshes[0].primitives[0].normals[2], 1.0), "computed normal is +Z");
        check(s.meshes[0].primitives[0].material == 0, "material index");
        check(s.materials.size() == 1 && nearly(s.materials[0].baseColor.r, 1) && nearly(s.materials[0].metallic, 0.25) &&
                  nearly(s.materials[0].roughness, 0.5) && s.materials[0].doubleSided,
              "material values");
        dhVector mn, mx;
        check(s.ComputeBounds(&mn, &mx) && nearly(mn.x, 1) && nearly(mn.y, 2) && nearly(mn.z, 3) &&
                  nearly(mx.x, 2) && nearly(mx.y, 3),
              "node translation applied to bounds");
    }
    check(s.IsValid(), "scene valid");

    // 파일 경로 버전
    const simul::String path = tu::Tmp("simul_test_tri.glb");
    {
        simul::WriteFileBytes(path, &glb[0], glb.size());
    }
    Scene s2;
    check(LoadGlb(path, &s2, &err) && s2.TriangleCount() == 1, "LoadGlb from file");
    // LoadMesh 확장자 분기는 STL/OBJ/DAE 로더 작성 후 test_cmesh 에서 검사

    remove(path.c_str());
    check(!LoadGlb(tu::Tmp("does_not_exist.glb"), &s2, &err) && !err.empty(), "missing file fails with message");
}

void testGlbIndexVariants() {
    // 인덱스 없음 (3 정점이 곧 삼각형 1개)
    simul::Array<char> b;
    const float p[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 9; i++) testglb::putf(&b, p[i]);
    simul::String json =
        "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"scene\":0,\"nodes\":[{\"mesh\":0}],"
        "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[1,1,0]}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],\"buffers\":[{\"byteLength\":36}]}";
    simul::Array<char> glb = testglb::MakeGlb(json, b);
    Scene s;
    simul::String err;
    check(LoadGlbFromMemory(&glb[0], glb.size(), &s, &err) && s.TriangleCount() == 1, "non-indexed primitive");

    // 삼각형 스트립 (사각형 = 정점 4, 삼각형 2) 과 팬
    for (int mode = 5; mode <= 6; mode++) {
        simul::Array<char> b4;
        const float q[12] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0};
        for (int i = 0; i < 12; i++) testglb::putf(&b4, q[i]);
        char js[1024];
        snprintf(js, sizeof(js),
            "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"scene\":0,\"nodes\":[{\"mesh\":0}],"
            "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"mode\":%d}]}],"
            "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[1,1,0]}],"
            "\"bufferViews\":[{\"buffer\":0,\"byteLength\":48}],\"buffers\":[{\"byteLength\":48}]}", mode);
        glb = testglb::MakeGlb(js, b4);
        Scene t;
        check(LoadGlbFromMemory(&glb[0], glb.size(), &t, &err) && t.TriangleCount() == 2,
              mode == 5 ? "triangle strip -> 2 triangles" : "triangle fan -> 2 triangles");
    }

    // 점(points, mode 0) 은 건너뛰고 경고
    {
        simul::String js2 =
            "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"scene\":0,\"nodes\":[{\"mesh\":0}],"
            "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"mode\":0}]}],"
            "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[1,1,0]}],"
            "\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],\"buffers\":[{\"byteLength\":36}]}";
        glb = testglb::MakeGlb(js2, b);
        Scene t;
        check(LoadGlbFromMemory(&glb[0], glb.size(), &t, &err) && t.TriangleCount() == 0 && !t.warnings.empty(),
              "points skipped with warning");
    }
}

void testGlbHierarchy() {
    // 부모(이동 10,0,0) -> 자식(Z축 90도 회전, 이동 1,0,0). 자식의 점 (1,0,0) 은 월드 (10,1,0)+... 아래에서 계산
    simul::Array<char> b = TriBin16();
    char js[2048];
    snprintf(js, sizeof(js),
        "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"scene\":0,"
        "\"nodes\":[{\"translation\":[10,0,0],\"children\":[1]},"
        "{\"mesh\":0,\"translation\":[1,0,0],\"rotation\":[0,0,0.70710678,0.70710678]}],"
        "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"indices\":1}]}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[1,1,0]},"
        "{\"bufferView\":1,\"componentType\":5123,\"count\":3,\"type\":\"SCALAR\"}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},{\"buffer\":0,\"byteOffset\":36,\"byteLength\":6}],"
        "\"buffers\":[{\"byteLength\":%d}]}", static_cast<int>(b.size()));
    simul::Array<char> glb = testglb::MakeGlb(js, b);
    Scene s;
    simul::String err;
    check(LoadGlbFromMemory(&glb[0], glb.size(), &s, &err), "hierarchy loads");
    check(s.nodes.size() == 2 && s.nodes[1].parent == 0 && s.nodes[0].children.size() == 1, "parent/child links");
    // 삼각형 (0,0,0),(1,0,0),(0,1,0) 을 Z 90도 회전 -> (0,0,0),(0,1,0),(-1,0,0), 이동 (11,0,0) -> x∈[10,11], y∈[0,1]
    dhVector mn, mx;
    check(s.ComputeBounds(&mn, &mx) && nearly(mn.x, 10) && nearly(mx.x, 11) && nearly(mn.y, 0) && nearly(mx.y, 1),
          "child rotation + parent translation composed");

    // Y-up -> Z-up 변환: (x,y,z) -> (x,-z,y). 점 (0,1,0) 이 (0,0,1) 로 가야 한다
    Scene t = s;
    t.ApplyRootTransform(dhMat::RotX(static_cast<float>(dhRad(90.0))));
    check(t.ComputeBounds(&mn, &mx) && nearly(mx.z, 1) && nearly(mn.z, 0) && nearly(mn.y, 0),
          "Y-up -> Z-up root transform");
}

void testGlbErrors() {
    Scene s;
    simul::String err;
    check(!LoadGlbFromMemory(NULL, 0, &s, &err) && !err.empty(), "null data");
    const char junk[] = "this is definitely not a glb file";
    check(!LoadGlbFromMemory(junk, sizeof(junk), &s, &err), "garbage rejected");
    check(!LoadGlbFromMemory(junk, sizeof(junk), &s, NULL), "garbage rejected without error pointer");

    // Draco 필수 확장
    simul::Array<char> bin = TriBin16();
    simul::Array<char> glb = testglb::MakeGlb(
        TriJson("5123", "", "", bin.size(), "", 3, "\"extensionsRequired\":[\"KHR_draco_mesh_compression\"],\"extensionsUsed\":[\"KHR_draco_mesh_compression\"],"),
        bin);
    check(!LoadGlbFromMemory(&glb[0], glb.size(), &s, &err), "draco rejected");
    check(s.nodes.empty() && s.meshes.empty(), "scene cleared after failure");

    // 인덱스가 정점 범위를 벗어남 (정점 3개, 인덱스 7)
    simul::Array<char> b2 = TriBin16();
    b2[36 + 4] = 7;
    glb = testglb::MakeGlb(TriJson("5123", "", "", b2.size()), b2);
    check(!LoadGlbFromMemory(&glb[0], glb.size(), &s, &err), "out of range index rejected");

    // 버퍼보다 큰 접근자
    glb = testglb::MakeGlb(TriJson("5123", "", "", bin.size(), "", 300), bin);
    check(!LoadGlbFromMemory(&glb[0], glb.size(), &s, &err), "accessor larger than buffer rejected");

    // 올바른 파일의 모든 접두 길이로 잘라도 죽지 않아야 한다
    glb = testglb::MakeGlb(TriJson("5123", "", ",\"translation\":[1,2,3]", bin.size()), bin);
    int accepted = 0;
    for (size_t n = 0; n < glb.size(); n++) {
        Scene t;
        if (LoadGlbFromMemory(&glb[0], n, &t, &err)) accepted++;
    }
    check(accepted == 0, "truncated prefixes are all rejected");

    // 바이트 하나씩 망가뜨려도 죽지 않아야 한다 (결과는 성공/실패 무관)
    for (size_t i = 0; i < glb.size(); i++) {
        simul::Array<char> c = glb;
        c[i] = static_cast<char>(c[i] ^ 0xFF);
        Scene t;
        LoadGlbFromMemory(&c[0], c.size(), &t, &err);
        if (!t.IsValid()) check(false, "loader returned invalid scene on corrupted input");
    }
}

}  // namespace

// 정점 색(COLOR_0)과 기본 색 텍스처(GLB 안, data URI, 외부 파일)를 읽는다
void testGlbColorsAndTextures() {
    simul::Array<char> b;
    const float p[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 9; i++) testglb::putf(&b, p[i]);                       // 0..35 POSITION
    const float c[12] = {1, 0, 0, 1, 0, 1, 0, 0.5f, 0, 0, 1, 1};
    for (int i = 0; i < 12; i++) testglb::putf(&b, c[i]);                      // 36..83 COLOR_0 (vec4)
    const float uv[6] = {0, 0, 1, 0, 0, 1};
    for (int i = 0; i < 6; i++) testglb::putf(&b, uv[i]);                      // 84..107 TEXCOORD_0
    const char img[8] = {'I', 'M', 'G', '1', '2', '3', '4', '5'};
    b.insert(b.end(), img, img + 8);                                           // 108..115 임베디드 이미지
    simul::String json =
        "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"scene\":0,\"nodes\":[{\"mesh\":0}],"
        "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0,\"COLOR_0\":1,\"TEXCOORD_0\":2},\"material\":0}]}],"
        "\"materials\":[{\"pbrMetallicRoughness\":{\"baseColorTexture\":{\"index\":0}}},"
        "{\"pbrMetallicRoughness\":{\"baseColorTexture\":{\"index\":1}}},"
        "{\"pbrMetallicRoughness\":{\"baseColorTexture\":{\"index\":2}},\"alphaMode\":\"MASK\",\"alphaCutoff\":0.3}],"
        "\"textures\":[{\"source\":0},{\"source\":1},{\"source\":2}],"
        "\"images\":[{\"bufferView\":3,\"mimeType\":\"image/png\"},{\"uri\":\"data:image/png;base64,QUJDRA==\"},{\"uri\":\"maps/tex%20a.png\"}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[1,1,0]},"
        "{\"bufferView\":1,\"componentType\":5126,\"count\":3,\"type\":\"VEC4\"},"
        "{\"bufferView\":2,\"componentType\":5126,\"count\":3,\"type\":\"VEC2\"}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},{\"buffer\":0,\"byteOffset\":36,\"byteLength\":48},"
        "{\"buffer\":0,\"byteOffset\":84,\"byteLength\":24},{\"buffer\":0,\"byteOffset\":108,\"byteLength\":8}],"
        "\"buffers\":[{\"byteLength\":116}]}";
    simul::Array<char> glb = testglb::MakeGlb(json, b);
    const simul::String path = tu::Tmp("simul_test_color.glb");
    simul::WriteFileBytes(path, &glb[0], glb.size());
    Scene s;
    simul::String err;
    check(LoadGlb(path, &s, &err), "glb with colors and textures loads");
    if (!s.meshes.empty() && !s.meshes[0].primitives.empty()) {
        const Primitive& pr = s.meshes[0].primitives[0];
        check(pr.colors.size() == 12 && nearly(pr.colors[0], 1) && nearly(pr.colors[7], 0.5) && nearly(pr.colors[10], 1), "COLOR_0 read as rgba");
        check(pr.uvs.size() == 6 && nearly(pr.uvs[2], 1), "uv read");
    }
    check(s.IsValid() && s.materials.size() == 3, "scene with colors valid");
    check(s.materials.size() == 3 && s.materials[2].alphaMode == 1 && nearly(s.materials[2].alphaCutoff, 0.3), "MASK alpha mode and cutoff read");
    check(s.materials.size() == 3 && s.materials[0].alphaMode == 0 && nearly(s.materials[0].alphaCutoff, 0.5), "alpha mode defaults to opaque");
    if (s.materials.size() == 3) {
        check(s.materials[0].baseColorImage.size() == 8 && s.materials[0].baseColorImage[0] == 'I' && s.materials[0].baseColorTexture.empty(), "embedded image bytes kept");
        check(s.materials[1].baseColorImage.size() == 4 && s.materials[1].baseColorImage[0] == 'A' && s.materials[1].baseColorImage[3] == 'D', "data URI decoded");
        const simul::String& t = s.materials[2].baseColorTexture;
        check(t.size() > 10 && t.find("maps/tex a.png") != simul::String::npos && t.find(tu::Tmp("")) == 0, "external texture path resolved next to the file");
    }
    remove(path.c_str());
}

// 같은 파일을 다시 읽으면 캐시에서 주고, 파일이 바뀌면 다시 읽는다
void testMeshCache() {
    ClearMeshCache();
    const simul::String path = tu::Tmp("simul_test_cache.obj");
    const char* one = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
    const char* two = "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\nf 1 2 3\nf 2 4 3\n";
    simul::WriteFileBytes(path, one, strlen(one));
    Scene a, b;
    simul::String err;
    check(LoadMeshCached(path, &a, &err) && a.TriangleCount() == 1, "cached load reads the file");
    check(LoadMeshCached(path, &b, &err) && b.TriangleCount() == 1, "second cached load works");
    b.meshes.clear();   // 돌려준 것을 고쳐도 캐시는 그대로다
    Scene c;
    check(LoadMeshCached(path, &c, &err) && c.TriangleCount() == 1, "editing the returned copy does not change the cache");
    check(LoadMeshCached(path, nullptr, &err), "null out only warms the cache");
    simul::WriteFileBytes(path, two, strlen(two));   // 크기가 달라져 옛 항목은 쓰지 않는다
    Scene d;
    check(LoadMeshCached(path, &d, &err) && d.TriangleCount() == 2, "changed file is read again");
    check(!LoadMeshCached(tu::Tmp("simul_missing_cache.obj"), &d, &err) && !err.empty(), "missing file fails through the cache");
    remove(path.c_str());
    ClearMeshCache();
}

// 스킨이 붙은 메시는 노드 변환을 무시한다 (뼈대 변형은 없이 기본 자세). NaN 좌표는 0 으로 바꾼다.
void testGlbSkinAndNan() {
    simul::Array<char> b;
    const float p[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 9; i++) testglb::putf(&b, p[i]);
    simul::String json =
        "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0,1]}],\"scene\":0,"
        "\"nodes\":[{\"mesh\":0,\"skin\":0,\"translation\":[100,0,0],\"scale\":[5,5,5]},{\"name\":\"bone\"}],"
        "\"skins\":[{\"joints\":[1]}],"
        "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[1,1,0]}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],\"buffers\":[{\"byteLength\":36}]}";
    simul::Array<char> glb = testglb::MakeGlb(json, b);
    Scene s;
    simul::String err;
    check(LoadGlbFromMemory(&glb[0], glb.size(), &s, &err), "skinned glb loads");
    dhVector mn, mx;
    check(s.ComputeBounds(&mn, &mx) && nearly(mn.x, 0) && nearly(mx.x, 1) && nearly(mx.y, 1), "skinned mesh ignores its node transform");
    bool warned = false;
    for (size_t i = 0; i < s.warnings.size(); i++) warned = warned || s.warnings[i].find("스킨") != simul::String::npos;
    check(warned, "skin warning is recorded");
    check(s.IsValid(), "scene with detached skinned node is valid");

    simul::Array<char> b2;
    const float q[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    for (int i = 0; i < 9; i++) testglb::putf(&b2, i == 4 ? 0.0f / 0.0f : q[i]);
    simul::String js2 =
        "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"scene\":0,\"nodes\":[{\"mesh\":0}],"
        "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}],"
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"}],"
        "\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],\"buffers\":[{\"byteLength\":36}]}";
    glb = testglb::MakeGlb(js2, b2);
    Scene s2;
    check(LoadGlbFromMemory(&glb[0], glb.size(), &s2, &err) && s2.ComputeBounds(&mn, &mx) && mx.x < 10 && mn.y > -10, "NaN coordinate replaced so bounds stay finite");
}

int main() {
    testPrimitives();
    testSceneUtil();
    testGlbBasic();
    testGlbIndexVariants();
    testGlbHierarchy();
    testGlbErrors();
    testGlbColorsAndTextures();
    testMeshCache();
    testGlbSkinAndNan();
    if (g_fail == 0) printf("test_cmesh: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
