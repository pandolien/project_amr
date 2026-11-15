// STL / OBJ / COLLADA 로더 테스트 (정상, 오류, 잘린/손상 입력)
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>

#include "CMesh/MeshLoader.h"
#include "TestGlb.h"
#include "TestUtil.h"

using namespace amr;

namespace {

int g_fail = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

bool nearly(double a, double b, double eps = 1e-4) { return std::fabs(a - b) <= eps; }

void writeFile(const std::string& path, const std::string& s) {
    std::ofstream f(path.c_str(), std::ios::binary);
    f.write(s.data(), static_cast<std::streamsize>(s.size()));
}

// ---------------- STL ----------------
std::vector<char> BinaryStl(const float* tris, uint32_t count) {
    std::vector<char> b(80, 0);
    testglb::put32(&b, count);
    for (uint32_t t = 0; t < count; t++) {
        for (int k = 0; k < 3; k++) testglb::putf(&b, 0.0f);  // 법선(무시됨)
        for (int k = 0; k < 9; k++) testglb::putf(&b, tris[t * 9 + static_cast<uint32_t>(k)]);
        testglb::putu16(&b, 0);
    }
    return b;
}

void testStl() {
    const float tri[9] = {0, 0, 0, 2, 0, 0, 0, 3, 0};
    std::vector<char> bin = BinaryStl(tri, 1);
    Scene s;
    std::string err;
    check(LoadStlFromMemory(&bin[0], bin.size(), &s, &err), "binary stl");
    check(s.TriangleCount() == 1 && s.IsValid(), "binary stl 1 triangle");
    dhVector mn, mx;
    check(s.ComputeBounds(&mn, &mx) && nearly(mx.x, 2) && nearly(mx.y, 3), "binary stl bounds");
    check(nearly(s.meshes[0].primitives[0].normals[2], 1.0), "normal recomputed (+Z)");

    // "solid" 로 시작하는 헤더를 가진 바이너리도 크기로 판별한다
    std::vector<char> tricky = bin;
    std::memcpy(&tricky[0], "solid fake binary header", 24);
    check(LoadStlFromMemory(&tricky[0], tricky.size(), &s, &err) && s.TriangleCount() == 1, "binary with 'solid' header");

    const std::string ascii =
        "solid t\n facet normal 0 0 1\n  outer loop\n   vertex 0 0 0\n   vertex 1.5e0 0 0\n   vertex 0 1 0\n"
        "  endloop\n endfacet\nendsolid t\n";
    check(LoadStlFromMemory(ascii.data(), ascii.size(), &s, &err) && s.TriangleCount() == 1, "ascii stl");
    check(s.ComputeBounds(&mn, &mx) && nearly(mx.x, 1.5), "ascii exponent parsed");

    const std::string badAscii = "solid t\n vertex 0 0 0\n vertex 1 0 0\n endsolid";
    check(!LoadStlFromMemory(badAscii.data(), badAscii.size(), &s, &err), "ascii with 2 vertices rejected");
    const std::string notStl = "hello world, not an stl";
    check(!LoadStlFromMemory(notStl.data(), notStl.size(), &s, &err), "non-stl rejected");

    // 바이너리 크기 불일치(잘림)
    std::vector<char> cut(bin.begin(), bin.end() - 1);
    check(!LoadStlFromMemory(&cut[0], cut.size(), &s, &err), "truncated binary stl rejected");

    // NaN 좌표
    float nan[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    nan[4] = std::nanf("");
    std::vector<char> nb = BinaryStl(nan, 1);
    check(!LoadStlFromMemory(&nb[0], nb.size(), &s, &err), "NaN coordinate rejected");

    // 모든 접두 길이 / 모든 바이트 손상에도 죽지 않는다
    for (size_t n = 0; n < bin.size(); n++) {
        Scene t;
        if (LoadStlFromMemory(&bin[0], n, &t, &err) && n != bin.size()) check(false, "stl prefix accepted");
    }
    check(!LoadStl(tu::Tmp("amr_missing.stl"), &s, &err) && !err.empty(), "missing stl file");
    writeFile(tu::Tmp("amr_t.stl"), std::string(bin.begin(), bin.end()));
    check(LoadMesh(tu::Tmp("amr_t.stl"), &s, &err) && s.TriangleCount() == 1, "LoadMesh dispatches .stl");
    std::remove(tu::Tmp("amr_t.stl").c_str());
}

// ---------------- OBJ ----------------
void testObj() {
    Scene s;
    std::string err;
    // 사각형 -> 삼각형 2개, 음수 인덱스, 슬래시 형식 혼합
    const std::string quad =
        "# comment\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n"
        "f 1/1/1 2/2/1 3/3/1 4/4/1\n";
    check(LoadObjFromString(quad, &s, &err), "obj quad");
    check(s.TriangleCount() == 2 && s.meshes[0].primitives[0].VertexCount() == 4, "quad -> 2 triangles, 4 shared vertices");
    check(s.meshes[0].primitives[0].uvs.size() == 8, "uv kept");

    const std::string neg = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n";
    check(LoadObjFromString(neg, &s, &err) && s.TriangleCount() == 1, "negative indices");
    check(nearly(s.meshes[0].primitives[0].normals[2], 1.0), "normals computed when absent");

    // 재질별 primitive 분리
    const std::string mats =
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nusemtl red\nf 1 2 3\nusemtl blue\nf 1 3 4\nusemtl red\nf 1 2 4\n";
    check(LoadObjFromString(mats, &s, &err), "obj materials");
    check(s.meshes[0].primitives.size() == 2 && s.materials.size() == 2, "2 groups for 2 materials");
    check(s.TriangleCount() == 3, "3 triangles total");

    // 법선이 없는 상자: 면마다 정점을 따로 만들어 평평한 음영 (정점 24개, 법선은 축 방향)
    const std::string cube =
        "v 0 0 0\nv 2 0 0\nv 2 1 0\nv 0 1 0\nv 0 0 1\nv 2 0 1\nv 2 1 1\nv 0 1 1\n"
        "f 1 4 3 2\nf 5 6 7 8\nf 1 2 6 5\nf 2 3 7 6\nf 3 4 8 7\nf 4 1 5 8\n";
    check(LoadObjFromString(cube, &s, &err) && s.TriangleCount() == 12, "obj cube without normals");
    if (!s.meshes.empty()) {
        const Primitive& cp = s.meshes[0].primitives[0];
        bool axisAligned = true;
        for (size_t v = 0; v < cp.VertexCount(); v++) {
            int big = 0;
            for (int k = 0; k < 3; k++) if (nearly(std::fabs(cp.normals[v * 3 + static_cast<size_t>(k)]), 1.0)) big++;
            if (big != 1) axisAligned = false;
        }
        check(cp.VertexCount() == 24 && axisAligned, "obj without normals is flat shaded (24 vertices, axis-aligned normals)");
    }

    check(!LoadObjFromString("v 0 0 0\nv 1 0 0\nf 1 2 9\n", &s, &err), "out of range face index");
    check(!LoadObjFromString("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 x\n", &s, &err), "non-numeric face index");
    check(!LoadObjFromString("v 0 0\n", &s, &err), "bad v line");
    check(!LoadObjFromString("v 0 0 0\n", &s, &err), "no faces");
    check(!LoadObjFromString("", &s, &err), "empty obj");

    // 파일 + MTL
    writeFile(tu::Tmp("amr_t.mtl"), "newmtl red\nKd 1 0 0\nd 0.5\nmap_Kd tex.png\n");
    writeFile(tu::Tmp("amr_t.obj"), "mtllib amr_t.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nusemtl red\nf 1 2 3\n");
    check(LoadObj(tu::Tmp("amr_t.obj"), &s, &err) && s.materials.size() == 1, "obj file with mtl");
    check(nearly(s.materials[0].baseColor.r, 1) && nearly(s.materials[0].baseColor.g, 0) && nearly(s.materials[0].baseColor.a, 0.5),
          "mtl Kd/d applied");
    check(s.materials[0].baseColorTexture == "tex.png", "mtl map_Kd");
    std::remove(tu::Tmp("amr_t.mtl").c_str());
    check(LoadObj(tu::Tmp("amr_t.obj"), &s, &err) && !s.warnings.empty(), "missing mtl -> warning, still loads");
    check(LoadMesh(tu::Tmp("amr_t.obj"), &s, &err) && s.TriangleCount() == 1, "LoadMesh dispatches .obj");
    std::remove(tu::Tmp("amr_t.obj").c_str());

    // 손상: 모든 접두 길이
    for (size_t n = 0; n <= quad.size(); n++) {
        Scene t;
        LoadObjFromString(quad.substr(0, n), &t, &err);
        if (!t.IsValid()) check(false, "obj prefix produced invalid scene");
    }
}

// ---------------- DAE ----------------
void replaceStr(std::string* s, const std::string& from, const std::string& to) {
    const size_t p = s->find(from);
    if (p == std::string::npos) {
        std::printf("FAIL: test setup, pattern not found: %s\n", from.c_str());
        ++g_fail;
        return;
    }
    s->replace(p, from.size(), to);
}

std::string DaeTri(const std::string& unit, const std::string& up, const std::string& nodeXform) {
    return "<?xml version=\"1.0\"?><COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">"
           "<asset><unit meter=\"" + unit + "\"/><up_axis>" + up + "</up_axis></asset>"
           "<library_effects><effect id=\"e1\"><profile_COMMON><technique sid=\"c\"><phong><diffuse><color>0 1 0 1</color></diffuse></phong></technique></profile_COMMON></effect></library_effects>"
           "<library_materials><material id=\"m1\"><instance_effect url=\"#e1\"/></material></library_materials>"
           "<library_geometries><geometry id=\"g1\" name=\"tri\"><mesh>"
           "<source id=\"pos\"><float_array id=\"pa\" count=\"9\">0 0 0 1 0 0 0 1 0</float_array>"
           "<technique_common><accessor source=\"#pa\" count=\"3\" stride=\"3\"><param name=\"X\" type=\"float\"/><param name=\"Y\" type=\"float\"/><param name=\"Z\" type=\"float\"/></accessor></technique_common></source>"
           "<vertices id=\"v\"><input semantic=\"POSITION\" source=\"#pos\"/></vertices>"
           "<triangles material=\"mat\" count=\"1\"><input semantic=\"VERTEX\" source=\"#v\" offset=\"0\"/><p>0 1 2</p></triangles>"
           "</mesh></geometry></library_geometries>"
           "<library_visual_scenes><visual_scene id=\"vs\"><node id=\"n\" name=\"n\">" + nodeXform +
           "<instance_geometry url=\"#g1\"><bind_material><technique_common><instance_material symbol=\"mat\" target=\"#m1\"/></technique_common></bind_material></instance_geometry>"
           "</node></visual_scene></library_visual_scenes>"
           "<scene><instance_visual_scene url=\"#vs\"/></scene></COLLADA>";
}

void testDae() {
    Scene s;
    std::string err;
    dhVector mn, mx;

    check(LoadDaeFromString(DaeTri("1", "Z_UP", ""), &s, &err), "dae basic");
    check(s.TriangleCount() == 1 && s.IsValid(), "dae 1 triangle");
    check(s.materials.size() == 1 && nearly(s.materials[0].baseColor.g, 1) && s.meshes[0].primitives[0].material == 0,
          "dae material binding + diffuse color");
    check(s.ComputeBounds(&mn, &mx) && nearly(mx.x, 1) && nearly(mx.y, 1) && nearly(mx.z, 0), "Z_UP unchanged");

    // 단위: 1 단위 = 0.001 m  -> x 최대 0.001
    check(LoadDaeFromString(DaeTri("0.001", "Z_UP", ""), &s, &err) && s.ComputeBounds(&mn, &mx) && nearly(mx.x, 0.001, 1e-6),
          "unit meter scale applied");

    // Y_UP: (x,y,z) -> (x,-z,y). 점 (0,1,0) -> (0,0,1)
    check(LoadDaeFromString(DaeTri("1", "Y_UP", ""), &s, &err) && s.ComputeBounds(&mn, &mx) && nearly(mx.z, 1) && nearly(mx.y, 0),
          "Y_UP -> Z_UP");
    // up_axis 가 없으면 기본이 Y_UP
    std::string noUp = DaeTri("1", "Z_UP", "");
    const size_t p = noUp.find("<up_axis>Z_UP</up_axis>");
    noUp.erase(p, std::string("<up_axis>Z_UP</up_axis>").size());
    check(LoadDaeFromString(noUp, &s, &err) && s.ComputeBounds(&mn, &mx) && nearly(mx.z, 1), "default up_axis is Y_UP");
    // X_UP: x 축이 위. 점 (1,0,0) -> (0,0,1)
    check(LoadDaeFromString(DaeTri("1", "X_UP", ""), &s, &err) && s.ComputeBounds(&mn, &mx) && nearly(mx.z, 1) && nearly(mn.z, 0),
          "X_UP -> Z_UP");

    // 노드 변환: translate 후 rotate (문서 순서대로 누적) -> T * R
    check(LoadDaeFromString(DaeTri("1", "Z_UP", "<translate>10 0 0</translate><rotate>0 0 1 90</rotate>"), &s, &err) &&
              s.ComputeBounds(&mn, &mx) && nearly(mn.x, 9) && nearly(mx.x, 10) && nearly(mx.y, 1),
          "translate then rotate order");
    // 행 우선 matrix: 이동 (1,2,3)
    check(LoadDaeFromString(DaeTri("1", "Z_UP", "<matrix>1 0 0 1 0 1 0 2 0 0 1 3 0 0 0 1</matrix>"), &s, &err) &&
              s.ComputeBounds(&mn, &mx) && nearly(mn.x, 1) && nearly(mn.y, 2) && nearly(mn.z, 3),
          "row-major matrix translation");
    check(LoadDaeFromString(DaeTri("1", "Z_UP", "<scale>2 2 2</scale>"), &s, &err) && s.ComputeBounds(&mn, &mx) && nearly(mx.x, 2),
          "scale");

    // polylist 사각형 -> 삼각형 2개
    std::string poly = DaeTri("1", "Z_UP", "");
    replaceStr(&poly, "<float_array id=\"pa\" count=\"9\">0 0 0 1 0 0 0 1 0</float_array>",
               "<float_array id=\"pa\" count=\"12\">0 0 0 1 0 0 1 1 0 0 1 0</float_array>");
    replaceStr(&poly, "count=\"3\" stride", "count=\"4\" stride");
    replaceStr(&poly, "<triangles material=\"mat\" count=\"1\">", "<polylist material=\"mat\" count=\"1\">");
    replaceStr(&poly, "<p>0 1 2</p></triangles>", "<vcount>4</vcount><p>0 1 2 3</p></polylist>");
    check(LoadDaeFromString(poly, &s, &err) && s.TriangleCount() == 2, "polylist quad -> 2 triangles");

    // 오류들
    check(!LoadDaeFromString("<notxml", &s, &err), "invalid xml");
    check(!LoadDaeFromString("<?xml version=\"1.0\"?><root/>", &s, &err), "not COLLADA");
    check(!LoadDaeFromString("", &s, &err), "empty");
    std::string badIdx = DaeTri("1", "Z_UP", "");
    replaceStr(&badIdx, "<p>0 1 2</p>", "<p>0 1 9</p>");
    check(!LoadDaeFromString(badIdx, &s, &err), "index out of source range rejected");
    std::string shortArr = DaeTri("1", "Z_UP", "");
    replaceStr(&shortArr, "count=\"9\">", "count=\"99\">");
    check(!LoadDaeFromString(shortArr, &s, &err), "float_array shorter than count rejected");

    // instance_node 순환 참조는 깊이 제한으로 중단
    const std::string cyc =
        "<COLLADA><library_nodes><node id=\"a\"><instance_node url=\"#a\"/></node></library_nodes>"
        "<library_visual_scenes><visual_scene id=\"vs\"><node id=\"r\"><instance_node url=\"#a\"/></node></visual_scene></library_visual_scenes>"
        "<scene><instance_visual_scene url=\"#vs\"/></scene></COLLADA>";
    check(!LoadDaeFromString(cyc, &s, &err), "cyclic instance_node rejected");

    // 파일 + LoadMesh
    writeFile(tu::Tmp("amr_t.DAE"), DaeTri("1", "Z_UP", ""));
    check(LoadMesh(tu::Tmp("amr_t.DAE"), &s, &err) && s.TriangleCount() == 1, "LoadMesh dispatches .DAE (case-insensitive)");
    std::remove(tu::Tmp("amr_t.DAE").c_str());
    check(!LoadMesh(tu::Tmp("model.xyz"), &s, &err) && !err.empty(), "unknown extension rejected");

    // 접두 길이 / 바이트 손상 반복 (죽지 않고 유효한 Scene 만 반환)
    const std::string good = DaeTri("1", "Z_UP", "<translate>1 2 3</translate>");
    for (size_t n = 0; n < good.size(); n += 3) {
        Scene t;
        LoadDaeFromString(good.substr(0, n), &t, &err);
        if (!t.IsValid()) check(false, "dae prefix produced invalid scene");
    }
    for (size_t i = 0; i < good.size(); i += 5) {
        std::string c = good;
        c[i] = static_cast<char>(c[i] ^ 0x55);
        Scene t;
        LoadDaeFromString(c, &t, &err);
        if (!t.IsValid()) check(false, "dae corruption produced invalid scene");
    }
}

}  // namespace

int main() {
    testStl();
    testObj();
    testDae();
    if (g_fail == 0) std::printf("test_cmesh_formats: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
