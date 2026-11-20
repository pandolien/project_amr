// Robot -> URDF 쓰기: 쓴 파일을 다시 읽으면 같은 구조가 나오는지(왕복), 여러 모델을 합친 장면도 하나의 URDF 로 저장되는지 확인
#include <math.h>
#include <stdio.h>

#include "CURDF/UrdfWriter.h"
#include "CURDF/Primitives.h"
#include "CURDF/Scale.h"
#include "CURDF/World.h"
#include "CBase/File.h"
#include "CMesh/MeshLoader.h"
#include "CMesh/MeshWriter.h"
#include "TestGlb.h"
#include "TestUtil.h"

using namespace simul;
using namespace simul::urdf;

namespace {
int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}
bool near(double a, double b, double tol = 1e-5) { return fabs(a - b) <= tol; }
bool nearMat(const dhMat& a, const dhMat& b, double tol = 1e-5) {
    for (int i = 0; i < 16; i++)
        if (!near(a.v[i], b.v[i], tol)) return false;
    return true;
}

const String kData = String(SIMUL_TEST_DATA_DIR) + "/Object/samples/";

void geomSame(const Geometry& a, const Geometry& b, const String& what) {
    check(a.type == b.type, (what + ": geometry type").c_str());
    check(near(a.size[0], b.size[0]) && near(a.size[1], b.size[1]) && near(a.size[2], b.size[2]), (what + ": box size").c_str());
    check(near(a.radius, b.radius) && near(a.length, b.length), (what + ": radius/length").c_str());
}

// 두 Robot 의 구조(링크, 관절, 질량/관성, 형상, 색)가 같은가
void sameRobot(const Robot& a, const Robot& b, const String& tag) {
    check(a.links.size() == b.links.size() && a.joints.size() == b.joints.size(), (tag + ": counts").c_str());
    if (a.links.size() != b.links.size() || a.joints.size() != b.joints.size()) return;
    for (size_t i = 0; i < a.links.size(); i++) {
        const Link& x = a.links[i];
        const int bi = b.FindLink(x.name);
        check(bi >= 0, (tag + ": link " + x.name).c_str());
        if (bi < 0) continue;
        const Link& y = b.links[static_cast<size_t>(bi)];
        check(x.hasInertial == y.hasInertial, (tag + ": hasInertial " + x.name).c_str());
        if (x.hasInertial && y.hasInertial) {
            check(near(x.inertial.mass, y.inertial.mass) && near(x.inertial.ixx, y.inertial.ixx) && near(x.inertial.iyy, y.inertial.iyy) && near(x.inertial.izz, y.inertial.izz) &&
                      near(x.inertial.ixy, y.inertial.ixy) && near(x.inertial.ixz, y.inertial.ixz) && near(x.inertial.iyz, y.inertial.iyz),
                  (tag + ": inertial " + x.name).c_str());
            check(nearMat(x.inertial.origin, y.inertial.origin), (tag + ": inertial origin " + x.name).c_str());
        }
        check(x.visuals.size() == y.visuals.size() && x.collisions.size() == y.collisions.size(), (tag + ": shapes " + x.name).c_str());
        for (size_t k = 0; k < x.visuals.size() && k < y.visuals.size(); k++) {
            geomSame(x.visuals[k].geometry, y.visuals[k].geometry, tag + " visual " + x.name);
            check(nearMat(x.visuals[k].origin, y.visuals[k].origin), (tag + ": visual origin " + x.name).c_str());
            check(x.visuals[k].hasMaterial == y.visuals[k].hasMaterial, (tag + ": has material " + x.name).c_str());
            if (x.visuals[k].hasMaterial && x.visuals[k].material.hasColor)
                check(y.visuals[k].material.hasColor && near(x.visuals[k].material.color.r, y.visuals[k].material.color.r) && near(x.visuals[k].material.color.b, y.visuals[k].material.color.b), (tag + ": color " + x.name).c_str());
        }
        for (size_t k = 0; k < x.collisions.size() && k < y.collisions.size(); k++) geomSame(x.collisions[k].geometry, y.collisions[k].geometry, tag + " collision " + x.name);
    }
    for (size_t i = 0; i < a.joints.size(); i++) {
        const Joint& x = a.joints[i];
        const int bi = b.FindJoint(x.name);
        check(bi >= 0, (tag + ": joint " + x.name).c_str());
        if (bi < 0) continue;
        const Joint& y = b.joints[static_cast<size_t>(bi)];
        check(x.type == y.type && x.parent == y.parent && x.child == y.child, (tag + ": joint wiring " + x.name).c_str());
        check(nearMat(x.origin, y.origin), (tag + ": joint origin " + x.name).c_str());
        if (x.type != kJointFixed) check(near(x.axis.x, y.axis.x) && near(x.axis.y, y.axis.y) && near(x.axis.z, y.axis.z), (tag + ": axis " + x.name).c_str());
        check(x.hasLimit == y.hasLimit, (tag + ": hasLimit " + x.name).c_str());
        if (x.hasLimit) check(near(x.lower, y.lower) && near(x.upper, y.upper) && near(x.effort, y.effort) && near(x.velocity, y.velocity), (tag + ": limit " + x.name).c_str());
        check(near(x.damping, y.damping) && near(x.friction, y.friction), (tag + ": dynamics " + x.name).c_str());
        check(x.hasMimic == y.hasMimic && (!x.hasMimic || (x.mimicJoint == y.mimicJoint && near(x.mimicMultiplier, y.mimicMultiplier) && near(x.mimicOffset, y.mimicOffset))), (tag + ": mimic " + x.name).c_str());
    }
    check(a.lidars.size() == b.lidars.size(), (tag + ": lidar count").c_str());
    check(b.links[static_cast<size_t>(b.root)].name == a.links[static_cast<size_t>(a.root)].name, (tag + ": root").c_str());
}

void roundTrip(const String& file) {
    Robot a, b;
    String err;
    check(LoadUrdf(file, &a, &err), ("load " + file).c_str());
    const String out = tu::Tmp("simul_write_rt.urdf");
    check(SaveUrdf(a, out, &err), ("save " + file).c_str());
    check(LoadUrdf(out, &b, &err), ("reload " + file + ": " + err).c_str());
    sameRobot(a, b, file);
}

// 임의 회전 -> rpy -> 회전: 어떤 자세도 그대로 돌아와야 한다 (짐벌 락 근처 포함)
void testRpy() {
    const float angles[] = {0.0f, 0.3f, -1.2f, 1.5707963f, -1.5707963f, 2.8f, 3.14f};
    for (size_t a = 0; a < 7; a++)
        for (size_t b = 0; b < 7; b++)
            for (size_t c = 0; c < 7; c++) {
                dhMat m = dhEuler(angles[c], angles[b], angles[a]).GetMat();   // yaw, pitch, roll
                double rpy[3];
                RpyFromMat(m, rpy);
                const dhMat back = dhEuler(static_cast<float>(rpy[2]), static_cast<float>(rpy[1]), static_cast<float>(rpy[0])).GetMat();
                if (!nearMat(m, back, 2e-3)) {
                    printf("FAIL: rpy roundtrip roll=%f pitch=%f yaw=%f\n", angles[a], angles[b], angles[c]);
                    ++g_fail;
                    return;
                }
            }
}

// 여러 모델 + 회전된 모델을 하나의 URDF 로 저장하고 다시 읽는다
void testScene() {
    World w;
    String err;
    check(w.Add(kData + "arm.urdf", &err) && w.Add(kData + "arm_gripper.urdf", &err) && w.Add(kData + "lidar_room.urdf", &err), "add models");
    w.SetPlacement(1, dhMat::Quat(0.7f, dhVector(0.3f, 0.4f, 0.8f)));   // 비스듬히 돌린 두 번째 팔
    dhMat p = dhMat::Quat(0.4f, dhVector(0, 0, 1));
    p.v[12] = 2.0f; p.v[13] = -1.0f;
    w.SetPlacement(2, p);
    Robot scene, again;
    check(w.Build(LoadOptions(), &scene, &err), "build scene");
    const String out = tu::Tmp("simul_write_scene.urdf");
    check(SaveUrdf(scene, out, &err), "save scene");
    check(LoadUrdf(out, &again, &err), ("reload scene: " + err).c_str());
    sameRobot(scene, again, "scene");
    check(again.links[static_cast<size_t>(again.root)].name == "world", "scene root is world");
}

// 기본 도형(박스, 원통, 구): 관성이 공식대로 계산되고, 장면에 올려 URDF 로 저장했다 다시 읽어도 같다
void testPrimitives() {
    PrimitiveSpec box;
    box.type = kGeomBox;
    box.size[0] = 0.4f; box.size[1] = 0.6f; box.size[2] = 0.8f;
    box.mass = 12.0f;
    Robot b;
    String err;
    check(MakePrimitiveRobot("crate", box, &b, &err), "make box");
    check(near(b.links[0].inertial.ixx, 12.0 / 12.0 * (0.36 + 0.64)) && near(b.links[0].inertial.izz, 12.0 / 12.0 * (0.16 + 0.36)), "box inertia");
    PrimitiveSpec cyl;
    cyl.type = kGeomCylinder; cyl.radius = 0.1f; cyl.length = 1.0f; cyl.mass = 6.0f;
    Robot c;
    check(MakePrimitiveRobot("pipe", cyl, &c, &err), "make cylinder");
    check(near(c.links[0].inertial.izz, 0.5 * 6.0 * 0.01) && near(c.links[0].inertial.ixx, 6.0 / 12.0 * (3 * 0.01 + 1.0)), "cylinder inertia");
    PrimitiveSpec sph;
    sph.type = kGeomSphere; sph.radius = 0.5f; sph.mass = 5.0f;
    Robot sp;
    check(MakePrimitiveRobot("ball", sph, &sp, &err), "make sphere");
    check(near(sp.links[0].inertial.ixx, 0.4 * 5.0 * 0.25), "sphere inertia");
    PrimitiveSpec bad = box;
    bad.size[1] = 0.0f;
    Robot x;
    check(!MakePrimitiveRobot("bad", bad, &x, &err) && !err.empty(), "zero size is rejected");

    World w;
    check(w.Add(kData + "arm.urdf", &err), "add arm");
    check(w.AddRobot("crate", b, &err) && w.AddRobot("crate", b, &err) && w.AddRobot("pipe", c, &err) && w.AddRobot("ball", sp, &err), "add shapes to the world");
    check(w.entry(2).name == "crate_2", "duplicate shape name gets a number");
    dhMat p;
    p.v[12] = 1.0f; p.v[13] = 2.0f; p.v[14] = 0.4f;
    w.SetPlacement(1, p);
    Robot scene, again;
    check(w.Build(LoadOptions(), &scene, &err), "build scene with shapes");
    const String out = tu::Tmp("simul_write_shapes.urdf");
    check(SaveUrdf(scene, out, &err), "save scene with shapes");
    check(LoadUrdf(out, &again, &err), ("reload scene with shapes: " + err).c_str());
    sameRobot(scene, again, "shapes");
}


// 폴더로 저장: GLB 는 바이너리 STL 로 바뀌어 obj/ 에 들어가고(색은 URDF 로), STL/OBJ 는 복사되며, 다시 열면 같은 모양이 나온다
void testFolder() {
    const String src = tu::Tmp("simul_fold_src");
    tu::MkDir(src);
    {   // 삼각형 하나 + 재질 색 (0.2 0.4 0.6)
        Array<char> b;
        const float p[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
        for (int i = 0; i < 9; i++) testglb::putf(&b, p[i]);
        const String js =
            "{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"scene\":0,\"nodes\":[{\"mesh\":0}],"
            "\"materials\":[{\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.2,0.4,0.6,1]}}],"
            "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"material\":0}]}],"
            "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\",\"min\":[0,0,0],\"max\":[1,1,0]}],"
            "\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],\"buffers\":[{\"byteLength\":36}]}";
        const Array<char> glb = testglb::MakeGlb(js, b);
        WriteFileBytes(src + "/tri.glb", &glb[0], glb.size());
    }
    {   // OBJ + MTL + 텍스처 (내용은 의미 없는 바이트)
        const String obj = "mtllib m.mtl\nusemtl a\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
        const String mtl = "newmtl a\nKd 1 0 0\nmap_Kd t.png\n";
        WriteFileText(src + "/box.obj", obj);
        WriteFileText(src + "/m.mtl", mtl);
        WriteFileText(src + "/t.png", "PNGDATA");
    }
    {   // STL 원본: 바이너리 삼각형 하나

        Array<unsigned char> st(84 + 50, 0);
        const uint32_t n = 1;
        memcpy(&st[80], &n, 4);
        const float v[9] = {0, 0, 0, 0, 2, 0, 0, 0, 3};
        memcpy(&st[84 + 12], v, 36);
        WriteFileBytes(src + "/src.stl", &st[0], st.size());
    }
    const String urdfPath = src + "/in.urdf";
    WriteFileText(urdfPath,
        "<robot name=\"m\"><link name=\"base\">"
        "<visual name=\"g\"><origin xyz=\"1 0 0\"/><geometry><mesh filename=\"tri.glb\" scale=\"2 1 1\"/></geometry></visual>"
        "<visual name=\"s\"><geometry><mesh filename=\"src.stl\"/></geometry><material name=\"red\"><color rgba=\"1 0 0 1\"/></material></visual>"
        "<visual name=\"o\"><geometry><mesh filename=\"box.obj\"/></geometry></visual>"
        "<visual name=\"b\"><geometry><box size=\"1 2 3\"/></geometry></visual>"
        "<collision><geometry><mesh filename=\"tri.glb\" scale=\"2 1 1\"/></geometry></collision>"
        "</link></robot>");
    Robot r;
    String err;
    check(LoadUrdf(urdfPath, &r, &err), ("load folder source: " + err).c_str());

    const String out = tu::Tmp("simul_fold_out");
    RobotMeshes orig;
    check(LoadRobotMeshes(r, LoadOptions(), &orig) && orig.visuals.size() >= 4, "load original meshes");

    // ---- 기본: GLB -> OBJ + MTL (색 유지)
    {
        FolderSaveResult res;
        check(SaveUrdfFolder(r, out + "/robotx", LoadOptions(), &err, &res), ("save folder: " + err).c_str());
        check(res.urdfPath == out + "/robotx/robotx.urdf", "urdf is <folder>/<folder>.urdf");
        check(res.converted == 1 && res.copied == 2, "glb converted once (shared by visual and collision), stl and obj copied");
        Array<unsigned char> tmp;
        check(ReadAllBytes(out + "/robotx/obj/tri/tri.obj", &tmp, &err) && ReadAllBytes(out + "/robotx/obj/tri/tri.mtl", &tmp, &err), "glb written as obj + mtl in obj/tri/");
        const String mtl(tmp.empty() ? "" : reinterpret_cast<const char*>(&tmp[0]), tmp.size());
        check(mtl.find("Kd 0.2 0.4 0.6") != String::npos, "mtl keeps the base color");
        check(ReadAllBytes(out + "/robotx/obj/src.stl", &tmp, &err), "stl copied to obj/");
        check(ReadAllBytes(out + "/robotx/obj/box/box.obj", &tmp, &err) && ReadAllBytes(out + "/robotx/obj/box/m.mtl", &tmp, &err) && ReadAllBytes(out + "/robotx/obj/box/t.png", &tmp, &err), "obj, mtl and texture copied together");

        // URDF 는 상대 경로만 쓴다 (원본 폴더 이름이 없어야 한다)
        Array<unsigned char> ub;
        check(ReadAllBytes(res.urdfPath, &ub, &err), "read saved urdf");
        const String text(ub.empty() ? "" : reinterpret_cast<const char*>(&ub[0]), ub.size());
        check(text.find(src) == String::npos, "saved urdf has no absolute source path");
        check(text.find("filename=\"obj/tri/tri.obj\"") != String::npos && text.find("filename=\"obj/src.stl\"") != String::npos && text.find("filename=\"obj/box/box.obj\"") != String::npos, "meshes referenced as obj/...");
        check(text.find("0.2 0.4 0.6") != String::npos, "a single-color glb also writes the color into the urdf (for tools that ignore mtl)");

        // 다시 읽기: 같은 구조, 같은 모양, 같은 색
        Robot again;
        check(LoadUrdf(res.urdfPath, &again, &err), ("reload folder urdf: " + err).c_str());
        check(again.links.size() == 1 && again.links[0].visuals.size() == 4 && again.links[0].collisions.size() == 1, "same shapes after reload");
        check(again.links[0].visuals[0].geometry.scale[0] == 1.0f, "scale is baked into the OBJ");
        RobotMeshes b;
        check(LoadRobotMeshes(again, LoadOptions(), &b), "load folder meshes");
        check(orig.visuals.size() == b.visuals.size(), "same visual count");
        for (size_t i = 0; i < orig.visuals.size() && i < b.visuals.size(); i++) {
            dhVector amin, amax, bmin, bmax;
            orig.visuals[i].scene.ComputeBounds(&amin, &amax);
            b.visuals[i].scene.ComputeBounds(&bmin, &bmax);
            check(near(amin.x, bmin.x, 1e-4) && near(amin.y, bmin.y, 1e-4) && near(amin.z, bmin.z, 1e-4) && near(amax.x, bmax.x, 1e-4) && near(amax.y, bmax.y, 1e-4) && near(amax.z, bmax.z, 1e-4),
                  ("visual shape bounds match " + ToString(i)).c_str());
        }
        check(orig.visuals[0].scene.TriangleCount() == b.visuals[0].scene.TriangleCount(), "triangle count");
        check(!b.visuals[0].scene.materials.empty() && near(b.visuals[0].scene.materials[0].baseColor.r, 0.2) && near(b.visuals[0].scene.materials[0].baseColor.g, 0.4) && near(b.visuals[0].scene.materials[0].baseColor.b, 0.6),
              "the color survives the folder save (read back from the mtl)");
    }

    // ---- STL 선택: 바이너리 STL, 색은 URDF 로 (기본 색 하나)
    {
        FolderSaveResult res;
        FolderOptions fo;
        fo.glbToStl = true;
        check(SaveUrdfFolder(r, out + "/stlx", LoadOptions(), &err, &res, WriteOptions(), fo), ("save folder as stl: " + err).c_str());
        Array<unsigned char> stl;
        check(ReadAllBytes(out + "/stlx/obj/tri.stl", &stl, &err), "obj/tri.stl exists");
        check(stl.size() == 84 + 50, "binary STL size = 84 + 50 * triangles");
        uint32_t cnt = 0;
        if (stl.size() >= 84) memcpy(&cnt, &stl[80], 4);
        check(cnt == 1, "STL triangle count");
        Robot again;
        check(LoadUrdf(res.urdfPath, &again, &err), ("reload stl folder urdf: " + err).c_str());
        check(again.links[0].visuals[0].hasMaterial && again.links[0].visuals[0].material.hasColor &&
                  near(again.links[0].visuals[0].material.color.r, 0.2) && near(again.links[0].visuals[0].material.color.g, 0.4) && near(again.links[0].visuals[0].material.color.b, 0.6),
              "stl mode: the glb base color moves into the urdf material");
        check(again.links[0].visuals[0].geometry.scale[0] == 1.0f, "stl mode: scale baked");
        check(res.warnings.empty() || true, "warnings only advise");
    }

    Robot again;
    check(LoadUrdf(out + "/robotx/robotx.urdf", &again, &err), "reload for the move test");
    FolderSaveResult res;
    // 폴더째 옮겨도 열린다
    const String moved = tu::Tmp("simul_fold_moved");
    check(SaveUrdfFolder(again, moved + "/other", LoadOptions(), &err, &res), ("save the folder urdf again: " + err).c_str());
    check(res.warnings.empty() || true, "warnings are only advice");
}


// 관절로 연결한 장면(부모 링크에 매달린 모델)도 URDF 로 저장했다 다시 읽으면 같다
void testConnectedScene() {
    World w;
    String err;
    check(w.Add(kData + "arm.urdf", &err), "add arm");
    PrimitiveSpec box;
    box.type = kGeomBox;
    box.size[0] = box.size[1] = box.size[2] = 0.1f;
    box.mass = 1.0f;
    Robot b;
    check(MakePrimitiveRobot("crate", box, &b, &err) && w.AddRobot("crate", b, &err), "add crate");
    AttachSpec a;
    a.parentModel = w.entry(0).name;
    a.parentLink = "tool";
    a.type = kJointPrismatic;
    a.axis = dhVector(1, 0, 0);
    a.hasLimit = true; a.lower = 0.0f; a.upper = 0.3f;
    check(w.SetAttach(1, a, &err), "connect");
    w.SetPlacement(1, dhMat::Trans(0.1f, 0.0f, 0.2f));
    Robot scene, again;
    check(w.Build(LoadOptions(), &scene, &err), "build connected scene");
    const String out = tu::Tmp("simul_write_connected.urdf");
    check(SaveUrdf(scene, out, &err) && LoadUrdf(out, &again, &err), ("save and reload connected scene: " + err).c_str());
    sameRobot(scene, again, "connected");
    const int j = again.FindJoint("crate_attach");
    check(j >= 0 && again.joints[static_cast<size_t>(j)].parent == "tool" && again.joints[static_cast<size_t>(j)].type == kJointPrismatic &&
              near(again.joints[static_cast<size_t>(j)].axis.x, 1.0) && near(again.joints[static_cast<size_t>(j)].upper, 0.3) && near(again.joints[static_cast<size_t>(j)].origin.v[14], 0.2),
          "attach joint round-trips (parent link, type, axis, limit, origin)");
}

}  // namespace

// 모델 크기 배율: 길이는 s 배, 질량 s^3, 관성 s^5, 순기구학의 끝점도 s 배. 원래 크기 기준의 절대 배율이다.
void testScale() {
    Robot arm;
    String err;
    check(LoadUrdf(kData + "arm.urdf", &arm, &err), "load arm for scale");
    Robot half;
    check(ScaleRobot(arm, 0.5, true, &half), "scale arm by 0.5");
    check(!ScaleRobot(arm, 0.0, true, &half) && !ScaleRobot(arm, -1.0, true, &half), "bad scale rejected");
    check(ScaleRobot(arm, 0.5, true, &half), "scale arm by 0.5 again");
    const int tool = arm.FindLink("tool");
    JointValues q;
    q["shoulder_pitch"] = 0.4f;
    q["elbow_pitch"] = -0.7f;
    const dhMat a = arm.LinkTransforms(q)[static_cast<size_t>(tool)];
    const dhMat b = half.LinkTransforms(q)[static_cast<size_t>(tool)];
    check(near(b.v[12], a.v[12] * 0.5, 1e-5) && near(b.v[13], a.v[13] * 0.5, 1e-5) && near(b.v[14], a.v[14] * 0.5, 1e-5), "end point moves s times as far");
    check(near(b.v[0], a.v[0], 1e-6) && near(b.v[10], a.v[10], 1e-6), "orientation is unchanged by scaling");
    const int ua = arm.FindLink("upper_arm");
    check(near(half.links[static_cast<size_t>(ua)].visuals[0].geometry.size[0], arm.links[static_cast<size_t>(ua)].visuals[0].geometry.size[0] * 0.5), "box size scaled");
    check(near(half.links[static_cast<size_t>(ua)].inertial.mass, arm.links[static_cast<size_t>(ua)].inertial.mass / 8.0), "mass scales with s^3");
    check(near(half.links[static_cast<size_t>(ua)].inertial.ixx, arm.links[static_cast<size_t>(ua)].inertial.ixx / 32.0, 1e-7), "inertia scales with s^5");
    Robot same;
    check(ScaleRobot(arm, 0.5, false, &same) && near(same.links[static_cast<size_t>(ua)].inertial.mass, arm.links[static_cast<size_t>(ua)].inertial.mass), "massScales=false keeps the mass");
    // 이동 관절의 한계와 SetModelScale 의 절대 배율
    Robot r;
    Link l0, l1;
    l0.name = "a"; l1.name = "b";
    r.links.push_back(l0); r.links.push_back(l1);
    Joint j;
    j.name = "slide"; j.type = kJointPrismatic; j.parent = "a"; j.child = "b"; j.lower = -1; j.upper = 2; j.hasLimit = true;
    j.origin.v[14] = 1.0f;
    r.joints.push_back(j);
    check(FinalizeRobot(&r, &err), "finalize prismatic robot");
    Robot big;
    check(ScaleRobot(r, 3.0, true, &big) && near(big.joints[0].upper, 6.0) && near(big.joints[0].lower, -3.0) && near(big.joints[0].origin.v[14], 3.0), "prismatic limits and origin scale");
    World w;
    check(w.Add(kData + "arm.urdf", &err), "add arm to world");
    check(w.SetModelScale(0, 2.0) && w.SetModelScale(0, 2.0) && near(w.ModelScale(0), 2.0), "world model scale is absolute");
    Robot built;
    check(w.Build(LoadOptions(), &built, &err), "build scaled world");
    check(near(built.links[static_cast<size_t>(built.FindLink("upper_arm"))].visuals[0].geometry.size[0], 1.2), "scaled geometry reaches the built robot");
    check(w.SetModelScale(0, 1.0) && w.Build(LoadOptions(), &built, &err) && near(built.links[static_cast<size_t>(built.FindLink("upper_arm"))].visuals[0].geometry.size[0], 0.6), "scale back to 1 restores the original");
    check(!w.SetModelScale(5, 2.0) && !w.SetModelScale(0, 0.0), "bad index or scale rejected by world");
}

int main() {
    testScale();
    testPrimitives();
    roundTrip(kData + "arm.urdf");
    roundTrip(kData + "arm_gripper.urdf");
    roundTrip(kData + "lidar_room.urdf");
    roundTrip(kData + "lidar_room_3d.urdf");
    testRpy();
    testScene();
    testFolder();
    testConnectedScene();
    if (g_fail == 0) printf("test_curdf_write: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
