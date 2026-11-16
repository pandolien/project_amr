// AMR + 로봇 팔 시스템 진입점.
//   amr_main [옵션] [파일]      파일: .urdf .glb .gltf .stl .obj .dae
//   Qt 로 빌드했으면 3D 뷰어 창을 띄우고, 아니면 파일 정보만 출력한다 (--info 와 같음).
#include "CBase/File.h"
#include "CSensor/Lidar.h"
#include "CoreQt/QtString.h"
#include "CBase/Math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "CBase/String.h"
#include "CBase/Array.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#include "CCollision/RobotCollision.h"
#include "CDynamics/Gravity.h"
#include "CMesh/MeshLoader.h"
#include "CURDF/Urdf.h"

namespace {

// Windows 콘솔은 기본이 로컬 코드 페이지라 UTF-8 한글이 깨진다. 출력만 UTF-8 로 바꾼다.
void InitConsole() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
}

struct Options {
    amr::String file;
    bool info;                 // 창 없이 정보만 출력
    bool aabb;                 // AABB 충돌 박스를 처음부터 표시
    bool obb;                  // OBB 충돌 박스를 처음부터 표시
    bool torque;               // 관절별 중력 토크를 출력 (--info 와 같이)
    double gravity[3];         // 중력 가속도 (m/s^2, 루트 링크 좌표계)
    amr::String shot;          // 비어 있지 않으면 3D 화면을 PNG 로 저장하고 종료
    amr::String shotWindow;    // 비어 있지 않으면 창 전체(패널 포함)를 PNG 로 저장하고 종료
    amr::String select;        // 시작할 때 선택(강조)할 링크 이름
    int dragX0, dragY0, dragX1, dragY1;  // --drag: 눌렀다가 끌고 떼는 마우스 동작 (-1 이면 안 함)
    bool motorsOff;            // --motors-off: 처음부터 모터를 끈다 (관절이 중력에 처짐)
    double simulate;           // --simulate 초: 모터를 끄고 그 시간만큼 시뮬레이션한 뒤 관절 값을 출력 (확인용, 실시간이 아님)
    bool noFall;               // --no-fall: 중력 낙하를 끈다
    bool settle;               // --settle: 낙하가 끝날 때까지 진행한 뒤 위치를 출력 (확인용)
    amr::String lidarLink;     // --lidar: 2D 라이다를 달 링크 이름 (비어 있으면 없음)
    float lidarMount[3];       //   그 링크 좌표계에서의 센서 위치 (m)
    amr::String lidarCsv;      // --lidar-csv: 스캔 결과를 CSV 로 저장할 파일
    int lidarChannels;         // --lidar-channels: 수직 채널 수 (1 이면 2D)
    float lidarElev[2];        //   첫/마지막 채널의 고각 (도)
    bool hasMove;
    float move[3];             // --move: 시작할 때 오브젝트를 옮길 위치 (월드, m)
    int clickX, clickY;        // --click: 3D 화면의 이 픽셀을 마우스로 클릭한 것처럼 처리 (-1 이면 안 함)
    amr::Array<amr::Pair<amr::String, double> > joints;           // --joint 이름=값 (revolute: 도, prismatic: m)
    amr::Array<amr::Pair<amr::String, amr::String> > packages;    // --package 이름=폴더
    int width, height;
    Options() : info(false), aabb(false), obb(false), torque(false), dragX0(-1), dragY0(-1), dragX1(-1), dragY1(-1), motorsOff(false), simulate(0), noFall(false), settle(false), hasMove(false), clickX(-1), clickY(-1), width(1100), height(720) {
        lidarMount[0] = lidarMount[1] = lidarMount[2] = 0;
        lidarChannels = 1;
        lidarElev[0] = lidarElev[1] = 0;
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -amr::dynamics::kStandardGravity;
    }
};

void PrintUsage(const char* exe) {
    printf(
        "사용법: %s [옵션] [파일]\n"
        "  파일               .urdf .glb .gltf .stl .obj .dae (생략하면 빈 뷰어)\n"
        "  --info             창을 띄우지 않고 파일 정보만 출력\n"
        "  --aabb, --obb      AABB / OBB 충돌 박스를 표시 (--info 와 같이 쓰면 링크별 AABB 와 충돌 목록을 출력)\n"
        "  --torque           관절별 중력 토크를 출력 (--info 와 같이 쓰거나, 창의 관절 패널에서 확인)\n"
        "  --gravity gx,gy,gz 중력 가속도 m/s^2 (루트 링크 좌표계, 기본 0,0,-9.80665)\n"
        "  --joint 이름=값    관절 값 지정 (revolute/continuous: 도, prismatic: m), 여러 번 가능\n"
        "  --package 이름=폴더 package://이름/ 을 찾을 폴더, 여러 번 가능\n"
        "  --shot 파일.png    3D 화면을 저장하고 종료 (Qt 빌드에서만)\n"
        "  --shot-window 파일.png  창 전체(구조 트리, 관절 패널 포함)를 저장하고 종료\n"
        "  --select 링크이름  시작할 때 그 링크를 선택해서 강조\n"
        "  --drag x0,y0,x1,y1 3D 화면에서 마우스를 (x0,y0) 에서 누르고 (x1,y1) 까지 끈 것처럼 처리 (좌표계 화살표 이동 확인용. 팔 링크를 --select 로 골라 끌면 IK 로 관절이 따라오고 관절 값을 출력)\n"
        "  --motors-off       모터를 끈 상태로 시작: 관절이 중력, 관성, 마찰에 따라 움직임 (보기 > 모터 끄기 / M)\n"
        "  --simulate 초      (확인용) 모터를 끄고 그 시간만큼 진행한 뒤 관절 값을 출력\n"
        "  --no-fall          중력 낙하를 끈다 (기본은 들어 올린 물체가 바닥으로 떨어짐)\n"
        "  --settle           (확인용) 낙하가 끝날 때까지 진행하고 위치를 출력\n"
        "  --move x,y,z       시작할 때 오브젝트(정적 모델, 로봇 전체)를 이 위치(월드, m)로 옮김\n"
        "  --lidar 링크[,x,y,z]  그 링크에 2D 라이다를 달아 스캔한 점(빨강)과 광선을 표시. x,y,z 는 링크 좌표계의 센서 위치(m).\n"
        "                     센서가 붙은 링크 자신을 뺀 나머지 메시를 스캔하고, 맞은 점 수와 거리 범위를 출력\n"
        "  --lidar-channels 개수,최소고각,최대고각  (도) 수직 채널을 늘려 3D 라이다로 (예: 16,-15,15). 큰 메시에서는 느리다\n"
        "  --lidar-csv 파일.csv  (--lidar 와 함께) 스캔 결과를 CSV 로 저장: 광선 번호, 채널, 방위각(도), 고각(도), 맞음(1/0), 거리(m), 맞은 점 x,y,z(월드), 맞은 링크\n"
        "  --click x,y        3D 화면의 (x,y) 픽셀을 왼쪽 클릭한 것처럼 선택 (--shot 과 함께 확인용)\n"
        "  --size 가로x세로   창 크기 (기본 1100x720)\n"
        "  -h, --help         도움말\n",
        exe);
}

bool SplitPair(const amr::String& s, amr::String* a, amr::String* b) {
    const size_t eq = s.find('=');
    if (eq == amr::String::npos || eq == 0) return false;
    *a = s.substr(0, eq);
    *b = s.substr(eq + 1);
    return true;
}

// 0: 계속, 1: 정상 종료(도움말), 2: 인자 오류
int ParseArgs(int argc, char** argv, Options* o) {
    for (int i = 1; i < argc; i++) {
        const amr::String a = argv[i];
        if (a == "-h" || a == "--help") { PrintUsage(argv[0]); return 1; }
        if (a == "--info") { o->info = true; continue; }
        if (a == "--aabb") { o->aabb = true; continue; }
        if (a == "--obb") { o->obb = true; continue; }
        if (a == "--torque") { o->torque = true; continue; }
        if (a == "--gravity" && i + 1 < argc) {
            if (sscanf(argv[++i], "%lf,%lf,%lf", &o->gravity[0], &o->gravity[1], &o->gravity[2]) != 3) {
                fprintf(stderr, "--gravity 는 gx,gy,gz 형식입니다 (예: 0,0,-9.80665)\n");
                return 2;
            }
            continue;
        }
        const bool hasNext = i + 1 < argc;
        if (a == "--shot" && hasNext) { o->shot = argv[++i]; continue; }
        if (a == "--shot-window" && hasNext) { o->shotWindow = argv[++i]; continue; }
        if (a == "--select" && hasNext) { o->select = argv[++i]; continue; }
        if (a == "--drag" && hasNext) {
            if (sscanf(argv[++i], "%d,%d,%d,%d", &o->dragX0, &o->dragY0, &o->dragX1, &o->dragY1) != 4) { fprintf(stderr, "--drag 는 x0,y0,x1,y1 형식입니다\n"); return false; }
            continue;
        }
        if (a == "--motors-off") { o->motorsOff = true; continue; }
        if (a == "--simulate" && hasNext) { o->simulate = atof(argv[++i]); o->motorsOff = true; continue; }
        if (a == "--no-fall") { o->noFall = true; continue; }
        if (a == "--settle") { o->settle = true; continue; }
        if (a == "--lidar" && hasNext) {
            const amr::String spec = argv[++i];
            const size_t comma = spec.find(',');
            o->lidarLink = spec.substr(0, comma);
            if (comma != amr::String::npos &&
                sscanf(spec.c_str() + comma + 1, "%f,%f,%f", &o->lidarMount[0], &o->lidarMount[1], &o->lidarMount[2]) != 3) {
                fprintf(stderr, "--lidar 는 링크이름 또는 링크이름,x,y,z 형식입니다\n");
                return false;
            }
            continue;
        }
        if (a == "--lidar-channels" && hasNext) {
            if (sscanf(argv[++i], "%d,%f,%f", &o->lidarChannels, &o->lidarElev[0], &o->lidarElev[1]) != 3 || o->lidarChannels < 1) {
                fprintf(stderr, "--lidar-channels 는 개수,최소고각,최대고각 형식입니다 (예: 16,-15,15)\n");
                return false;
            }
            continue;
        }
        if (a == "--lidar-csv" && hasNext) { o->lidarCsv = argv[++i]; continue; }
        if (a == "--move" && hasNext) {
            if (sscanf(argv[++i], "%f,%f,%f", &o->move[0], &o->move[1], &o->move[2]) != 3) { fprintf(stderr, "--move 는 x,y,z 형식입니다\n"); return false; }
            o->hasMove = true;
            continue;
        }
        if (a == "--click" && hasNext) {
            if (sscanf(argv[++i], "%d,%d", &o->clickX, &o->clickY) != 2) { fprintf(stderr, "--click 은 x,y 형식입니다\n"); return false; }
            continue;
        }
        if (a == "--joint" && hasNext) {
            amr::String n, v;
            if (!SplitPair(argv[++i], &n, &v)) { fprintf(stderr, "--joint 은 이름=값 형식입니다\n"); return 2; }
            o->joints.push_back(amr::MakePair(n, atof(v.c_str())));
            continue;
        }
        if (a == "--package" && hasNext) {
            amr::String n, v;
            if (!SplitPair(argv[++i], &n, &v)) { fprintf(stderr, "--package 는 이름=폴더 형식입니다\n"); return 2; }
            o->packages.push_back(amr::MakePair(n, v));
            continue;
        }
        if (a == "--size" && hasNext) {
            if (sscanf(argv[++i], "%dx%d", &o->width, &o->height) != 2 || o->width < 100 || o->height < 100) {
                fprintf(stderr, "--size 는 가로x세로 형식입니다 (예: 1280x800)\n");
                return 2;
            }
            continue;
        }
        if (!a.empty() && a[0] == '-') { fprintf(stderr, "알 수 없는 옵션: %s (--help)\n", a.c_str()); return 2; }
        o->file = a;
    }
    return 0;
}

amr::String LowerExt(const amr::String& p) {
    const size_t dot = p.find_last_of('.');
    if (dot == amr::String::npos) return amr::String();
    amr::String e = p.substr(dot + 1);
    for (size_t i = 0; i < e.size(); i++) if (e[i] >= 'A' && e[i] <= 'Z') e[i] = static_cast<char>(e[i] - 'A' + 'a');
    return e;
}

// --joint 로 받은 값(각도 관절은 degree, 이동 관절은 m)을 내부 값(rad, m)으로 바꾼다
amr::urdf::JointValues JointValuesFromOptions(const amr::urdf::Robot& robot, const Options& o) {
    amr::urdf::JointValues q;
    for (size_t i = 0; i < o.joints.size(); i++) {
        const int ji = robot.FindJoint(o.joints[i].first);
        if (ji < 0) { fprintf(stderr, "경고: 관절 '%s' 이 없습니다\n", o.joints[i].first.c_str()); continue; }
        q[o.joints[i].first] = static_cast<float>(robot.joints[static_cast<size_t>(ji)].FromDisplay(o.joints[i].second));
    }
    return q;
}

// 링크별 AABB 와 겹치는 링크 쌍(AABB 후보 -> OBB 로 확인한 충돌)을 출력한다 (--joint 로 준 자세 기준)
void PrintCollisions(const amr::urdf::Robot& robot, const Options& o, const amr::urdf::LoadOptions& load) {
    const amr::urdf::JointValues q = JointValuesFromOptions(robot, o);
    amr::CollisionModelOptions copt;
    copt.load = load;
    copt.fallbackToVisual = true;
    amr::RobotCollisionModel model;
    amr::BuildCollisionModel(robot, copt, &model);
    for (size_t i = 0; i < model.warnings.size(); i++) printf("경고: %s\n", model.warnings[i].c_str());
    size_t fromVisual = 0;
    for (size_t i = 0; i < model.shapes.size(); i++) if (model.shapes[i].fromVisual) fromVisual++;
    printf("AABB 충돌 검사: 도형 %zu개 (visual 로 대체한 것 %zu개)\n", model.shapes.size(), fromVisual);
    const amr::SelfCollisionReport rep = amr::CheckSelfCollisions(robot, model, q);
    for (size_t i = 0; i < rep.linkAabbs.size(); i++) {
        if (rep.linkAabbs[i].IsEmpty()) continue;
        const amr::Aabb& b = rep.linkAabbs[i];
        printf("  link '%s': (%.3f, %.3f, %.3f) ~ (%.3f, %.3f, %.3f)\n", robot.links[i].name.c_str(), static_cast<double>(b.lo.x),
                    static_cast<double>(b.lo.y), static_cast<double>(b.lo.z), static_cast<double>(b.hi.x), static_cast<double>(b.hi.y),
                    static_cast<double>(b.hi.z));
    }
    const struct { const char* label; const amr::Array<amr::CollisionPair>* pairs; } lists[2] = {
        {"AABB 겹침 후보", &rep.aabbCandidates}, {"OBB 충돌", &rep.collisions}};
    for (int k = 0; k < 2; k++) {
        const amr::Array<amr::CollisionPair>& pairs = *lists[k].pairs;
        printf("%s %zu건", lists[k].label, pairs.size());
        for (size_t i = 0; i < pairs.size(); i++)
            printf("%s %s <-> %s", i == 0 ? ":" : ",", robot.links[static_cast<size_t>(pairs[i].a)].name.c_str(),
                        robot.links[static_cast<size_t>(pairs[i].b)].name.c_str());
        printf("%s\n", k == 0 ? " (직접 연결된 링크는 제외)" : "");
    }
}

// 관절별 중력 하중 (--joint 로 준 자세, --gravity 방향 기준). 모든 관절을 따로 보여준다.
//   구동 관절: 모터가 내야 할 토크 (mimic 이 달려 있으면 합산된 값), 링크별 기여 내역
//   mimic 관절: 자기 몫 (원본 관절에 합산됨), fixed 등: 체결부가 받는 힘/모멘트
void PrintTorques(const amr::urdf::Robot& robot, const Options& o) {
    const amr::urdf::JointValues q = JointValuesFromOptions(robot, o);
    const amr::dynamics::Gravity g(o.gravity[0], o.gravity[1], o.gravity[2]);
    const amr::dynamics::JointTorques motor = amr::dynamics::GravityTorques(robot, q, g);
    const amr::Array<amr::dynamics::JointLoad> loads = amr::dynamics::GravityLoads(robot, q, g);
    printf("중력 하중: g = (%.4f, %.4f, %.4f) m/s^2 (루트 링크 좌표계)\n", g.x, g.y, g.z);
    double com[3], mass = 0;
    if (amr::dynamics::CenterOfMass(robot, q, com, &mass))
        printf("  전체 질량 %.4f kg, 질량중심 (%.4f, %.4f, %.4f) m\n", mass, com[0], com[1], com[2]);
    else
        printf("  (<inertial> 질량이 있는 링크가 없습니다)\n");
    for (size_t i = 0; i < robot.joints.size(); i++) {
        const amr::urdf::Joint& j = robot.joints[i];
        const amr::dynamics::JointLoad& l = loads[i];
        if (!j.IsMovable()) {
            const double f = amr::Sqrt(l.force[0] * l.force[0] + l.force[1] * l.force[1] + l.force[2] * l.force[2]);
            const double m = amr::Sqrt(l.moment[0] * l.moment[0] + l.moment[1] * l.moment[1] + l.moment[2] * l.moment[2]);
            printf("  %-16s (fixed)  체결 하중: 힘 %.4f N, 모멘트 %.4f N*m\n", j.name.c_str(), f, m);
            continue;
        }
        const amr::urdf::JointValues::const_iterator qi = q.find(j.name);
        const double shown = j.ToDisplay(qi == q.end() ? 0.0 : static_cast<double>(qi->second));
        if (j.hasMimic) {
            printf("  %-16s (mimic %s)  자기 몫 %.4f %s  (원본 관절에 합산됨)\n", j.name.c_str(), j.mimicJoint.c_str(), l.torque, j.TorqueUnit());
        } else {
            const double tau = motor.find(j.name)->second;
            printf("  %-16s %9.3f %-3s  토크 %10.4f %s", j.name.c_str(), shown, j.DisplayUnit(), tau, j.TorqueUnit());
            if (j.effort > 0.0f) printf("  (한계 %g%s)", static_cast<double>(j.effort), amr::Abs(tau) > static_cast<double>(j.effort) ? ", 초과!" : "");
            printf("\n");
        }
        const amr::Array<amr::dynamics::LinkContribution> parts = amr::dynamics::GravityContributions(robot, q, j.name, g);
        for (size_t k = 0; k < parts.size(); k++) printf("      <- %-14s %10.4f %s\n", parts[k].link.c_str(), parts[k].value, j.TorqueUnit());
    }
}

// 창 없이 파일 정보를 출력한다. 성공하면 0.
int PrintInfo(const Options& o) {
    if (o.file.empty()) { PrintUsage("amr_main"); return 2; }
    amr::String err;
    if (LowerExt(o.file) == "urdf") {
        amr::urdf::Robot robot;
        if (!amr::urdf::LoadUrdf(o.file, &robot, &err)) { fprintf(stderr, "URDF 읽기 실패: %s\n", err.c_str()); return 1; }
        printf("로봇 '%s': link %zu개, joint %zu개 (구동 %zu개), 루트 link '%s'\n", robot.name.c_str(), robot.links.size(),
                    robot.joints.size(), robot.ActuatedJoints().size(), robot.links[static_cast<size_t>(robot.root)].name.c_str());
        amr::urdf::LoadOptions opt;
        for (size_t i = 0; i < o.packages.size(); i++) opt.packagePaths.push_back(o.packages[i]);
        amr::urdf::RobotMeshes meshes;
        const bool ok = amr::urdf::LoadRobotMeshes(robot, opt, &meshes);
        size_t tris = 0;
        for (size_t i = 0; i < meshes.visuals.size(); i++) tris += meshes.visuals[i].scene.TriangleCount();
        printf("visual %zu개 읽음, 삼각형 %zu개\n", meshes.visuals.size(), tris);
        for (size_t i = 0; i < robot.warnings.size(); i++) printf("경고: %s\n", robot.warnings[i].c_str());
        for (size_t i = 0; i < meshes.warnings.size(); i++) printf("경고: %s\n", meshes.warnings[i].c_str());
        if (o.aabb || o.obb) PrintCollisions(robot, o, opt);
        if (o.torque) PrintTorques(robot, o);
        return ok ? 0 : 1;
    }
    amr::Scene scene;
    if (!amr::LoadMesh(o.file, &scene, &err)) { fprintf(stderr, "읽기 실패: %s\n", err.c_str()); return 1; }
    dhVector mn, mx;
    scene.ComputeBounds(&mn, &mx);
    printf("메시 %zu개, 노드 %zu개, 삼각형 %zu개, 재질 %zu개\n", scene.meshes.size(), scene.nodes.size(), scene.TriangleCount(),
                scene.materials.size());
    printf("경계 상자: (%g, %g, %g) ~ (%g, %g, %g)\n", static_cast<double>(mn.x), static_cast<double>(mn.y), static_cast<double>(mn.z),
                static_cast<double>(mx.x), static_cast<double>(mx.y), static_cast<double>(mx.z));
    for (size_t i = 0; i < scene.warnings.size(); i++) printf("경고: %s\n", scene.warnings[i].c_str());
    return 0;
}

}  // namespace

#ifdef AMR_HAS_QT

#include <QAction>
#include <QApplication>
#include <QMouseEvent>
#include <QDockWidget>
#include <QDragEnterEvent>
#include <QFileDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <QLabel>
#include <QMainWindow>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QSurfaceFormat>
#include <QStatusBar>
#include <QTimer>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QDir>
#include <QFileInfo>

#include "3d/Units.h"
#include "CDynamics/Dynamics.h"
#include "3d/dhPose.h"
#include "CoreQt/JointPanel.h"
#include "CoreQt/ModelViewer.h"
#include "CoreQt/StructurePanel.h"

namespace {

class MainWindow : public QMainWindow {
public:
    explicit MainWindow(const Options& o) : QMainWindow(nullptr), gravity_(o.gravity[0], o.gravity[1], o.gravity[2]) {
        setWindowTitle("AMR RobotArm Viewer");
        resize(o.width, o.height);
        setAcceptDrops(true);

        viewer_ = new ModelViewer(this);
        for (size_t i = 0; i < o.packages.size(); i++)
            viewer_->LoadOptions().packagePaths.push_back(amr::MakePair(o.packages[i].first, o.packages[i].second));
        setCentralWidget(viewer_);

        tree_ = new StructurePanel;
        treeDock_ = new QDockWidget(QString::fromUtf8("구조"), this);
        treeDock_->setWidget(tree_);
        addDockWidget(Qt::LeftDockWidgetArea, treeDock_);
        treeDock_->hide();
        QObject::connect(tree_, &StructurePanel::linkSelected, this, [this](int link) {
            viewer_->SetSelectedLink(link);
            updateSelectionLabel();
        });
        // 화면에서 물체를 클릭하면 구조 트리의 같은 링크가 선택되고, 좌표계와 자세가 표시된다
        QObject::connect(viewer_, &ModelViewer::selectionPicked, this, [this](int link, bool) {
            tree_->SelectLink(link);
            updateSelectionLabel();
        });
        QObject::connect(viewer_, &ModelViewer::objectMoved, this, [this]() { updateSelectionLabel(); });
        // 팔 링크를 끌어서(IK) 관절 값이 바뀌면 슬라이더와 토크 표시를 따라 갱신하고, 모터가 꺼져 있으면 시뮬레이션도 그 값에서 다시 시작한다
        QObject::connect(viewer_, &ModelViewer::jointValuesDragged, this, [this](const amr::urdf::JointValues& q) {
            panel_->ShowValues(q);
            if (motorsOff_) {
                const amr::Array<int> act = viewer_->Robot().ActuatedJoints();
                for (size_t i = 0; i < act.size(); i++) {
                    const amr::String& n = viewer_->Robot().joints[static_cast<size_t>(act[i])].name;
                    sim_.SetValue(n, q.at(n));
                }
            }
            updateTorques(q);
            updateSelectionLabel();
        });
        loadGizmo();

        panel_ = new JointPanel;
        dock_ = new QDockWidget(QString::fromUtf8("관절"), this);
        dock_->setWidget(panel_);
        addDockWidget(Qt::RightDockWidgetArea, dock_);
        dock_->hide();
        QObject::connect(panel_, &JointPanel::valueEdited, this, [this](const QString& name, double value) {
            if (motorsOff_) sim_.SetValue(amrqt::FromQ(name), value);  // 손으로 옮긴 관절은 속도를 0 으로 다시 시작
        });
        QObject::connect(panel_, &JointPanel::valuesChanged, this, [this]() {
            viewer_->SetJointValues(panel_->Values());
            updateTorques();
            updateSelectionLabel();
        });

        QMenu* file = menuBar()->addMenu(QString::fromUtf8("파일"));
        QAction* open = file->addAction(QString::fromUtf8("열기..."));
        open->setShortcut(QKeySequence::Open);
        QObject::connect(open, &QAction::triggered, this, [this]() {
            const QString p = QFileDialog::getOpenFileName(
                this, QString::fromUtf8("모델 또는 URDF 열기"), QString(),
                QString::fromUtf8("3D 모델 / URDF (*.urdf *.glb *.gltf *.stl *.obj *.dae);;모든 파일 (*)"));
            if (!p.isEmpty()) openFile(p);
        });
        QAction* quit = file->addAction(QString::fromUtf8("종료"));
        quit->setShortcut(QKeySequence::Quit);
        QObject::connect(quit, &QAction::triggered, this, &QWidget::close);
        QMenu* view = menuBar()->addMenu(QString::fromUtf8("보기"));
        QAction* fit = view->addAction(QString::fromUtf8("화면에 맞춤"));
        fit->setShortcut(Qt::Key_F);
        QObject::connect(fit, &QAction::triggered, viewer_, &ModelViewer::FitView);
        QAction* grid = view->addAction(QString::fromUtf8("격자"));
        grid->setCheckable(true);
        grid->setChecked(true);
        QObject::connect(grid, &QAction::toggled, viewer_, &ModelViewer::SetShowGrid);
        QAction* grav = view->addAction(QString::fromUtf8("중력 방향..."));
        QObject::connect(grav, &QAction::triggered, this, [this]() {
            bool ok = false;
            const QString cur = QString("%1, %2, %3").arg(gravity_.x).arg(gravity_.y).arg(gravity_.z);
            const QString text = QInputDialog::getText(this, QString::fromUtf8("중력 가속도"),
                                                       QString::fromUtf8("gx, gy, gz (m/s², 루트 링크 좌표계)\n경사면의 AMR 이면 기울어진 방향을 넣으세요."),
                                                       QLineEdit::Normal, cur, &ok);
            if (!ok) return;
            QString flat = text;
            flat.replace(',', ' ');
            const QStringList p = flat.simplified().split(' ');
            bool a = false, b = false, c = false;
            double gx = 0, gy = 0, gz = 0;
            if (p.size() == 3) {
                gx = p[0].toDouble(&a);
                gy = p[1].toDouble(&b);
                gz = p[2].toDouble(&c);
            }
            if (a && b && c) {
                gravity_ = amr::dynamics::Gravity(gx, gy, gz);
                viewer_->SetGravity(gx, gy, gz);
                sim_.SetGravity(gravity_);
                updateTorques();
            } else {
                QMessageBox::warning(this, QString::fromUtf8("중력 가속도"), QString::fromUtf8("숫자 세 개(gx, gy, gz)를 넣어 주세요."));
            }
        });
        QAction* structure = treeDock_->toggleViewAction();
        structure->setText(QString::fromUtf8("구조 트리"));
        structure->setShortcut(Qt::Key_T);
        view->addAction(structure);
        QAction* frame = view->addAction(QString::fromUtf8("선택 좌표계"));
        frame->setShortcut(Qt::Key_G);
        frame->setCheckable(true);
        frame->setChecked(true);
        QObject::connect(frame, &QAction::toggled, viewer_, &ModelViewer::SetShowSelectionFrame);
        QAction* fall = view->addAction(QString::fromUtf8("중력 낙하 (물체를 놓으면 떨어짐)"));
        fall->setCheckable(true);
        fall->setChecked(!o.noFall);
        viewer_->SetFallEnabled(!o.noFall);
        viewer_->SetGravity(gravity_.x, gravity_.y, gravity_.z);
        QObject::connect(fall, &QAction::toggled, viewer_, &ModelViewer::SetFallEnabled);
        QAction* motors = view->addAction(QString::fromUtf8("모터 끄기 (관절이 중력에 처짐)"));
        motors->setShortcut(Qt::Key_M);
        motors->setCheckable(true);
        simManual_ = o.simulate > 0;
        QObject::connect(motors, &QAction::toggled, this, [this](bool on) { setMotorsOff(on); });
        simTimer_ = new QTimer(this);
        simTimer_->setInterval(16);
        QObject::connect(simTimer_, &QTimer::timeout, this, [this]() {
            double dt = static_cast<double>(simClock_.restart()) / 1000.0;
            if (dt > 0.05) dt = 0.05;
            stepSimulation(dt);
        });
        motorsOff_ = false;
        motors->setChecked(o.motorsOff);
        QAction* resetPos = view->addAction(QString::fromUtf8("오브젝트 위치 초기화"));
        QObject::connect(resetPos, &QAction::triggered, viewer_, [this]() { viewer_->ResetObjectPosition(); });
        QAction* obbs = view->addAction(QString::fromUtf8("OBB 충돌 박스"));
        obbs->setShortcut(Qt::Key_O);
        obbs->setCheckable(true);
        obbs->setChecked(o.obb);
        viewer_->SetShowObbs(o.obb);
        QObject::connect(obbs, &QAction::toggled, viewer_, &ModelViewer::SetShowObbs);
        QAction* boxes = view->addAction(QString::fromUtf8("AABB 충돌 박스"));
        boxes->setShortcut(Qt::Key_B);
        boxes->setCheckable(true);
        boxes->setChecked(o.aabb);
        viewer_->SetShowCollisionBoxes(o.aabb);
        QObject::connect(boxes, &QAction::toggled, viewer_, &ModelViewer::SetShowCollisionBoxes);
        collisionLabel_ = new QLabel;
        statusBar()->addPermanentWidget(collisionLabel_);
        QObject::connect(viewer_, &ModelViewer::collisionsChanged, this, [this]() { updateCollisionLabel(); });
        QObject::connect(viewer_, &ModelViewer::collisionsChanged, this, [this]() { tree_->SetCollidingLinks(viewer_->CollidingLinks()); });
        infoMessage_ = QString::fromUtf8("파일 > 열기 (Ctrl+O) 또는 파일을 창에 끌어다 놓으세요. 왼쪽 드래그: 회전, 오른쪽 드래그: 이동, 휠: 확대, 왼쪽 클릭: 물체 선택");
        statusBar()->showMessage(infoMessage_);
    }

    bool openFile(const QString& path) {
        QString err;
        if (!viewer_->LoadFile(path, &err)) {
            QMessageBox::warning(this, QString::fromUtf8("열 수 없음"), err);
            return false;
        }
        afterLoad(path);
        return true;
    }

    // 명령행에서 받은 관절 값을 적용한다 (각도 관절은 degree 로 받아 rad 로 바꾼다)
    void applyJoints(const Options& o) {
        for (size_t i = 0; i < o.joints.size(); i++) {
            const int ji = viewer_->Robot().FindJoint(o.joints[i].first);
            if (ji < 0) { fprintf(stderr, "경고: 관절 '%s' 이 없습니다\n", o.joints[i].first.c_str()); continue; }
            panel_->SetValue(amrqt::ToQ(o.joints[i].first), viewer_->Robot().joints[static_cast<size_t>(ji)].FromDisplay(o.joints[i].second));
        }
    }

    // 명령행의 --select 링크 이름을 선택한다
    void applySelect(const Options& o) {
        if (o.select.empty() || !viewer_->HasRobot()) return;
        const int li = viewer_->Robot().FindLink(o.select);
        if (li < 0) { fprintf(stderr, "경고: 링크 '%s' 이 없습니다\n", o.select.c_str()); return; }
        tree_->SelectLink(li);
        viewer_->SetSelectedLink(li);
        updateSelectionLabel();
    }

    void applyLidar(const Options& o) {
        if (o.lidarLink.empty() || !viewer_->HasRobot()) return;
        const int li = viewer_->Robot().FindLink(o.lidarLink);
        if (li < 0) { fprintf(stderr, "경고: 링크 '%s' 이 없습니다 (라이다를 달지 않음)\n", o.lidarLink.c_str()); return; }
        amr::sensor::LidarSettings ls;
        ls.channels = o.lidarChannels;
        ls.elevationMin = static_cast<float>(dhRad(o.lidarElev[0]));
        ls.elevationMax = static_cast<float>(dhRad(o.lidarElev[1]));
        viewer_->SetLidar(li, ls, dhMat::Trans(o.lidarMount[0], o.lidarMount[1], o.lidarMount[2]));
    }

    void applyMove(const Options& o) {
        if (o.hasMove) viewer_->SetObjectPosition(o.move[0], o.move[1], o.move[2]);
    }

    // 확인용: 모터를 끈 시뮬레이션을 seconds 초만큼 (1/60 초씩) 진행하고 관절 값을 출력한다
    void runSimulation(double seconds) {
        if (!viewer_->HasRobot() || !motorsOff_) return;
        for (double t = 0; t < seconds - 1e-9; t += 1.0 / 60.0) stepSimulation(1.0 / 60.0);
        const amr::urdf::JointValues q = sim_.Values();
        const amr::urdf::Robot& robot = viewer_->Robot();
        printf("시뮬레이션 %.2f 초 후 (모터 꺼짐, 중력 %.3f, %.3f, %.3f):\n", seconds, gravity_.x, gravity_.y, gravity_.z);
        const amr::Array<int> act = robot.ActuatedJoints();
        for (size_t i = 0; i < act.size(); i++) {
            const amr::urdf::Joint& j = robot.joints[static_cast<size_t>(act[i])];
            const amr::urdf::JointValues::const_iterator it = q.find(j.name);
            if (it == q.end()) continue;
            printf("  %-18s %9.3f %s\n", j.name.c_str(), j.ToDisplay(static_cast<double>(it->second)), j.DisplayUnit());
        }
        printf("  총 에너지 %.4f J (운동 %.4f)\n", sim_.TotalEnergy(), sim_.KineticEnergy());
    }

    ModelViewer* viewer() { return viewer_; }

protected:
    void dragEnterEvent(QDragEnterEvent* e) override {
        if (e->mimeData()->hasUrls()) e->acceptProposedAction();
    }
    void dropEvent(QDropEvent* e) override {
        const QList<QUrl> urls = e->mimeData()->urls();
        if (!urls.isEmpty()) openFile(urls[0].toLocalFile());
    }

private:
    // 모터 끄기: 켜면 지금 관절 값에서 시작해서 중력, 관성, 마찰로 움직인다. 끄면(모터 켜짐) 지금 자세를 유지한다.
    void setMotorsOff(bool on) {
        motorsOff_ = on;
        if (!on) {
            simTimer_->stop();
            return;
        }
        resetSimulation();
        if (!simManual_) {
            simClock_.restart();
            simTimer_->start();
        }
    }

    void resetSimulation() {
        if (!motorsOff_ || !viewer_->HasRobot()) return;
        sim_.Reset(viewer_->Robot(), panel_->Values());
        sim_.SetGravity(gravity_);
        amr::dynamics::DynamicsOptions opt;
        opt.defaultDamping = 0.2;  // URDF 에 감쇠가 없는 관절도 영원히 흔들리지 않도록 약한 점성 감쇠를 준다
        sim_.SetOptions(opt);
    }

    void stepSimulation(double dt) {
        if (!sim_.IsReady()) return;
        sim_.Step(dt);
        const amr::urdf::JointValues q = sim_.Values();
        viewer_->SetJointValues(q);
        panel_->ShowValues(q);
        updateTorques(q);
        updateSelectionLabel();
    }

    // 좌표계 OBJ 를 찾아서 뷰어에 넣는다: 소스의 Data 폴더(개발 중), 실행 파일 옆 Data 폴더(배포) 순서
    void loadGizmo() {
        QStringList dirs;
#ifdef AMR_DATA_DIR
        dirs << QString::fromUtf8(AMR_DATA_DIR);
#endif
        dirs << QCoreApplication::applicationDirPath() + "/Data" << QCoreApplication::applicationDirPath() + "/../Data";
        for (int i = 0; i < dirs.size(); i++) {
            const QString path = QDir(dirs[i]).filePath("Object/gizmo/axes.obj");
            if (!QFileInfo(path).isFile()) continue;
            QString err;
            if (viewer_->LoadGizmo(path, &err)) return;
            fprintf(stderr, "경고: 좌표계 OBJ 를 읽을 수 없습니다 (%s): %s\n", path.toUtf8().constData(), err.toUtf8().constData());
        }
        fprintf(stderr, "경고: Data/Object/gizmo/axes.obj 를 찾을 수 없어 선으로 대신 그립니다\n");
    }

    // 선택한 것의 이름과 자세 (루트 링크 기준, 위치 m, 사람에게는 RPY degree, 데이터는 쿼터니언)
    void updateSelectionLabel() {
        dhMat f;
        if (!viewer_->SelectedFrame(&f)) {
            statusBar()->showMessage(infoMessage_);
            return;
        }
        QString name = QString::fromUtf8("모델");
        if (viewer_->SelectedLink() >= 0 && static_cast<size_t>(viewer_->SelectedLink()) < viewer_->Robot().links.size())
            name = amrqt::ToQ(viewer_->Robot().links[static_cast<size_t>(viewer_->SelectedLink())].name);
        const dhPose pose = dhPose::FromMat(f).Normalized();
        const amr::units::RpyDeg r = amr::units::RpyDegFromQuat(pose.orientation);
        statusBar()->showMessage(QString::fromUtf8("선택 %1 | 위치 %2, %3, %4 m | RPY %5°, %6°, %7° | 쿼터니언 %8, %9, %10, %11  | %12")
                                     .arg(name)
                                     .arg(static_cast<double>(pose.position.x), 0, 'f', 3).arg(static_cast<double>(pose.position.y), 0, 'f', 3).arg(static_cast<double>(pose.position.z), 0, 'f', 3)
                                     .arg(r.roll, 0, 'f', 1).arg(r.pitch, 0, 'f', 1).arg(r.yaw, 0, 'f', 1)
                                     .arg(static_cast<double>(pose.orientation.x), 0, 'f', 3).arg(static_cast<double>(pose.orientation.y), 0, 'f', 3)
                                     .arg(static_cast<double>(pose.orientation.z), 0, 'f', 3).arg(static_cast<double>(pose.orientation.w), 0, 'f', 3)
                                     .arg(viewer_->CanMoveSelection() ? QString::fromUtf8("화살표를 끌어서 이동") : QString::fromUtf8("관절로 움직이는 링크")));
    }

    // 현재 자세와 중력으로 관절별 중력 하중을 다시 계산해서 관절 패널(모터 토크)과 구조 트리(모든 관절)에 보여준다
    void updateTorques() { updateTorques(panel_->Values()); }
    void updateTorques(const amr::urdf::JointValues& q) {
        if (!viewer_->HasRobot()) return;
        const amr::urdf::Robot& robot = viewer_->Robot();
        const amr::dynamics::JointTorques motor = amr::dynamics::GravityTorques(robot, q, gravity_);
        panel_->SetTorques(motor);
        dock_->setWindowTitle(QString::fromUtf8("관절  (중력 %1, %2, %3 m/s²)").arg(gravity_.x, 0, 'g', 5).arg(gravity_.y, 0, 'g', 5).arg(gravity_.z, 0, 'g', 5));

        const amr::Array<amr::dynamics::JointLoad> loads = amr::dynamics::GravityLoads(robot, q, gravity_);
        amr::Map<amr::String, amr::Pair<QString, bool> > notes;
        for (size_t i = 0; i < robot.joints.size(); i++) {
            const amr::urdf::Joint& j = robot.joints[i];
            const amr::dynamics::JointLoad& l = loads[i];
            QString text;
            bool warn = false;
            if (!j.IsMovable()) {
                const double f = amr::Sqrt(l.force[0] * l.force[0] + l.force[1] * l.force[1] + l.force[2] * l.force[2]);
                const double m = amr::Sqrt(l.moment[0] * l.moment[0] + l.moment[1] * l.moment[1] + l.moment[2] * l.moment[2]);
                text = QString::fromUtf8("체결 하중 힘 %1 N, 모멘트 %2 N·m").arg(f, 0, 'f', 2).arg(m, 0, 'f', 2);
            } else if (j.hasMimic) {
                text = QString::fromUtf8("자기 몫 %1 %2 (원본 %3 에 합산)").arg(l.torque, 0, 'f', 3).arg(j.TorqueUnit(), amrqt::ToQ(j.mimicJoint));
            } else {
                const double tau = motor.find(j.name)->second;
                text = QString::fromUtf8("모터 %1 %2").arg(tau, 0, 'f', 3).arg(j.TorqueUnit());
                if (j.effort > 0.0f && amr::Abs(tau) > static_cast<double>(j.effort)) {
                    text += QString::fromUtf8("  (한계 %1 초과)").arg(static_cast<double>(j.effort), 0, 'g', 4);
                    warn = true;
                }
            }
            notes[j.name] = amr::MakePair(text, warn);
        }
        tree_->SetJointNotes(notes);
    }

    void updateCollisionLabel() {
        if (!viewer_->HasRobot()) {
            collisionLabel_->clear();
            return;
        }
        const QStringList d = viewer_->CollisionDescriptions();
        const int cand = static_cast<int>(viewer_->AabbCandidates().size());
        if (d.isEmpty()) {
            collisionLabel_->setText(QString::fromUtf8("충돌 없음 (AABB 후보 %1건)").arg(cand));
            collisionLabel_->setStyleSheet(QString());
        } else {
            collisionLabel_->setText(QString::fromUtf8("OBB 충돌 %1건: %2 (AABB 후보 %3건)").arg(d.size()).arg(d.join(", ")).arg(cand));
            collisionLabel_->setStyleSheet("color: #d33; font-weight: bold;");
        }
    }

    void afterLoad(const QString& path) {
        if (viewer_->HasRobot()) {
            tree_->SetRobot(viewer_->Robot());
            tree_->SetCollidingLinks(viewer_->CollidingLinks());
            panel_->SetRobot(viewer_->Robot());
            updateTorques();
            dock_->setVisible(!panel_->IsEmpty());
        } else {
            if (viewer_->HasScene()) tree_->SetScene(viewer_->StaticScene());
            else tree_->Clear();
            dock_->hide();
        }
        treeDock_->setVisible(tree_->topLevelItemCount() > 0);
        if (treeDock_->isVisible()) resizeDocks(QList<QDockWidget*>() << treeDock_, QList<int>() << 520, Qt::Horizontal);
        updateCollisionLabel();
        QString msg = path + "  -  " + viewer_->Summary();
        if (!viewer_->Warnings().isEmpty()) msg += QString::fromUtf8("  (경고 %1건: %2)").arg(viewer_->Warnings().size()).arg(viewer_->Warnings()[0]);
        resetSimulation();
        infoMessage_ = msg;
        statusBar()->showMessage(msg);
        updateSelectionLabel();
        setWindowTitle(QString("AMR RobotArm Viewer - ") + path);
    }

    ModelViewer* viewer_;
    JointPanel* panel_;
    QDockWidget* dock_;
    StructurePanel* tree_;
    QDockWidget* treeDock_;
    QLabel* collisionLabel_;
    amr::dynamics::Simulator sim_;
    QTimer* simTimer_;
    QElapsedTimer simClock_;
    bool motorsOff_;
    bool simManual_;   // --simulate: 타이머 없이 runSimulation 으로만 진행
    QString infoMessage_;   // 선택이 없을 때 상태바에 보이는 파일 정보
    amr::dynamics::Gravity gravity_;
};

}  // namespace

int main(int argc, char** argv) {
    InitConsole();
    Options o;
    const int pr = ParseArgs(argc, argv, &o);
    if (pr == 1) return 0;
    if (pr == 2) return 2;
    if (o.info) return PrintInfo(o);

    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    MainWindow w(o);
    w.show();
    if (!o.file.empty()) {
        if (!w.openFile(QString::fromLocal8Bit(o.file.c_str()))) {
            if (!o.shot.empty() || !o.shotWindow.empty()) return 1;
        } else {
            w.applyJoints(o);
            w.applySelect(o);
            w.applyMove(o);
            w.applyLidar(o);
        }
    }
    if (!o.shot.empty() || !o.shotWindow.empty()) {
        const amr::String shot = o.shot, shotWindow = o.shotWindow;
        ModelViewer* v = w.viewer();
        MainWindow* win = &w;
        const int cx = o.clickX, cy = o.clickY;
        const bool settle = o.settle;
        const double simSeconds = o.simulate;
        const amr::String lidarCsv = o.lidarCsv;
        const int d0x = o.dragX0, d0y = o.dragY0, d1x = o.dragX1, d1y = o.dragY1;
        QTimer::singleShot(600, &app, [&app, v, win, shot, shotWindow, cx, cy, d0x, d0y, d1x, d1y, settle, simSeconds, lidarCsv]() {
            bool ok = true;
            if (d0x >= 0 && d0y >= 0 && d1x >= 0 && d1y >= 0) {
                const QPoint p0(d0x, d0y), p1(d1x, d1y);
                if (!v->HasSelection()) {  // 선택한 것이 없으면 시작점을 먼저 클릭해서 고른다
                    QMouseEvent pick(QEvent::MouseButtonPress, p0, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                    QMouseEvent pickUp(QEvent::MouseButtonRelease, p0, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
                    QApplication::sendEvent(v, &pick);
                    QApplication::sendEvent(v, &pickUp);
                }
                QMouseEvent press(QEvent::MouseButtonPress, p0, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                QApplication::sendEvent(v, &press);
                for (int k = 1; k <= 8; k++) {  // 중간 지점을 거쳐서 끈다
                    const QPoint pk(p0.x() + (p1.x() - p0.x()) * k / 8, p0.y() + (p1.y() - p0.y()) * k / 8);
                    QMouseEvent mv(QEvent::MouseMove, pk, Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
                    QApplication::sendEvent(v, &mv);
                }
                QMouseEvent release(QEvent::MouseButtonRelease, p1, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
                QApplication::sendEvent(v, &release);
                const dhVector pos = v->ObjectPosition();
                printf("드래그 (%d,%d)->(%d,%d) -> 오브젝트 위치 %.4f, %.4f, %.4f m\n", d0x, d0y, d1x, d1y, static_cast<double>(pos.x), static_cast<double>(pos.y), static_cast<double>(pos.z));
                dhMat sel;
                if (v->SelectedLink() >= 0 && v->SelectedFrame(&sel)) {  // 팔 링크를 끌었다면 IK 로 옮겨진 링크 위치와 관절 값
                    printf("선택 링크 %d 원점 %.4f, %.4f, %.4f m\n", v->SelectedLink(), static_cast<double>(sel.v[12]), static_cast<double>(sel.v[13]), static_cast<double>(sel.v[14]));
                    const amr::Array<int> act = v->Robot().ActuatedJoints();
                    for (size_t i = 0; i < act.size(); i++) {
                        const amr::urdf::Joint& j = v->Robot().joints[static_cast<size_t>(act[i])];
                        printf("  %s = %.3f %s\n", j.name.c_str(), j.ToDisplay(static_cast<double>(v->Robot().EffectiveJointValue(v->CurrentJointValues(), act[i]))), j.DisplayUnit());
                    }
                }
                v->repaint();
            }
            if (cx >= 0 && cy >= 0) {  // 실제 마우스 이벤트를 보내서 클릭 선택까지 한꺼번에 확인한다
                const QPoint p(cx, cy);
                QMouseEvent press(QEvent::MouseButtonPress, p, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                QMouseEvent release(QEvent::MouseButtonRelease, p, Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
                QApplication::sendEvent(v, &press);
                QApplication::sendEvent(v, &release);
                printf("클릭 (%d,%d) -> 선택 링크 %d%s\n", cx, cy, v->SelectedLink(), v->ModelSelected() ? " (모델)" : "");
                v->repaint();
            }
            if (v->LidarEnabled()) {  // 확인용: 스캔 요약
                const amr::sensor::LidarScan& sc = v->ScanLidar();
                size_t hits = 0;
                float lo = 1e30f, hi = 0;
                for (size_t i = 0; i < sc.size(); i++)
                    if (sc.IsHit(i)) {
                        hits++;
                        if (sc.ranges[i] < lo) lo = sc.ranges[i];
                        if (sc.ranges[i] > hi) hi = sc.ranges[i];
                    }
                printf("라이다: 광선 %zu 개 중 %zu 개가 맞음", sc.size(), hits);
                if (hits) printf(", 거리 %.3f ~ %.3f m", static_cast<double>(lo), static_cast<double>(hi));
                printf("\n");
                if (!lidarCsv.empty()) {
                    amr::String csv = "index,channel,azimuth_deg,elevation_deg,hit,range_m,x,y,z,link\n";
                    for (size_t i = 0; i < sc.size(); i++) {
                        const bool hit = sc.IsHit(i);
                        csv += amr::Format("%zu,%zu,%.4f,%.4f,%d,", i, sc.Channel(i), dhDeg(static_cast<double>(sc.Angle(i))),
                                           dhDeg(static_cast<double>(sc.Elevation(sc.Channel(i)))), hit ? 1 : 0);
                        if (hit)
                            csv += amr::Format("%.5f,%.5f,%.5f,%.5f,%s\n", static_cast<double>(sc.ranges[i]), static_cast<double>(sc.points[i].x),
                                               static_cast<double>(sc.points[i].y), static_cast<double>(sc.points[i].z),
                                               v->Robot().links[static_cast<size_t>(sc.targetIds[i])].name.c_str());
                        else
                            csv += "inf,,,,\n";
                    }
                    if (amr::WriteFileText(lidarCsv, csv)) printf("스캔을 저장했습니다: %s\n", lidarCsv.c_str());
                    else fprintf(stderr, "스캔 저장 실패: %s\n", lidarCsv.c_str());
                }
            }
            if (simSeconds > 0) win->runSimulation(simSeconds);
            if (settle) {  // 낙하가 끝날 때까지 시간을 진행시킨다
                const double z0 = static_cast<double>(v->ObjectPosition().z);
                const double t = v->SimulateFall(10.0);
                printf("낙하: 높이 %.4f m -> %.4f m, %.2f 초 걸림 (바닥 높이 %.4f m)\n", z0, static_cast<double>(v->ObjectPosition().z), t, static_cast<double>(v->ObjectBottom()));
                v->repaint();
            }
            if (!shot.empty()) {
                const QImage img = v->grabFramebuffer();
                ok = !img.isNull() && img.save(QString::fromLocal8Bit(shot.c_str()));
                if (!ok) fprintf(stderr, "화면 저장 실패: %s\n", shot.c_str());
            }
            if (!shotWindow.empty()) {
                const QImage img = win->grab().toImage();
                const bool ok2 = !img.isNull() && img.save(QString::fromLocal8Bit(shotWindow.c_str()));
                if (!ok2) fprintf(stderr, "화면 저장 실패: %s\n", shotWindow.c_str());
                ok = ok && ok2;
            }
            app.exit(ok ? 0 : 1);
        });
    }
    return app.exec();
}

#else  // Qt 없이 빌드: 콘솔 모드

int main(int argc, char** argv) {
    InitConsole();
    Options o;
    const int pr = ParseArgs(argc, argv, &o);
    if (pr == 1) return 0;
    if (pr == 2) return 2;
    if (o.file.empty()) {
        PrintUsage(argv[0]);
        printf("\n(이 빌드는 Qt 없이 만들어져 3D 창이 없습니다. -DAMR_BUILD_QT=ON 으로 다시 빌드하세요.)\n");
        return 0;
    }
    return PrintInfo(o);
}

#endif
