// Project Simul (AMR + 로봇 팔) 진입점.
//   simul_main [옵션] [파일]      파일: .urdf .glb .gltf .stl .obj .dae
//   Qt 로 빌드했으면 3D 뷰어 창을 띄우고, 아니면 파일 정보만 출력한다 (--info 와 같음).
#include "CEnv/Sun.h"
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
#include "CMarker/Marker.h"
#include "CProject/Project.h"
#ifdef SIMUL_HAS_QT
#include "ProjectPack.h"
#endif
#ifdef SIMUL_HAS_PYTHON
#include "CorePy/PyRuntime.h"
#endif

namespace {

// Windows 콘솔은 기본이 로컬 코드 페이지라 UTF-8 한글이 깨진다. 출력만 UTF-8 로 바꾼다.
void InitConsole() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
}

struct Options {
    bool showFps = false;   // 화면에 FPS 와 그리기 시간을 표시
    simul::String file;
    simul::Array<simul::String> moreFiles;  // 둘째 파일부터 (모두 한 장면에 올린다)
    struct Place { simul::String model; double v[4]; };   // --place 모델=x,y,z[,회전도]
    simul::Array<Place> places;
    simul::Array<simul::String> connects;   // --connect 자식=부모링크[:종류[:축 x,y,z[:최소,최대]]]
    struct Scale { simul::String model; double factor; };   // --scale 모델=배율
    simul::Array<Scale> scales;
    struct Shape { char kind; simul::String name; double a[3]; double pos[3]; };   // --add-box/--add-cylinder/--add-sphere 이름=크기[@x,y,z]
    simul::Array<Shape> shapes;
    simul::String floorModel;   // --floor 모델: 바닥으로 지정할 모델 이름
    bool onlyView;             // --only-view: 메뉴/도크/상태바를 숨기고 3D 화면만
    bool hideMenu;             // --hide-menu: 메뉴만 숨김
    simul::String exportUrdf;  // --export-urdf 파일: 장면을 URDF 하나로 저장하고 끝낸다
    int exportFormat = 0;      // --export-mesh obj|stl|glb (기본 obj)
    simul::String saveProjectDir;   // --save-project 폴더: 지금 장면과 등록한 python 을 프로젝트 폴더로 저장하고 끝낸다
    simul::String exportDir;   // --export-dir 폴더: 장면을 폴더(폴더이름.urdf + obj/)로 저장하고 끝낸다
    double uiScale = 1.0;      // --ui-scale: 3D 화면 안 UI 크기 배율
    int shading;               // 조명 방식 0 Lambert, 1 Blinn-Phong, 2 PBR (기본)
    bool info;                 // 창 없이 정보만 출력
    bool pyEnabled;            // --python: 내장 Python 을 시작 때 띄운다 (안 주면 메뉴에서 처음 시작할 때)
    simul::String pyMain;      // --python-main: Main.py 경로 (기본 Data/Python/Main.py)
    simul::Array<simul::Pair<simul::String, simul::String> > pyScripts;   // --python-script 파일[:오브젝트,...]
    double pyHz;               // --python-hz: 초당 틱 수
    int pySteps;               // --python-steps N: 시작하자마자 N 틱을 돌린 뒤 멈춘다 (확인용)
    bool objPanel;             // --object-panel: 오브젝트 / 관절 값 도크를 연 채로 시작
    bool pyEditor;             // --python-editor: 오른쪽 Python 편집기를 연 채로 시작
    bool pyRun;                // --python-run: 시작하자마자 실시간으로 돌린다
    simul::String projectFile; // --project 파일: project.simul 을 열어 장면, 스크립트 등록, 자동 시작까지 한다
    simul::String packProject; // --pack 프로젝트: 그 프로젝트를 이 프로그램의 실행 파일 뒤에 붙여 하나의 실행 파일로 만든다
    simul::String packOut;     // -o / --output: --pack 의 출력 파일
    simul::String embeddedDir; // (내부) 실행 파일에 붙은 프로젝트를 푼 폴더
    simul::String makeMarker;  // --make-marker 명세: ArUco/ChArUco 마커 메시와 URDF 를 만들고 끝낸다
    simul::String makeMarkerDir;  // --marker-dir: 마커 파일을 쓸 폴더 (기본 현재 폴더)
    bool aabb;                 // AABB 충돌 박스를 처음부터 표시
    bool obb;                  // OBB 충돌 박스를 처음부터 표시
    bool torque;               // 관절별 중력 토크를 출력 (--info 와 같이)
    double gravity[3];         // 중력 가속도 (m/s^2, 루트 링크 좌표계)
    simul::String shot;          // 비어 있지 않으면 3D 화면을 PNG 로 저장하고 종료
    simul::String shotWindow;    // 비어 있지 않으면 창 전체(패널 포함)를 PNG 로 저장하고 종료
    simul::String select;        // 시작할 때 선택(강조)할 링크 이름
    int dragX0, dragY0, dragX1, dragY1;  // --drag: 눌렀다가 끌고 떼는 마우스 동작 (-1 이면 안 함)
    bool motorsOff;            // --motors-off: 처음부터 모터를 끈다 (관절이 중력에 처짐)
    double simulate;           // --simulate 초: 모터를 끄고 그 시간만큼 시뮬레이션한 뒤 관절 값을 출력 (확인용, 실시간이 아님)
    bool noFall;               // --no-fall: 중력 낙하를 끈다
    bool settle;               // --settle: 낙하가 끝날 때까지 진행한 뒤 위치를 출력 (확인용)
    simul::String lidarLink;     // --lidar: 2D 라이다를 달 링크 이름 (비어 있으면 없음)
    float lidarMount[3];       //   그 링크 좌표계에서의 센서 위치 (m)
    simul::String lidarCsv;      // --lidar-csv: 스캔 결과를 CSV 로 저장할 파일
    int lidarChannels;         // --lidar-channels: 수직 채널 수 (1 이면 2D)
    float lidarElev[2];        //   첫/마지막 채널의 고각 (도)
    bool noSun;                 // --no-sun: 태양 조명을 끈다 (기본은 켜짐: 오늘 10:00, 그림자 포함)
    bool sunOn;                 // --sun / --sun-now / --sun-site: 태양 조명을 켠다
    simul::env::SunTime sunTime;   // --sun 날짜시각 (현지). 지정하지 않으면 지금 시각
    bool sunNow;                // --sun-now: 지금 시각 (아니면 오늘 날짜의 10:00)
    bool sunTimeSet;            // --sun 에 날짜를 줬는가
    simul::env::SunSite sunSite;   // --sun-site 위도,경도[,UTC오프셋[,월드+y방위]]
    bool sunOffsetSet;          // UTC 오프셋을 직접 줬는가 (안 줬으면 서울 기본이면 +9, 위치를 줬으면 경도/15 반올림)
    bool sunSiteSet;
    bool viewSet;               // --view-angle 방위각,고도각 (도)
    double viewYaw, viewPitch;
    bool sunGuide;              // --sun-guide: 태양 위치 표시(선, 방위 글자, 방위각/고도 글자)를 켠다 (기본은 꺼짐)
    bool noSky;                 // --no-sky: 3D 화면의 하늘(구름)을 끄고 단색 배경
    bool noShadow;              // --no-shadow: 3D 화면의 그림자를 끈다
    simul::String cameraLink;    // --camera: 카메라를 달 링크 이름 (비어 있으면 없음)
    float cameraMount[3];      //   그 링크 좌표계에서의 센서 위치 (m). 센서 +x 가 보는 방향
    int cameraW, cameraH;      // --camera-size 가로x세로
    float cameraFov;           // --camera-fov 가로 시야각 (도)
    float cameraRange[2];      // --camera-range 최소,최대 (m)
    float cameraDistort[5];    // --camera-distort k1,k2,p1,p2,k3: 렌즈 왜곡 (OpenCV 와 같은 뜻, 0 = 없음)
    bool cameraDistortSet;
    float cameraNoise[2];      // --camera-noise 색,깊이 (색은 0..1 비율, 깊이는 m). 0,0 = 잡음 없음
    bool cameraNoiseSet;
    int cameraAa;              // --camera-aa 1~4: 안티앨리어싱 (픽셀을 n x n 으로 나눠 찍어 평균, 1 = 끔)
    simul::String cameraSave;  // --camera-save: 찍은 영상을 저장할 PNG 파일 (깊이는 이름_depth.png, 16비트 mm 는 이름_depth16.png)
    bool hasMove;
    float move[3];             // --move: 시작할 때 오브젝트를 옮길 위치 (월드, m)
    int clickX, clickY;        // --click: 3D 화면의 이 픽셀을 마우스로 클릭한 것처럼 처리 (-1 이면 안 함)
    simul::Array<simul::Pair<simul::String, double> > joints;           // --joint 이름=값 (revolute: 도, prismatic: m)
    simul::Array<simul::Pair<simul::String, simul::String> > packages;    // --package 이름=폴더
    int width, height;
    Options() : onlyView(false), hideMenu(false), shading(2), info(false), pyEnabled(false), pyHz(60.0), pySteps(0), objPanel(false), pyEditor(false), pyRun(false), aabb(false), obb(false), torque(false), dragX0(-1), dragY0(-1), dragX1(-1), dragY1(-1), motorsOff(false), simulate(0), noFall(false), settle(false), hasMove(false), clickX(-1), clickY(-1), width(1100), height(720) {
        lidarMount[0] = lidarMount[1] = lidarMount[2] = 0;
        lidarChannels = 1;
        lidarElev[0] = lidarElev[1] = 0;
        cameraMount[0] = cameraMount[1] = cameraMount[2] = 0;
        sunOn = false;
        noSun = false;
        sunTimeSet = false;
        sunNow = false;
        sunOffsetSet = false;
        sunSiteSet = false;
        noShadow = false;
        noSky = false;
        sunGuide = false;
        viewSet = false;
        viewYaw = viewPitch = 0.0;
        cameraW = 320;
        cameraH = 240;
        cameraFov = 60.0f;
        cameraRange[0] = 0.05f;
        cameraRange[1] = 10.0f;
        cameraAa = 1;
        for (int i = 0; i < 5; i++) cameraDistort[i] = 0.0f;
        cameraDistortSet = false;
        cameraNoise[0] = 0.02f;
        cameraNoise[1] = 0.005f;
        cameraNoiseSet = false;
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -simul::dynamics::kStandardGravity;
    }
};

void PrintUsage(const char* exe) {
    printf(
        "사용법: %s [옵션] [파일]\n"
        "  파일               .urdf .glb .gltf .stl .obj .dae (생략하면 빈 뷰어)\n"
        "  --info             창을 띄우지 않고 파일 정보만 출력\n"
        "  --python           내장 Python 을 시작 때 띄운다 (Data/Python/Main.py 를 올린다. 메뉴 Python 으로도 된다)\n"
        "  --python-script 파일[:오브젝트,...]  Main.py 에 등록할 외부 python 파일과, 그 파일의 클래스가 움직일 수 있는 오브젝트 (여러 번 가능, --python 을 포함)\n"
        "  --python-main 파일  Main.py 대신 쓸 파일    --python-hz N  초당 틱 수 (기본 60)\n"
        "  --python-editor    오른쪽 Python 편집기를 연 채로 시작 (메뉴 Python > Python 편집기, Ctrl+E)\n"
        "  --save-project 폴더  지금 장면과 등록한 python 을 프로젝트 폴더(project.simul, models, scripts)로 저장하고 끝낸다\n"
        "  --object-panel     오브젝트 / 관절 값 도크를 연 채로 시작 (메뉴 Python > 오브젝트 / 관절 값, Ctrl+J)\n"
        "  --python-run       시작하자마자 시뮬레이션을 실시간으로 돌린다    --python-steps N  N 틱을 바로 돌리고 멈춘다 (확인용)\n"
        "  --project 파일     project.simul 을 연다 (모델 배치, python 등록과 권한, 자동 시작까지)\n"
        "  --pack 프로젝트 [-o 출력]  프로젝트 폴더를 이 프로그램 뒤에 붙여 하나의 실행 파일로 만든다 (실행하면 그 프로젝트가 자동으로 열려 돌아간다)\n"
        "  --make-marker 명세 ArUco/ChArUco 마커를 만들어 STL 2개 + URDF 로 쓰고 끝낸다 (--marker-dir 폴더 기본 .)\n"
        "                     aruco,사전,id,한변(m)  예: aruco,4x4,0,0.15\n"
        "                     charuco,사전,가로칸,세로칸,칸길이(m),마커한변(m)[,첫id]  예: charuco,4x4,5,7,0.04,0.03\n"
        "                     사전: 4x4 5x5 6x6 7x7 (OpenCV DICT_nXn_1000 과 같은 마커)\n"
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
        "  파일을 여러 개 주면 모두 한 장면에 올린다.  --place 모델=x,y,z[,회전도]  모델 위치 (모델 이름은 파일 이름)\n"
        "  --fps                      화면에 FPS 와 그리기 시간 표시\n"
        "  --ui-scale X               3D 화면 안 메뉴/트리/패널 크기 배율 (기본 1.0, 실행 중 Ctrl+= / Ctrl+- / Ctrl+0)\n"
        "  --export-urdf 파일 열린 모델들(--place 반영)을 URDF 하나로 합쳐 저장하고 끝낸다 (창 없이 쓰려면 QT_QPA_PLATFORM=offscreen)\n"
        "  --connect 자식=부모링크[:종류[:x,y,z[:최소,최대]]]  모델을 다른 모델의 링크에 관절로 연결 (종류 fixed/revolute/continuous/prismatic, 기본 fixed, 축 기본 0,0,1, 회전 한계는 도)\n"
        "                  예: --connect gripper=tool:revolute:0,0,1:-90,90   (지금 자리 그대로 붙는다. 여러 번 쓸 수 있다)\n"
        "  --export-dir 폴더 같은 장면을 폴더로 저장하고 끝낸다: 폴더/폴더이름.urdf + 폴더/obj/ (GLB 는 OBJ+MTL+텍스처로 바꿔 넣어 색이 유지된다)\n"
        "  --export-mesh obj|stl|glb  --export-dir 에서 GLB 를 넣는 형식 (기본 obj. stl 은 호환 최고지만 색이 URDF 로만 남고 용량이 큼, glb 는 변환 없이 복사해 가장 작다)\n"
        "  --floor 모델        그 모델을 바닥으로 지정 (다른 모델이 z=0 평면 대신 그 윗면 위로 떨어져 멈춘다). 예: --floor table\n"
        "  --scale 모델=배율   모델 전체 크기를 원래의 몇 배로 (길이와 질량이 함께 바뀐다). 예: --scale arm=0.5\n"
        "  --add-box 이름=가로,세로,높이[@x,y,z]  / --add-cylinder 이름=반지름,길이[@x,y,z] / --add-sphere 이름=반지름[@x,y,z]  파일 없이 기본 도형을 장면에 올린다 (여러 번 가능)\n"
        "  --only-view / --hide-menu  처음부터 3D 화면만 / 메뉴만 숨김 (실행 중에는 F11 / F10, 3D 만 보기는 Esc 로도 돌아옴)\n"
        "  --shading 방식        조명 방식: lambert, phong, pbr (기본). 실행 중에는 보기 -> 조명 방식에서 바꾼다\n"
        "  (라이다는 URDF 의 <gazebo><sensor type=\"ray\"> 에 적으면 열기만 해도 달린다. 아래 옵션은 URDF 에 없을 때/덮어쓸 때)\n"
        "  --lidar 링크[,x,y,z]  그 링크에 2D 라이다를 달아 스캔한 점(빨강)과 광선을 표시. x,y,z 는 링크 좌표계의 센서 위치(m).\n"
        "                     센서가 붙은 링크 자신을 뺀 나머지 메시를 스캔하고, 맞은 점 수와 거리 범위를 출력\n"
        "  --lidar-channels 개수,최소고각,최대고각  (도) 수직 채널을 늘려 3D 라이다로 (예: 16,-15,15). 큰 메시에서는 느리다\n"
        "  --camera 링크[,x,y,z]  그 링크에 카메라를 달고 시야 절두체(노랑)와 영상 창을 표시. 센서 +x 가 보는 방향, +y 왼쪽, +z 위\n"
        "  --camera-size 가로x세로  --camera-fov 도(가로 시야각)  --camera-range 최소,최대  (기본 320x240, 60도, 0.05~10 m)\n"
        "  --camera-distort k1,k2,p1,p2,k3  카메라 렌즈 왜곡 (OpenCV 와 같음. 예: -0.3,0.1,0,0,0 = 통 모양. URDF 의 <distortion> 보다 우선)\n"
        "  --camera-noise 색,깊이  카메라 잡음 (색은 0~1 비율, 깊이는 m. 기본 0.02,0.005. 0,0 이면 잡음 없음. URDF 의 <noise> 보다 우선)\n"
        "  --camera-aa 1~4  카메라 안티앨리어싱 (픽셀을 n x n 으로 나눠 찍어 평균. 1 = 끔, 시간은 n^2 배)\n"
        "  --camera-save 파일.png  (--camera 와 함께) 영상을 저장: 컬러 PNG, 이름_depth.png (회색, 가까울수록 밝음), 이름_depth16.png (16비트, mm)\n"
        "  --sun [YYYY-MM-DDTHH:MM]  태양 조명을 켠다 (현지 시각, 생략하면 오늘 날짜의 10:00). 해 위치는 시각과 위치로 계산하고 월드 +x 가 동, +y 가 북\n"
        "  --sun-now (지금 시각)  --sun-site 위도,경도[,UTC오프셋[,월드+y의 방위]]  지금 시각 / 위치 (기본 서울 37.5665,126.978, +9). 센서 카메라 영상에도 같은 태양과 그림자가 적용된다\n"
        "  --no-sun  태양 조명을 끈다 (기본은 켜짐: 오늘 날짜 10:00 의 태양과 그림자)  --no-shadow  3D 화면의 그림자를 끈다  --no-sky  하늘과 구름을 끄고 단색 배경  --sun-guide  태양 위치 표시(태양 선, 동서남북, 방위각/고도 글자, 끌어서 시각 선택)를 켠다\n"
        "  --view-angle 방위각,고도각  시작 시점(도, 고도각 -89~89, 음수면 올려다봄)\n"
        "  --lidar-csv 파일.csv  (--lidar 와 함께) 스캔 결과를 CSV 로 저장: 광선 번호, 채널, 방위각(도), 고각(도), 맞음(1/0), 거리(m), 맞은 점 x,y,z(월드), 맞은 링크\n"
        "  --click x,y        3D 화면의 (x,y) 픽셀을 왼쪽 클릭한 것처럼 선택 (--shot 과 함께 확인용)\n"
        "  --size 가로x세로   창 크기 (기본 1100x720)\n"
        "  -h, --help         도움말\n",
        exe);
}

bool SplitPair(const simul::String& s, simul::String* a, simul::String* b) {
    const size_t eq = s.find('=');
    if (eq == simul::String::npos || eq == 0) return false;
    *a = s.substr(0, eq);
    *b = s.substr(eq + 1);
    return true;
}

// 0: 계속, 1: 정상 종료(도움말), 2: 인자 오류
int ParseArgs(int argc, char** argv, Options* o) {
    for (int i = 1; i < argc; i++) {
        const simul::String a = argv[i];
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
        if (a == "--project" && hasNext) { o->projectFile = argv[++i]; continue; }
        if (a == "--save-project" && hasNext) { o->saveProjectDir = argv[++i]; continue; }
        if (a == "--pack" && hasNext) { o->packProject = argv[++i]; continue; }
        if ((a == "-o" || a == "--output") && hasNext) { o->packOut = argv[++i]; continue; }
        if (a == "--python") { o->pyEnabled = true; continue; }
        if (a == "--python-editor") { o->pyEditor = true; continue; }
        if (a == "--object-panel") { o->objPanel = true; continue; }
        if (a == "--python-run") { o->pyEnabled = true; o->pyRun = true; continue; }
        if (a == "--python-main" && hasNext) { o->pyEnabled = true; o->pyMain = argv[++i]; continue; }
        if (a == "--python-hz" && hasNext) {
            o->pyHz = atof(argv[++i]);
            if (!(o->pyHz >= 1.0 && o->pyHz <= 1000.0)) { fprintf(stderr, "--python-hz 는 1~1000 입니다\n"); return false; }
            continue;
        }
        if (a == "--python-steps" && hasNext) {
            o->pySteps = atoi(argv[++i]);
            if (o->pySteps < 1) { fprintf(stderr, "--python-steps 는 1 이상입니다\n"); return false; }
            o->pyEnabled = true;
            continue;
        }
        if (a == "--python-script" && hasNext) {
            simul::String spec = argv[++i], file = spec, objs;
            for (size_t k = spec.size(); k-- > 1;) {   // 마지막 ':' 뒤에 경로 구분자가 없으면 오브젝트 목록 (C:\... 의 드라이브 콜론은 건드리지 않는다)
                if (spec[k] != ':') continue;
                const simul::String tail = spec.substr(k + 1);
                if (tail.find('/') == simul::String::npos && tail.find('\\') == simul::String::npos) { file = spec.substr(0, k); objs = tail; }
                break;
            }
            o->pyScripts.push_back(simul::MakePair(file, objs));
            o->pyEnabled = true;
            continue;
        }
        if (a == "--make-marker" && hasNext) { o->makeMarker = argv[++i]; continue; }
        if (a == "--marker-dir" && hasNext) { o->makeMarkerDir = argv[++i]; continue; }
        if (a == "--fps") { o->showFps = true; continue; }
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
            const simul::String spec = argv[++i];
            const size_t comma = spec.find(',');
            o->lidarLink = spec.substr(0, comma);
            if (comma != simul::String::npos &&
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
        if (a == "--camera" && hasNext) {   // 링크 또는 링크,x,y,z
            const simul::String spec = argv[++i];
            const size_t comma = spec.find(',');
            o->cameraLink = spec.substr(0, comma);
            if (comma != simul::String::npos && sscanf(spec.c_str() + comma + 1, "%f,%f,%f", &o->cameraMount[0], &o->cameraMount[1], &o->cameraMount[2]) != 3) {
                fprintf(stderr, "--camera 는 링크이름 또는 링크이름,x,y,z 형식입니다\n");
                return false;
            }
            continue;
        }
        if (a == "--camera-size" && hasNext) {
            if (sscanf(argv[++i], "%dx%d", &o->cameraW, &o->cameraH) != 2 || o->cameraW < 1 || o->cameraH < 1 || o->cameraW > 4096 || o->cameraH > 4096) {
                fprintf(stderr, "--camera-size 는 가로x세로 형식입니다 (1~4096, 예: 640x480)\n");
                return false;
            }
            continue;
        }
        if (a == "--camera-fov" && hasNext) {
            o->cameraFov = static_cast<float>(atof(argv[++i]));
            if (!(o->cameraFov > 1.0f && o->cameraFov < 179.0f)) { fprintf(stderr, "--camera-fov 는 1~179 도입니다\n"); return false; }
            continue;
        }
        if (a == "--camera-range" && hasNext) {
            if (sscanf(argv[++i], "%f,%f", &o->cameraRange[0], &o->cameraRange[1]) != 2 || o->cameraRange[0] < 0 || o->cameraRange[1] <= o->cameraRange[0]) {
                fprintf(stderr, "--camera-range 는 최소,최대 형식입니다 (최대가 더 커야 함, 예: 0.05,10)\n");
                return false;
            }
            continue;
        }
        if (a == "--camera-distort" && hasNext) {
            const simul::String v = argv[++i];
            float f[5] = {0, 0, 0, 0, 0};
            int n = 0;
            const char* c = v.c_str();
            while (*c && n < 5) {
                char* end = nullptr;
                f[n++] = static_cast<float>(strtod(c, &end));
                if (end == c) { n = 99; break; }
                c = end;
                if (*c == ',') c++;
                else break;
            }
            if (n < 1 || n > 5 || *c) { fprintf(stderr, "--camera-distort 는 k1,k2,p1,p2,k3 형식입니다 (숫자 1~5개, 모자라면 0. 예: -0.3,0.1)\n"); return false; }
            for (int j = 0; j < 5; j++) o->cameraDistort[j] = f[j];
            o->cameraDistortSet = true;
            continue;
        }
        if (a == "--camera-noise" && hasNext) {
            float c = 0.0f, d = 0.0f;
            if (sscanf(argv[++i], "%f,%f", &c, &d) != 2 || c < 0.0f || d < 0.0f) { fprintf(stderr, "--camera-noise 는 색,깊이 형식입니다 (0 이상, 예: 0.02,0.005)\n"); return false; }
            o->cameraNoise[0] = c;
            o->cameraNoise[1] = d;
            o->cameraNoiseSet = true;
            continue;
        }
        if (a == "--camera-aa" && hasNext) {
            o->cameraAa = atoi(argv[++i]);
            if (o->cameraAa < 1 || o->cameraAa > 4) { fprintf(stderr, "--camera-aa 는 1~4 입니다 (1 = 끔, n 이면 픽셀을 n x n 으로 나눠 찍는다)\n"); return false; }
            continue;
        }
        if (a == "--camera-save" && hasNext) { o->cameraSave = argv[++i]; continue; }
        if (a == "--only-view") { o->onlyView = true; continue; }
        if (a == "--connect" && hasNext) { o->connects.push_back(argv[++i]); continue; }
        if (a == "--export-urdf" && hasNext) { o->exportUrdf = argv[++i]; continue; }
        if (a == "--export-dir" && hasNext) { o->exportDir = argv[++i]; continue; }
        if (a == "--export-mesh" && hasNext) {
            const simul::String v = argv[++i];
            if (v != "obj" && v != "stl" && v != "glb") { fprintf(stderr, "--export-mesh 는 obj, stl, glb 중 하나입니다\n"); return 2; }
            o->exportFormat = v == "stl" ? 1 : (v == "glb" ? 2 : 0);
            continue;
        }
        if (a == "--ui-scale" && i + 1 < argc) { o->uiScale = atof(argv[++i]); continue; }
        if (a == "--hide-menu") { o->hideMenu = true; continue; }
        if (a == "--floor" && hasNext) { o->floorModel = argv[++i]; continue; }
        if (a == "--scale" && hasNext) {   // 모델 전체 크기 배율. 예: --scale arm=0.5
            const simul::String v = argv[++i];
            const size_t eq = v.find('=');
            Options::Scale sc;
            sc.factor = eq == simul::String::npos ? 0.0 : atof(v.c_str() + eq + 1);
            if (eq == simul::String::npos || sc.factor <= 0.0) { fprintf(stderr, "--scale 은 모델=배율 형식입니다 (배율은 0 보다 커야 합니다. 예: --scale arm=0.5)\n"); return false; }
            sc.model = v.substr(0, eq);
            o->scales.push_back(sc);
            continue;
        }
        if ((a == "--add-box" || a == "--add-cylinder" || a == "--add-sphere") && hasNext) {
            const simul::String v = argv[++i];
            const size_t eq = v.find('=');
            Options::Shape sh;
            sh.kind = a == "--add-box" ? 'b' : (a == "--add-cylinder" ? 'c' : 's');
            sh.a[0] = sh.a[1] = sh.a[2] = 0;
            sh.pos[0] = sh.pos[1] = sh.pos[2] = 0;
            const int need = sh.kind == 'b' ? 3 : (sh.kind == 'c' ? 2 : 1);
            bool ok = eq != simul::String::npos && eq > 0;
            if (ok) {
                simul::String size = v.substr(eq + 1);
                const size_t at = size.find('@');
                if (at != simul::String::npos) {
                    ok = sscanf(size.c_str() + at + 1, "%lf,%lf,%lf", &sh.pos[0], &sh.pos[1], &sh.pos[2]) == 3;
                    size = size.substr(0, at);
                } else {
                    sh.pos[2] = -1e30;   // 위치를 안 주면 높이의 절반 위에 놓는다
                }
                int n = 0;
                if (ok) n = sscanf(size.c_str(), "%lf,%lf,%lf", &sh.a[0], &sh.a[1], &sh.a[2]);
                ok = ok && n == need;
            }
            if (!ok) {
                fprintf(stderr, "%s 는 이름=크기[@x,y,z] 형식입니다 (박스 가로,세로,높이 / 원통 반지름,길이 / 구 반지름. 예: --add-box crate=0.5,0.5,0.5@2,0,0.25)\n", a.c_str());
                return 2;
            }
            sh.name = v.substr(0, eq);
            o->shapes.push_back(sh);
            continue;
        }
        if (a == "--place" && hasNext) {
            const simul::String v = argv[++i];
            const size_t eq = v.find('=');
            Options::Place pl;
            pl.v[0] = pl.v[1] = pl.v[2] = pl.v[3] = 0;
            if (eq == simul::String::npos || eq == 0 || sscanf(v.c_str() + eq + 1, "%lf,%lf,%lf,%lf", &pl.v[0], &pl.v[1], &pl.v[2], &pl.v[3]) < 3) {
                fprintf(stderr, "--place 는 모델이름=x,y,z 또는 모델이름=x,y,z,회전도 형식입니다 (예: lidar_room=0,0,0,90)\n");
                return 2;
            }
            pl.model = v.substr(0, eq);
            o->places.push_back(pl);
            continue;
        }
        if (a == "--shading" && hasNext) {
            const simul::String v = argv[++i];
            if (v == "lambert") o->shading = 0;
            else if (v == "phong") o->shading = 1;
            else if (v == "pbr") o->shading = 2;
            else { fprintf(stderr, "--shading 은 lambert, phong, pbr 중 하나입니다: %s\n", v.c_str()); return false; }
            continue;
        }
        if (a == "--sun") {   // --sun [YYYY-MM-DDTHH:MM]  (현지 시각. 생략하면 오늘 날짜의 10:00, 시각만 생략하면 10:00)
            o->sunOn = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                const char* v = argv[++i];
                int y, mo, d, h = 10, mi = 0;
                const int n = sscanf(v, "%d-%d-%dT%d:%d", &y, &mo, &d, &h, &mi);
                if (n < 3) { fprintf(stderr, "--sun 은 YYYY-MM-DDTHH:MM 형식입니다 (현지 시각, 예: 2026-06-21T14:30)\n"); return false; }
                o->sunTime.year = y; o->sunTime.month = mo; o->sunTime.day = d; o->sunTime.hour = h; o->sunTime.minute = mi; o->sunTime.second = 0;
                o->sunTimeSet = true;
                simul::String why;
                if (!simul::env::IsValidSunTime(o->sunTime, &why)) { fprintf(stderr, "--sun: %s\n", why.c_str()); return false; }
            }
            continue;
        }
        if (a == "--no-sun") { o->noSun = true; continue; }
        if (a == "--no-shadow") { o->noShadow = true; continue; }
        if (a == "--view-angle" && hasNext) {   // 시작 시점: 방위각,고도각 (도). 고도각이 음수면 아래에서 올려다본다 (하늘 확인용)
            double y = 0, pt = 0;
            if (sscanf(argv[++i], "%lf,%lf", &y, &pt) != 2 || pt < -89.0 || pt > 89.0) { fprintf(stderr, "--view-angle 은 방위각,고도각(도, -89~89) 형식입니다\n"); return false; }
            o->viewYaw = y;
            o->viewPitch = pt;
            o->viewSet = true;
            continue;
        }
        if (a == "--sun-guide") { o->sunGuide = true; continue; }
        if (a == "--no-sky") { o->noSky = true; continue; }
        if (a == "--sun-now") { o->sunOn = true; o->sunTimeSet = false; o->sunNow = true; continue; }
        if (a == "--sun-site" && hasNext) {   // 위도,경도[,UTC오프셋[,월드+y방위]]
            double lat, lon, off = 0, nb = 0;
            const int n = sscanf(argv[++i], "%lf,%lf,%lf,%lf", &lat, &lon, &off, &nb);
            if (n < 2 || lat < -90 || lat > 90 || lon < -180 || lon > 180) { fprintf(stderr, "--sun-site 는 위도,경도[,UTC오프셋[,월드+y의 방위]] 형식입니다 (예: 37.5665,126.978,9)\n"); return false; }
            o->sunSite.latitudeDeg = lat;
            o->sunSite.longitudeDeg = lon;
            if (n >= 3) { o->sunTime.utcOffsetHours = off; o->sunOffsetSet = true; }
            if (n >= 4) o->sunSite.northBearingDeg = nb;
            o->sunSiteSet = true;
            o->sunOn = true;
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
            simul::String n, v;
            if (!SplitPair(argv[++i], &n, &v)) { fprintf(stderr, "--joint 은 이름=값 형식입니다\n"); return 2; }
            o->joints.push_back(simul::MakePair(n, atof(v.c_str())));
            continue;
        }
        if (a == "--package" && hasNext) {
            simul::String n, v;
            if (!SplitPair(argv[++i], &n, &v)) { fprintf(stderr, "--package 는 이름=폴더 형식입니다\n"); return 2; }
            o->packages.push_back(simul::MakePair(n, v));
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
        if (o->file.empty()) o->file = a; else o->moreFiles.push_back(a);
    }
    return 0;
}

simul::String LowerExt(const simul::String& p) {
    const size_t dot = p.find_last_of('.');
    if (dot == simul::String::npos) return simul::String();
    simul::String e = p.substr(dot + 1);
    for (size_t i = 0; i < e.size(); i++) if (e[i] >= 'A' && e[i] <= 'Z') e[i] = static_cast<char>(e[i] - 'A' + 'a');
    return e;
}

// --joint 로 받은 값(각도 관절은 degree, 이동 관절은 m)을 내부 값(rad, m)으로 바꾼다
simul::urdf::JointValues JointValuesFromOptions(const simul::urdf::Robot& robot, const Options& o) {
    simul::urdf::JointValues q;
    for (size_t i = 0; i < o.joints.size(); i++) {
        const int ji = robot.FindJoint(o.joints[i].first);
        if (ji < 0) { fprintf(stderr, "경고: 관절 '%s' 이 없습니다\n", o.joints[i].first.c_str()); continue; }
        q[o.joints[i].first] = static_cast<float>(robot.joints[static_cast<size_t>(ji)].FromDisplay(o.joints[i].second));
    }
    return q;
}

// 링크별 AABB 와 겹치는 링크 쌍(AABB 후보 -> OBB 로 확인한 충돌)을 출력한다 (--joint 로 준 자세 기준)
void PrintCollisions(const simul::urdf::Robot& robot, const Options& o, const simul::urdf::LoadOptions& load) {
    const simul::urdf::JointValues q = JointValuesFromOptions(robot, o);
    simul::CollisionModelOptions copt;
    copt.load = load;
    copt.fallbackToVisual = true;
    simul::RobotCollisionModel model;
    simul::BuildCollisionModel(robot, copt, &model);
    for (size_t i = 0; i < model.warnings.size(); i++) printf("경고: %s\n", model.warnings[i].c_str());
    size_t fromVisual = 0;
    for (size_t i = 0; i < model.shapes.size(); i++) if (model.shapes[i].fromVisual) fromVisual++;
    printf("AABB 충돌 검사: 도형 %zu개 (visual 로 대체한 것 %zu개)\n", model.shapes.size(), fromVisual);
    const simul::SelfCollisionReport rep = simul::CheckSelfCollisions(robot, model, q);
    for (size_t i = 0; i < rep.linkAabbs.size(); i++) {
        if (rep.linkAabbs[i].IsEmpty()) continue;
        const simul::Aabb& b = rep.linkAabbs[i];
        printf("  link '%s': (%.3f, %.3f, %.3f) ~ (%.3f, %.3f, %.3f)\n", robot.links[i].name.c_str(), static_cast<double>(b.lo.x),
                    static_cast<double>(b.lo.y), static_cast<double>(b.lo.z), static_cast<double>(b.hi.x), static_cast<double>(b.hi.y),
                    static_cast<double>(b.hi.z));
    }
    const struct { const char* label; const simul::Array<simul::CollisionPair>* pairs; } lists[2] = {
        {"AABB 겹침 후보", &rep.aabbCandidates}, {"OBB 충돌", &rep.collisions}};
    for (int k = 0; k < 2; k++) {
        const simul::Array<simul::CollisionPair>& pairs = *lists[k].pairs;
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
void PrintTorques(const simul::urdf::Robot& robot, const Options& o) {
    const simul::urdf::JointValues q = JointValuesFromOptions(robot, o);
    const simul::dynamics::Gravity g(o.gravity[0], o.gravity[1], o.gravity[2]);
    const simul::dynamics::JointTorques motor = simul::dynamics::GravityTorques(robot, q, g);
    const simul::Array<simul::dynamics::JointLoad> loads = simul::dynamics::GravityLoads(robot, q, g);
    printf("중력 하중: g = (%.4f, %.4f, %.4f) m/s^2 (루트 링크 좌표계)\n", g.x, g.y, g.z);
    double com[3], mass = 0;
    if (simul::dynamics::CenterOfMass(robot, q, com, &mass))
        printf("  전체 질량 %.4f kg, 질량중심 (%.4f, %.4f, %.4f) m\n", mass, com[0], com[1], com[2]);
    else
        printf("  (<inertial> 질량이 있는 링크가 없습니다)\n");
    for (size_t i = 0; i < robot.joints.size(); i++) {
        const simul::urdf::Joint& j = robot.joints[i];
        const simul::dynamics::JointLoad& l = loads[i];
        if (!j.IsMovable()) {
            const double f = simul::Sqrt(l.force[0] * l.force[0] + l.force[1] * l.force[1] + l.force[2] * l.force[2]);
            const double m = simul::Sqrt(l.moment[0] * l.moment[0] + l.moment[1] * l.moment[1] + l.moment[2] * l.moment[2]);
            printf("  %-16s (fixed)  체결 하중: 힘 %.4f N, 모멘트 %.4f N*m\n", j.name.c_str(), f, m);
            continue;
        }
        const simul::urdf::JointValues::const_iterator qi = q.find(j.name);
        const double shown = j.ToDisplay(qi == q.end() ? 0.0 : static_cast<double>(qi->second));
        if (j.hasMimic) {
            printf("  %-16s (mimic %s)  자기 몫 %.4f %s  (원본 관절에 합산됨)\n", j.name.c_str(), j.mimicJoint.c_str(), l.torque, j.TorqueUnit());
        } else {
            const double tau = motor.find(j.name)->second;
            printf("  %-16s %9.3f %-3s  토크 %10.4f %s", j.name.c_str(), shown, j.DisplayUnit(), tau, j.TorqueUnit());
            if (j.effort > 0.0f) printf("  (한계 %g%s)", static_cast<double>(j.effort), simul::Abs(tau) > static_cast<double>(j.effort) ? ", 초과!" : "");
            printf("\n");
        }
        const simul::Array<simul::dynamics::LinkContribution> parts = simul::dynamics::GravityContributions(robot, q, j.name, g);
        for (size_t k = 0; k < parts.size(); k++) printf("      <- %-14s %10.4f %s\n", parts[k].link.c_str(), parts[k].value, j.TorqueUnit());
    }
}

// 창 없이 파일 정보를 출력한다. 성공하면 0.
int MakeMarkerFiles(const Options& o) {
    simul::marker::MarkerParts parts;
    simul::String name, err, urdf;
    if (!simul::marker::BuildMarkerFromSpec(o.makeMarker, simul::marker::MarkerBuildOptions(), &parts, &name, &err) ||
        !simul::marker::WriteMarkerFiles(parts, o.makeMarkerDir.empty() ? simul::String(".") : o.makeMarkerDir, name, &urdf, &err)) {
        fprintf(stderr, "마커를 만들 수 없습니다: %s\n", err.c_str());
        return 2;
    }
    printf("마커 %d개 (판 %.3f x %.3f m) -> %s\n", static_cast<int>(parts.ids.size()), parts.width, parts.height, urdf.c_str());
    return 0;
}

int PrintInfo(const Options& o) {
    if (o.file.empty()) { PrintUsage("simul_main"); return 2; }
    simul::String err;
    if (LowerExt(o.file) == "urdf") {
        simul::urdf::Robot robot;
        if (!simul::urdf::LoadUrdf(o.file, &robot, &err)) { fprintf(stderr, "URDF 읽기 실패: %s\n", err.c_str()); return 1; }
        printf("로봇 '%s': link %zu개, joint %zu개 (구동 %zu개), 루트 link '%s'\n", robot.name.c_str(), robot.links.size(),
                    robot.joints.size(), robot.ActuatedJoints().size(), robot.links[static_cast<size_t>(robot.root)].name.c_str());
        simul::urdf::LoadOptions opt;
        for (size_t i = 0; i < o.packages.size(); i++) opt.packagePaths.push_back(o.packages[i]);
        simul::urdf::RobotMeshes meshes;
        const bool ok = simul::urdf::LoadRobotMeshes(robot, opt, &meshes);
        size_t tris = 0;
        for (size_t i = 0; i < meshes.visuals.size(); i++) tris += meshes.visuals[i].scene.TriangleCount();
        printf("visual %zu개 읽음, 삼각형 %zu개\n", meshes.visuals.size(), tris);
        for (size_t i = 0; i < robot.warnings.size(); i++) printf("경고: %s\n", robot.warnings[i].c_str());
        for (size_t i = 0; i < meshes.warnings.size(); i++) printf("경고: %s\n", meshes.warnings[i].c_str());
        if (o.aabb || o.obb) PrintCollisions(robot, o, opt);
        if (o.torque) PrintTorques(robot, o);
        return ok ? 0 : 1;
    }
    simul::Scene scene;
    if (!simul::LoadMesh(o.file, &scene, &err)) { fprintf(stderr, "읽기 실패: %s\n", err.c_str()); return 1; }
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

#ifdef SIMUL_HAS_QT

#include <QAction>
#include <QApplication>
#include <QMouseEvent>
#include <QActionGroup>
#include <QDockWidget>
#include <QShortcut>
#include <QMenu>
#include <QDragEnterEvent>
#include <QFileDialog>
#include <QInputDialog>
#include "CoreQt/SimulView.h"
#include "CoreQt/SimulRunner.h"
#include "CURDF/UrdfWriter.h"
#include "CoreQt/ObjectInfoPanel.h"
#include "CoreQt/PythonPanel.h"
#include "CoreQt/LogView.h"
#include "CoreQt/PropertiesPanel.h"
#include <QSet>
#include <QLineEdit>
#include <QLabel>
#include <QMainWindow>
#include <QMenuBar>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QSpinBox>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QMimeData>
#include <QSurfaceFormat>
#include <QStatusBar>
#include <QTimer>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QDir>
#include <QStandardPaths>
#include <QFileInfo>

#include "3d/Units.h"
#include "CDynamics/Dynamics.h"
#include "3d/dhPose.h"
#include "CoreQt/FolderView.h"
#include "CoreQt/JointDialog.h"
#include "CoreQt/JointPanel.h"
#include "CoreQt/CameraView.h"
#include "CoreQt/LidarPlot.h"
#include "CoreQt/SunPanel.h"
#include "CoreQt/ModelViewer.h"
#include "CoreQt/StructurePanel.h"
#include "CoreQt/UrdfView.h"

namespace {

class MainWindow : public QMainWindow {
public:
    explicit MainWindow(const Options& o) : QMainWindow(nullptr), cli_(o), gravity_(o.gravity[0], o.gravity[1], o.gravity[2]) {
        setWindowTitle("Project Simul");
        resize(o.width, o.height);
        setAcceptDrops(true);

        view_ = new SimulView(this, SimulView::kSelectionPanel | SimulView::kMouseEdit | SimulView::kDropFiles | SimulView::kTree);   // 3D 뷰 부품 (ModelViewer + 선택 패널). 메뉴, 도크, 로그는 이 프로그램(호스트)이 따로 만든다
        viewer_ = view_->viewer();
        for (size_t i = 0; i < o.packages.size(); i++)
            viewer_->LoadOptions().packagePaths.push_back(simul::MakePair(o.packages[i].first, o.packages[i].second));
        setCentralWidget(view_);
        QObject::connect(view_, &SimulView::filesDropped, this, [this](const QStringList& paths) { for (int i = 0; i < paths.size(); i++) openFile(paths[i]); });
        setDockNestingEnabled(true);   // 도크를 서로 옆/위/아래, 탭으로 자유롭게 놓을 수 있다

        tree_ = view_->tree();   // 구조 트리는 3D 화면 안에 뜨는 창이다 (SimulView 가 선택/충돌 표시를 맞춘다)
        QObject::connect(view_, &SimulView::linkContextMenuRequested, this, [this](int link, const QPoint& p) { onLinkContextMenu(link, p); });
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
        QObject::connect(viewer_, &ModelViewer::jointValuesDragged, this, [this](const simul::urdf::JointValues& q) {
            panel_->ShowValues(q);
            if (motorsOff_) {
                const simul::Array<int> act = viewer_->Robot().ActuatedJoints();
                for (size_t i = 0; i < act.size(); i++) {
                    const simul::String& n = viewer_->Robot().joints[static_cast<size_t>(act[i])].name;
                    sim_.SetValue(n, q.at(n));
                }
            }
            updateTorques(q);
            updateSelectionLabel();
        });
        // 모델을 옮기거나 돌리면 기구학(attach 관절)이 바뀐다: 시뮬레이션과 토크도 새 배치로 계산해야 중력이 항상 월드 -Z 로 작용한다
        QObject::connect(viewer_, &ModelViewer::modelChanged, this, [this]() {
            if (!viewer_->HasRobot()) return;
            if (motorsOff_ && sim_.IsReady()) sim_.UpdateRobot(viewer_->Robot());
            updateTorques();
        });
        loadGizmo();

        panel_ = new JointPanel;
        dock_ = new QDockWidget(QString::fromUtf8("관절"), this);
        dock_->setWidget(panel_);
        addDockWidget(Qt::RightDockWidgetArea, dock_);
        dock_->hide();
        log_ = new LogView;   // 열기/제거/이동/충돌/경고/오류를 시간순으로 모아 둔다
        logDock_ = new QDockWidget(QString::fromUtf8("로그"), this);
        logDock_->setWidget(log_);
        addDockWidget(Qt::BottomDockWidgetArea, logDock_);   // 화면 구성: 위 메뉴 / 가운데 3D 화면 + 오른쪽 URDF 파일 / 아래 로그
        // 속성: 엔진(3D 화면) 밖의 독립 위젯. 엔진의 공개 함수와 신호만으로 제어한다 (3D 화면 안에는 들어가지 않는다)
        propDock_ = new QDockWidget(QString::fromUtf8("속성"), this);
        propDock_->setWidget(new PropertiesPanel(viewer_));
        addDockWidget(Qt::RightDockWidgetArea, propDock_);
        // 폴더(디스크 탐색기): 폴더로 저장한 URDF 구성(이름/, 이름.urdf, obj/)을 포함해 디스크를 탐색하고, 파일을 더블클릭하면 장면에 추가한다
        folderView_ = new FolderView;
        folderDock_ = new QDockWidget(QString::fromUtf8("폴더"), this);
        folderDock_->setWidget(folderView_);
        addDockWidget(Qt::RightDockWidgetArea, folderDock_);
        QObject::connect(folderView_, &FolderView::openRequested, this, [this](const QString& p) { openFile(p); });
        // URDF 내용 보기: 지금 장면이 URDF 로 저장되면 어떻게 되는지를 읽기 전용으로 실시간 보여 준다 (엔진 밖의 독립 위젯)
        urdfDock_ = new QDockWidget(QString::fromUtf8("URDF 내용"), this);
        urdfDock_->setWidget(new UrdfView(viewer_));
        addDockWidget(Qt::RightDockWidgetArea, urdfDock_);
        tabifyDockWidget(folderDock_, urdfDock_);   // 같은 자리에 탭으로 겹친다: 폴더, URDF 내용, 속성
        tabifyDockWidget(urdfDock_, propDock_);
        folderDock_->raise();
        resizeDocks(QList<QDockWidget*>() << folderDock_, QList<int>() << qMax(300, o.width * 2 / 5), Qt::Horizontal);
        resizeDocks(QList<QDockWidget*>() << logDock_, QList<int>() << qMax(120, o.height / 5), Qt::Vertical);
        QObject::connect(viewer_, &ModelViewer::modelDragFinished, this, [this](int model, bool rotated) {
            float v[6];
            viewer_->ModelPlacement6(model, v);
            log_->Info(QString::fromUtf8("%1 %2: 위치 %3, %4, %5 m, RPY %6°, %7°, %8°")
                           .arg(viewer_->ModelName(model)).arg(rotated ? QString::fromUtf8("회전") : QString::fromUtf8("이동"))
                           .arg(static_cast<double>(v[0]), 0, 'f', 3).arg(static_cast<double>(v[1]), 0, 'f', 3).arg(static_cast<double>(v[2]), 0, 'f', 3)
                           .arg(static_cast<double>(v[3]), 0, 'f', 1).arg(static_cast<double>(v[4]), 0, 'f', 1).arg(static_cast<double>(v[5]), 0, 'f', 1));
        });
        QObject::connect(viewer_, &ModelViewer::selectionPicked, this, [this](int link, bool) {
            if (link >= 0 && static_cast<size_t>(link) < viewer_->Robot().links.size()) log_->Info(QString::fromUtf8("선택: %1").arg(simulqt::ToQ(viewer_->Robot().links[static_cast<size_t>(link)].name)));
        });
        QObject::connect(view_, &SimulView::jointEdited, this, [this](const QString& name, double value) {
            panel_->SetValue(name, value);   // 3D 화면 안의 슬라이더로 바꾼 값을 관절 도크와 토크, 시뮬레이션에도 알린다
        });
        QObject::connect(panel_, &JointPanel::valueEdited, this, [this](const QString& name, double value) {
            if (motorsOff_) sim_.SetValue(simulqt::FromQ(name), value);  // 손으로 옮긴 관절은 속도를 0 으로 다시 시작
        });
        QObject::connect(panel_, &JointPanel::valuesChanged, this, [this]() {
            viewer_->SetJointValues(panel_->Values());
            updateTorques();
            updateSelectionLabel();
        });

        QMenu* file = menuBar()->addMenu(QString::fromUtf8("파일"));
        fileMenu_ = file;
        QAction* open = file->addAction(QString::fromUtf8("열기 (추가)..."));
        open->setShortcut(QKeySequence::Open);
        QObject::connect(open, &QAction::triggered, this, [this]() {
            const QString p = QFileDialog::getOpenFileName(
                this, QString::fromUtf8("모델 또는 URDF 열기 (현재 장면에 추가됩니다)"), QString(),
                QString::fromUtf8("3D 모델 / URDF (*.urdf *.glb *.gltf *.stl *.obj *.dae);;모든 파일 (*)"));
            if (!p.isEmpty()) openFile(p);
        });
        QAction* closeAll = file->addAction(QString::fromUtf8("모두 닫기"));
        QObject::connect(closeAll, &QAction::triggered, this, [this]() {
            showLidar(false);
            viewer_->CloseAll();
            loggedWarnings_.clear();
            log_->Info(QString::fromUtf8("모두 닫기"));
            lidarSpec_ = LidarSpec();
            afterLoad(QString());
            setWindowTitle("Project Simul");
        });
        QAction* exportAct = file->addAction(QString::fromUtf8("장면을 URDF 로 저장..."));
        QObject::connect(exportAct, &QAction::triggered, this, [this]() {
            const QString p = QFileDialog::getSaveFileName(this, QString::fromUtf8("지금 장면(모델과 배치)을 URDF 하나로 저장"), QString(), QString::fromUtf8("URDF (*.urdf)"));
            if (!p.isEmpty()) exportUrdf(p);
        });
        QAction* exportDirAct = file->addAction(QString::fromUtf8("장면을 폴더로 저장..."));
        QObject::connect(exportDirAct, &QAction::triggered, this, [this]() {
            const QString parent = QFileDialog::getExistingDirectory(this, QString::fromUtf8("저장할 위치 (이 안에 새 폴더를 만듭니다)"));
            if (parent.isEmpty()) return;
            bool ok = false;
            const QString name = QInputDialog::getText(this, QString::fromUtf8("폴더로 저장"), QString::fromUtf8("폴더 이름 (폴더이름.urdf 와 obj/ 가 들어갑니다)"), QLineEdit::Normal, QString::fromUtf8("scene"), &ok).trimmed();
            if (!ok || name.isEmpty()) return;
            const QString dir = QDir(parent).filePath(name);
            if (QFileInfo(dir).exists() && QMessageBox::question(this, QString::fromUtf8("폴더로 저장"), QString::fromUtf8("%1 이(가) 이미 있습니다. 같은 이름의 파일을 덮어씁니다. 계속할까요?").arg(dir)) != QMessageBox::Yes) return;
            const QStringList formats = QStringList() << QString::fromUtf8("OBJ + MTL (색과 텍스처 유지)") << QString::fromUtf8("STL (호환 최고, 색은 URDF 로 나눠 저장, 용량 큼)") << QString::fromUtf8("GLB 그대로 복사 (용량 가장 작음, 색/텍스처 유지)");
            const QString f = QInputDialog::getItem(this, QString::fromUtf8("폴더로 저장"), QString::fromUtf8("GLB 모델의 메시 형식"), formats, 0, false, &ok);
            if (!ok) return;
            exportFolder(dir, formats.indexOf(f));
        });
        QAction* openDirAct = file->addAction(QString::fromUtf8("폴더 열기 (추가)..."));
        QObject::connect(openDirAct, &QAction::triggered, this, [this]() {
            const QString d = QFileDialog::getExistingDirectory(this, QString::fromUtf8("폴더로 저장한 모델 열기 (현재 장면에 추가됩니다)"));
            if (!d.isEmpty() && openFile(d)) folderView_->Reveal(d);
        });
        file->addSeparator();
        QAction* quit = file->addAction(QString::fromUtf8("종료"));
        quit->setShortcut(QKeySequence::Quit);
        QObject::connect(quit, &QAction::triggered, this, &QWidget::close);
        QMenu* view = menuBar()->addMenu(QString::fromUtf8("보기"));      // 화면에 보이는 것
        QMenu* addMenu = view_->NewAddMenu(this);   // 박스, 원통, 구 추가
        menuBar()->addMenu(addMenu);
        QObject::connect(view_, &SimulView::shapeAddFailed, this, [this](const QString& e) { logError(QString::fromUtf8("도형 추가 실패: %1").arg(e)); });
        QMenu* phys = menuBar()->addMenu(QString::fromUtf8("물리"));      // 중력, 모터, 낙하
        QMenu* sensorMenu = menuBar()->addMenu(QString::fromUtf8("센서"));  // 라이다 표시
        QMenu* help = menuBar()->addMenu(QString::fromUtf8("도움말"));
        nativeMenus_ << file << view << addMenu << phys << sensorMenu << help;   // 3D 화면만 보기에서는 이 메뉴들을 3D 화면 안의 메뉴줄에 그대로 보여 준다
        QAction* fit = view->addAction(QString::fromUtf8("화면에 맞춤"));
        fit->setShortcut(Qt::Key_F);
        QObject::connect(fit, &QAction::triggered, viewer_, &ModelViewer::FitView);
        QAction* grid = view->addAction(QString::fromUtf8("격자"));
        grid->setCheckable(true);
        grid->setChecked(true);
        QObject::connect(grid, &QAction::toggled, viewer_, &ModelViewer::SetShowGrid);
        QAction* viewSep = view->addSeparator();  // 위: 화면 요소, 아래: 조명/충돌 박스
        QMenu* shadeMenu = view->addMenu(QString::fromUtf8("조명 방식"));
        QActionGroup* shadeGroup = new QActionGroup(this);
        const char* shadeNames[3] = {"Lambert (무광)", "Blinn-Phong (하이라이트)", "PBR (물리 기반)"};
        for (int m = 0; m < 3; m++) {
            QAction* sa = shadeMenu->addAction(QString::fromUtf8(shadeNames[m]));
            sa->setCheckable(true);
            sa->setChecked(m == cli_.shading);
            shadeGroup->addAction(sa);
            QObject::connect(sa, &QAction::triggered, this, [this, m]() { viewer_->SetShading(m); });
        }
        viewer_->SetShading(cli_.shading);
        QMenu* ringMenu = view->addMenu(QString::fromUtf8("회전 링 기준"));
        QActionGroup* ringGroup = new QActionGroup(this);
        const char* ringNames[2] = {"월드 축 (기본)", "오브젝트 축"};
        for (int k = 0; k < 2; k++) {
            QAction* ra = ringMenu->addAction(QString::fromUtf8(ringNames[k]));
            ra->setCheckable(true);
            ra->setChecked(k == 0);
            ringGroup->addAction(ra);
            QObject::connect(ra, &QAction::triggered, this, [this, k]() { viewer_->SetRingLocal(k == 1); });
        }
        QAction* grav = phys->addAction(QString::fromUtf8("중력 방향..."));
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
                gravity_ = simul::dynamics::Gravity(gx, gy, gz);
                log_->Info(QString::fromUtf8("중력 가속도: %1, %2, %3 m/s²").arg(gx).arg(gy).arg(gz));
                viewer_->SetGravity(gx, gy, gz);
                sim_.SetGravity(gravity_);
                updateTorques();
            } else {
                QMessageBox::warning(this, QString::fromUtf8("중력 가속도"), QString::fromUtf8("숫자 세 개(gx, gy, gz)를 넣어 주세요."));
            }
        });
        QAction* structure = new QAction(QString::fromUtf8("구조 트리"), this);   // 3D 화면 안의 구조 트리 창 켜고 끄기
        structure->setCheckable(true);
        structure->setChecked(true);
        structure->setShortcut(Qt::Key_T);
        QObject::connect(structure, &QAction::toggled, this, [this](bool on) { view_->SetTreeVisible(on); });
        QObject::connect(view_, &SimulView::featuresChanged, this, [structure](unsigned f) {   // 창의 X 로 닫아도 체크가 따라온다
            const bool b = structure->blockSignals(true);
            structure->setChecked((f & SimulView::kTree) != 0);
            structure->blockSignals(b);
        });
        view->insertAction(viewSep, structure);
        {   // 3D 화면 안 UI(메뉴줄, 구조 트리, 선택 패널) 크기
            QMenu* uiSize = view->addMenu(QString::fromUtf8("화면 UI 크기"));
            view->removeAction(uiSize->menuAction());
            view->insertMenu(viewSep, uiSize);
            QAction* big = uiSize->addAction(QString::fromUtf8("크게"), this, [this]() { view_->SetUiScale(view_->UiScale() * 1.15); }, QKeySequence(Qt::CTRL + Qt::Key_Equal));
            QAction* small = uiSize->addAction(QString::fromUtf8("작게"), this, [this]() { view_->SetUiScale(view_->UiScale() / 1.15); }, QKeySequence(Qt::CTRL + Qt::Key_Minus));
            QAction* norm = uiSize->addAction(QString::fromUtf8("기본 크기"), this, [this]() { view_->SetUiScale(1.0); }, QKeySequence(Qt::CTRL + Qt::Key_0));
            addAction(big); addAction(small); addAction(norm);   // 메뉴를 숨겨도 단축키는 계속 먹는다
        }
        QAction* jointDock = dock_->toggleViewAction();   // 오른쪽 관절 도크는 기본으로 숨긴다 (슬라이더는 클릭한 물체 옆 오버레이에 나온다)
        jointDock->setText(QString::fromUtf8("관절 도크 (전체 관절)"));
        view->insertAction(viewSep, jointDock);
        QAction* propAct = propDock_->toggleViewAction();
        propAct->setText(QString::fromUtf8("속성"));
        propAct->setShortcut(Qt::Key_P);
        view->insertAction(viewSep, propAct);
        QAction* folderAct = folderDock_->toggleViewAction();
        folderAct->setText(QString::fromUtf8("폴더 (탐색기)"));
        folderAct->setShortcut(Qt::Key_E);
        view->insertAction(viewSep, folderAct);
        QAction* urdfAct = urdfDock_->toggleViewAction();
        urdfAct->setText(QString::fromUtf8("URDF 내용 보기"));
        urdfAct->setShortcut(Qt::Key_U);
        view->insertAction(viewSep, urdfAct);
        QAction* logAct = logDock_->toggleViewAction();
        logAct->setText(QString::fromUtf8("로그"));
        logAct->setShortcut(Qt::Key_L);
        view->insertAction(viewSep, logAct);
        QObject::connect(viewer_, &ModelViewer::shaderFailed, this, [this](const QString& log) { log_->Error(log); });
        sunPanel_ = new SunPanel(viewer_);
        sunDock_ = new QDockWidget(QString::fromUtf8("태양"), this);
        sunDock_->setWidget(sunPanel_);
        addDockWidget(Qt::RightDockWidgetArea, sunDock_);
        configureDock(sunDock_);
        sunDock_->hide();
        QAction* sunAct = sunDock_->toggleViewAction();
        sunAct->setText(QString::fromUtf8("태양 (시각으로 조명)"));
        sunAct->setShortcut(Qt::Key_S);
        view->insertAction(viewSep, sunAct);
        // 화면 구성: 메뉴 숨기기(F10), 3D 화면만 보기(F11). 숨긴 메뉴의 단축키는 동작하지 않으므로 창 자체에도 달아 둔다.
        menuAct_ = new QAction(QString::fromUtf8("메뉴 표시"), this);
        menuAct_->setCheckable(true);
        menuAct_->setChecked(true);
        menuAct_->setShortcut(Qt::Key_F10);
        QObject::connect(menuAct_, &QAction::toggled, this, [this](bool on) { menuBar()->setVisible(on); if (!uiHidden_) menuWasVisible_ = on; });
        view->insertAction(viewSep, menuAct_);
        addAction(menuAct_);
        onlyViewAct_ = new QAction(QString::fromUtf8("3D 화면만 보기"), this);
        onlyViewAct_->setCheckable(true);
        onlyViewAct_->setShortcut(Qt::Key_F11);
        QObject::connect(onlyViewAct_, &QAction::toggled, this, [this](bool on) { setUiHidden(on); });
        view->insertAction(viewSep, onlyViewAct_);
        addAction(onlyViewAct_);
        QShortcut* esc = new QShortcut(QKeySequence(Qt::Key_Escape), this);   // 3D 화면만 보는 중에는 Esc 로도 돌아온다
        QObject::connect(esc, &QShortcut::activated, this, [this]() { if (uiHidden_) onlyViewAct_->setChecked(false); });
        view->insertSeparator(viewSep);
        configureDocks();
        QAction* frame = new QAction(QString::fromUtf8("선택 좌표계"), this);
        view->insertAction(viewSep, frame);
        frame->setShortcut(Qt::Key_G);
        frame->setCheckable(true);
        frame->setChecked(true);
        QObject::connect(frame, &QAction::toggled, viewer_, &ModelViewer::SetShowSelectionFrame);
        QAction* fall = phys->addAction(QString::fromUtf8("중력 낙하 (물체를 놓으면 떨어짐)"));
        fall->setCheckable(true);
        fall->setChecked(!o.noFall);
        viewer_->SetFallEnabled(!o.noFall);
        viewer_->SetGravity(gravity_.x, gravity_.y, gravity_.z);
        QObject::connect(fall, &QAction::toggled, viewer_, &ModelViewer::SetFallEnabled);
        QAction* motors = phys->addAction(QString::fromUtf8("모터 끄기 (관절이 중력에 처짐)"));
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
        phys->addSeparator();
        QAction* resetPos = phys->addAction(QString::fromUtf8("오브젝트 위치 초기화"));
        QObject::connect(resetPos, &QAction::triggered, viewer_, [this]() { viewer_->ResetObjectPosition(); });
        view->addSeparator();
        QMenu* colMenu = view->addMenu(QString::fromUtf8("충돌 박스"));
        QAction* obbs = colMenu->addAction(QString::fromUtf8("OBB (회전하는 박스, 정밀)"));
        obbs->setShortcut(Qt::Key_O);
        obbs->setCheckable(true);
        obbs->setChecked(o.obb);
        viewer_->SetShowObbs(o.obb);
        QObject::connect(obbs, &QAction::toggled, viewer_, &ModelViewer::SetShowObbs);
        QAction* boxes = colMenu->addAction(QString::fromUtf8("AABB (축에 맞춘 박스, 빠른 1차 검사)"));
        boxes->setShortcut(Qt::Key_B);
        boxes->setCheckable(true);
        boxes->setChecked(o.aabb);
        viewer_->SetShowCollisionBoxes(o.aabb);
        QObject::connect(boxes, &QAction::toggled, viewer_, &ModelViewer::SetShowCollisionBoxes);
        // 센서 메뉴: 현재 모델에 라이다가 있을 때만 켜진다. 열 때마다 상태를 다시 읽는다
        QMenu* lidarMenu = sensorMenu->addMenu(QString::fromUtf8("라이다"));
        QAction* lidarBeams = lidarMenu->addAction(QString::fromUtf8("레이저 표시 (3D 화면)"));
        lidarBeams->setCheckable(true);
        QAction* lidarGraph = lidarMenu->addAction(QString::fromUtf8("그래프 표시"));
        lidarGraph->setCheckable(true);
        QObject::connect(lidarMenu, &QMenu::aboutToShow, this, [this, lidarBeams, lidarGraph]() {
            const bool has = lidarSpec_.link >= 0;
            lidarBeams->setEnabled(has);
            lidarGraph->setEnabled(has);
            lidarBeams->setChecked(has && viewer_->LidarEnabled() && viewer_->LidarDrawn());
            lidarGraph->setChecked(has && lidarDock_ && lidarDock_->isVisible());
        });
        QObject::connect(lidarBeams, &QAction::triggered, this, [this](bool on) { showLidarBeams(on); });
        QObject::connect(lidarGraph, &QAction::triggered, this, [this](bool on) { showLidarGraph(on); });
        QObject::connect(sensorMenu, &QMenu::aboutToShow, this, [this, lidarMenu]() {
            lidarMenu->setEnabled(lidarSpec_.link >= 0);
            lidarMenu->setTitle(QString::fromUtf8(lidarSpec_.link >= 0 ? "라이다" : "라이다 (이 모델에는 없음)"));
        });
        QAction* cameraAct = sensorMenu->addAction(QString::fromUtf8("카메라 영상"));
        cameraAct->setCheckable(true);
        QObject::connect(sensorMenu, &QMenu::aboutToShow, this, [this, cameraAct]() {
            cameraAct->setEnabled(cameraSpec_.link >= 0);
            cameraAct->setChecked(cameraDock_ && cameraDock_->isVisible());
        });
        QObject::connect(cameraAct, &QAction::triggered, this, [this](bool on) { showCamera(on); });
        QAction* usage = help->addAction(QString::fromUtf8("조작 방법"));
        usage->setShortcut(Qt::Key_F1);
        QObject::connect(usage, &QAction::triggered, this, [this]() {
            QMessageBox::information(this, QString::fromUtf8("조작 방법"), QString::fromUtf8(
                "마우스\n"
                "  왼쪽 드래그: 회전\n"
                "  오른쪽 드래그: 이동\n"
                "  휠: 확대/축소\n"
                "  왼쪽 클릭: 물체(링크) 선택, 로봇팔 끝을 드래그하면 IK 로 따라감\n"
                "  구조 트리에서 라이다 링크 오른쪽 클릭: 레이저/그래프 켜고 끄기\n\n"
                "단축키\n"
                "  Ctrl+O 열기   F 화면에 맞춤   T 구조 트리   G 선택 좌표계\n"
                "  M 모터 끄기   O OBB 박스   B AABB 박스   F1 이 도움말"));
        });
        collisionLabel_ = new QLabel;
        statusBar()->addPermanentWidget(collisionLabel_);
        QObject::connect(viewer_, &ModelViewer::structureChanged, this, [this]() {   // 단독 모델을 처음 돌리면 world 가 생겨 링크 구성이 바뀐다
            tree_->SetRobot(viewer_->Robot());
            tree_->SetCollidingLinks(viewer_->CollidingLinks());
            tree_->SelectLink(viewer_->SelectedLink());
            panel_->SetRobot(viewer_->Robot());
            const simul::urdf::JointValues& q = viewer_->CurrentJointValues();
            panel_->blockSignals(true);
            for (simul::urdf::JointValues::const_iterator it = q.begin(); it != q.end(); ++it) panel_->SetValue(simulqt::ToQ(it->first), it->second);
            panel_->blockSignals(false);
            updateSelectionLabel();
        });
        QObject::connect(viewer_, &ModelViewer::collisionsChanged, this, [this]() { updateCollisionLabel(); });
        QObject::connect(viewer_, &ModelViewer::collisionsChanged, this, [this]() { tree_->SetCollidingLinks(viewer_->CollidingLinks()); });
        infoMessage_ = QString::fromUtf8("파일 > 열기 (Ctrl+O) 또는 파일을 창에 끌어다 놓으세요. 왼쪽 드래그: 회전, 오른쪽 드래그: 이동, 휠: 확대, 왼쪽 클릭: 물체 선택");
        statusBar()->showMessage(infoMessage_);
    }

    void startHidden(const Options& o) {
        if (o.hideMenu) menuAct_->setChecked(false);
        if (o.showFps) viewer_->SetFrameStatsVisible(true);
        if (o.uiScale != 1.0) view_->SetUiScale(o.uiScale);
        if (o.onlyView) onlyViewAct_->setChecked(true);
        applySun(o);
        if (o.viewSet) viewer_->SetCamera(viewer_->CameraTarget(), viewer_->CameraDistance(), dhRad(o.viewYaw), dhRad(o.viewPitch));
    }

    // 태양: 명령행에서 켠 경우. 날짜를 주지 않으면 오늘 날짜의 10:00 (--sun-now 면 지금 시각, 그 컴퓨터의 시간대), 위치만 주고 UTC 오프셋을 안 주면 경도/15 를 반올림한다.
    void applySun(const Options& o) {
        viewer_->SetShadowsEnabled(!o.noShadow);
        viewer_->SetSkyEnabled(!o.noSky);
        viewer_->SetSunGuideVisible(o.sunGuide);
        if (o.noSun) viewer_->SetSunEnabled(false);
        if (!o.sunOn) return;
        ModelViewer::SunSettings s;
        s.enabled = true;
        s.site = o.sunSite;
        if (o.sunTimeSet) {
            s.time = o.sunTime;
            if (!o.sunOffsetSet) s.time.utcOffsetHours = o.sunSiteSet ? simul::Floor(o.sunSite.longitudeDeg / 15.0 + 0.5) : 9.0;   // 기본 위치(서울)는 +9
        } else {
            // 기본값은 SunSettings 의 오늘 날짜 10:00. --sun-now 면 지금 시각
            if (o.sunNow) s.time = simul::env::SunTimeNow();
            else if (!o.sunOffsetSet && o.sunSiteSet) s.time.utcOffsetHours = simul::Floor(o.sunSite.longitudeDeg / 15.0 + 0.5);
            if (o.sunOffsetSet) s.time.utcOffsetHours = o.sunTime.utcOffsetHours;
        }
        QString err;
        if (!viewer_->SetSun(s, &err)) {
            fprintf(stderr, "태양 설정 오류: %s\n", err.toUtf8().constData());
            return;
        }
        log_->Info(QString::fromUtf8("태양: %1-%2-%3 %4:%5 (UTC%6%7), 위도 %8 경도 %9 -> 방위각 %10°, 고도 %11°")
                       .arg(s.time.year).arg(s.time.month, 2, 10, QChar('0')).arg(s.time.day, 2, 10, QChar('0')).arg(s.time.hour, 2, 10, QChar('0')).arg(s.time.minute, 2, 10, QChar('0'))
                       .arg(s.time.utcOffsetHours >= 0 ? "+" : "").arg(s.time.utcOffsetHours).arg(s.site.latitudeDeg).arg(s.site.longitudeDeg)
                       .arg(viewer_->SunStateNow().azimuthDeg, 0, 'f', 1).arg(viewer_->SunStateNow().elevationDeg, 0, 'f', 1));
    }

    // 지금 장면을 URDF 하나로 저장한다. 성공하면 true (실패는 로그와 상태줄에 남긴다)
    bool exportUrdf(const QString& path) {
        QString err;
        if (!viewer_->SaveUrdf(path, &err)) {
            logError(QString::fromUtf8("URDF 저장 실패: %1").arg(err));
            return false;
        }
        const QString msg = QString::fromUtf8("URDF 저장: %1 (모델 %2개, 링크 %3, 관절 %4)").arg(path).arg(viewer_->ModelCount()).arg(viewer_->Robot().links.size()).arg(viewer_->Robot().joints.size());
        log_->Info(msg);
        statusBar()->showMessage(msg);
        return true;
    }

    // 지금 장면을 폴더로 저장한다: dir/<폴더이름>.urdf + dir/obj/. 성공하면 true
    bool exportFolder(const QString& dir, int meshFormat = 0) {
        QString err;
        QStringList warns;
        if (!viewer_->SaveUrdfFolder(dir, &err, &warns, meshFormat)) {
            logError(QString::fromUtf8("폴더 저장 실패: %1").arg(err));
            return false;
        }
        for (int i = 0; i < warns.size(); i++) log_->Warning(QString::fromUtf8("폴더 저장: %1").arg(warns[i]));
        const QString msg = QString::fromUtf8("폴더로 저장: %1 (모델 %2개, 링크 %3, 관절 %4, 경고 %5)").arg(dir).arg(viewer_->ModelCount()).arg(viewer_->Robot().links.size()).arg(viewer_->Robot().joints.size()).arg(warns.size());
        log_->Info(msg);
        statusBar()->showMessage(msg);
        folderView_->Reveal(dir);   // 탐색기에서 방금 만든 폴더가 보이게 한다
        return true;
    }

    // 파일을 열어 장면에 추가한다 (이전 모델은 지우지 않는다). 지우려면 구조 트리에서 우클릭 -> 제거, 또는 파일 -> 모두 닫기.
#ifdef SIMUL_HAS_PYTHON
    // ---- 내장 Python: 엔진 + Main.py + 등록된 외부 클래스 ----
    ~MainWindow() override {
        if (runner_) runner_->Stop();
        delete py_;   // 인터프리터를 끝낸다 (엔진보다 먼저)
        py_ = nullptr;
    }

    static void pyOutput(bool isError, const char* line, void* user) {
        MainWindow* w = static_cast<MainWindow*>(user);
        const QString text = QString::fromUtf8(line);
        if (isError) { fprintf(stderr, "%s\n", line); w->log_->Error(text); }
        else { printf("%s\n", line); fflush(stdout); w->log_->Info(text); }
    }

    // 처음 필요할 때 엔진, 인터프리터, Main.py 를 만든다. 실패하면 로그에 남기고 false
    bool ensurePython(const Options& o) {
        if (py_) return pyReady_;
        if (!backend_) backend_ = new ViewerBackend(viewer_);
        engine_ = new simul::sim::Engine(backend_);
        runner_ = new SimulRunner(engine_, this);
        runner_->SetRate(o.pyHz);
        py_ = new simul::py::Runtime(engine_);
        py_->SetOutput(&MainWindow::pyOutput, this);
        QObject::connect(runner_, &SimulRunner::started, this, [this]() { log_->Info(QString::fromUtf8("시뮬레이션 시작 (%1 Hz)").arg(runner_->Rate())); });
        QObject::connect(runner_, &SimulRunner::stopped, this, [this]() { log_->Info(QString::fromUtf8("시뮬레이션 정지 (시계 %1 s)").arg(engine_->Time(), 0, 'f', 2)); });
        QObject::connect(runner_, &SimulRunner::stepFailed, this, [this](const QString& e) { log_->Error(e); });
        simul::String err;
        if (!py_->Initialize(&err)) { logError(QString::fromUtf8("Python 을 시작하지 못했습니다: %1").arg(simulqt::ToQ(err))); return false; }
        log_->Info(QString::fromUtf8("Python %1 시작").arg(simulqt::ToQ(simul::py::Runtime::PythonVersion())));
        simul::String mainPy = o.pyMain.empty() ? simul::String(SIMUL_DATA_DIR) + "/Python/Main.py" : o.pyMain;
        if (!py_->RunFile(mainPy, &err)) { logError(QString::fromUtf8("Main.py 를 올리지 못했습니다: %1 (%2)").arg(simulqt::ToQ(mainPy)).arg(simulqt::ToQ(err))); return false; }
        log_->Info(QString::fromUtf8("Main.py: %1").arg(simulqt::ToQ(mainPy)));
        pyReady_ = true;
        return true;
    }

    bool registerScript(const simul::String& file, const simul::String& objects, bool reload = false) {
        if (!pyReady_) return false;
        // 경로와 목록은 Python 문자열로 안전하게 넘긴다 (작은따옴표, 역슬래시 처리)
        simul::String f, ob;
        for (size_t i = 0; i < file.size(); i++) { if (file[i] == '\\' || file[i] == '\'') f += '\\'; f += file[i]; }
        for (size_t i = 0; i < objects.size(); i++) { if (objects[i] == '\\' || objects[i] == '\'') ob += '\\'; ob += objects[i]; }
        simul::String code = simul::String("import Main; _r = Main.") + (reload ? "reload_file" : "register_file") + "('" + f + "'" + (objects.empty() ? simul::String() : simul::String(", '") + ob + "'") + ")";
        simul::String err;
        py_->RunSnippet(code, &err);
        if (!err.empty()) { logError(QString::fromUtf8("등록 실패: %1").arg(simulqt::ToQ(err))); return false; }
        log_->Info(QString::fromUtf8("스크립트 등록: %1%2").arg(simulqt::ToQ(file)).arg(objects.empty() ? QString() : QString::fromUtf8("  (권한: %1)").arg(simulqt::ToQ(objects))));
        rememberScript(simulqt::ToQ(file), simulqt::ToQ(objects));
        return true;
    }

    // 메뉴 Python 을 만들고, 옵션에 따라 바로 시작하거나 N 틱을 돌린다. 모델을 모두 연 뒤에 부른다.
    void setupPython(const Options& o) {
        pyOpt_ = o;
        pyOpt_.pyEnabled = true;
        pyOpt_.pyScripts.clear();
        pyOpt_.projectFile.clear();
        pyOpt_.pyRun = false;
        pyOpt_.pyEditor = false;
        pyOpt_.objPanel = false;
        QMenu* m = menuBar()->addMenu(QString::fromUtf8("Python"));
        nativeMenus_ << m;
        QAction* start = m->addAction(QString::fromUtf8("시작"));
        start->setShortcut(Qt::Key_F5);
        QAction* stop = m->addAction(QString::fromUtf8("정지"));
        stop->setShortcut(Qt::SHIFT + Qt::Key_F5);
        QAction* once = m->addAction(QString::fromUtf8("한 틱"));
        once->setShortcut(Qt::Key_F10);
        m->addSeparator();
        QAction* reg = m->addAction(QString::fromUtf8("스크립트 등록..."));
        QAction* info = m->addAction(QString::fromUtf8("오브젝트와 등록 현황 보기"));
        // 오른쪽 Python 편집기: 파일을 쓰고, 저장하면서 등록(다시 읽기)하고, 한 줄씩 실행한다
        pyPanel_ = new PythonPanel;
        // 새 파일 기본 저장 위치. 개발용 Data 폴더가 없는 PC(패키지로 받은 실행 파일)에서는 문서 폴더를 쓴다 (없는 경로를 만들어 내지 않는다)
        QString userDir = QString::fromUtf8(SIMUL_DATA_DIR) + QStringLiteral("/Python");
        if (QFileInfo(userDir).isDir()) userDir += QStringLiteral("/user");
        else userDir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + QStringLiteral("/Simul/python");
        QDir().mkpath(userDir);
        pyPanel_->SetDefaultDir(userDir);
        pyDock_ = new QDockWidget(QString::fromUtf8("Python 편집기"), this);
        pyDock_->setWidget(pyPanel_);
        addDockWidget(Qt::RightDockWidgetArea, pyDock_);
        resizeDocks(QList<QDockWidget*>() << pyDock_, QList<int>() << 480, Qt::Horizontal);
        pyDock_->hide();
        // 오브젝트 / 관절 정보: 엔진 없이도 장면에서 바로 읽는다. 더블클릭하면 편집기 커서에 "이름" 이 들어간다
        if (!backend_) backend_ = new ViewerBackend(viewer_);
        objPanel_ = new ObjectInfoPanel(backend_);
        objDock_ = new QDockWidget(QString::fromUtf8("오브젝트 / 관절 값"), this);
        objDock_->setWidget(objPanel_);
        addDockWidget(Qt::RightDockWidgetArea, objDock_);
        splitDockWidget(pyDock_, objDock_, Qt::Vertical);
        objDock_->hide();
        m->insertAction(reg, objDock_->toggleViewAction());
        objDock_->toggleViewAction()->setText(QString::fromUtf8("오브젝트 / 관절 값"));
        objDock_->toggleViewAction()->setShortcut(Qt::CTRL + Qt::Key_J);
        QObject::connect(objPanel_, &ObjectInfoPanel::nameActivated, this, [this](const QString& quoted) {
            if (!pyDock_->isVisible()) pyDock_->show();
            pyPanel_->editor()->insertPlainText(quoted);
            pyPanel_->editor()->setFocus();
        });
        m->insertAction(reg, pyDock_->toggleViewAction());
        pyDock_->toggleViewAction()->setText(QString::fromUtf8("Python 편집기"));
        pyDock_->toggleViewAction()->setShortcut(Qt::CTRL + Qt::Key_E);
        const Options opt = o;
        QObject::connect(pyPanel_, &PythonPanel::registerRequested, this, [this, opt](const QString& path, const QString& objs) {
            if (!ensurePython(opt)) return;
            logDock_->show();
            registerScript(simulqt::FromQ(path), simulqt::FromQ(objs), true);
        });
        QObject::connect(pyPanel_, &PythonPanel::snippetRequested, this, [this, opt](const QString& code) {
            if (!ensurePython(opt)) return;
            logDock_->show();
            log_->Info(QStringLiteral(">>> ") + code);
            simul::String err;
            py_->RunSnippet(simulqt::FromQ(code), &err);
        });
        QObject::connect(start, &QAction::triggered, this, [this, opt]() { if (ensurePython(opt)) { logDock_->show(); runner_->Start(); } });
        QObject::connect(stop, &QAction::triggered, this, [this]() { if (runner_) runner_->Stop(); });
        QObject::connect(once, &QAction::triggered, this, [this, opt]() { if (ensurePython(opt)) { logDock_->show(); runner_->StepOnce(); } });
        QObject::connect(reg, &QAction::triggered, this, [this, opt]() {
            const QString f = QFileDialog::getOpenFileName(this, QString::fromUtf8("등록할 python 파일"), QString(), QString::fromUtf8("Python (*.py)"));
            if (f.isEmpty() || !ensurePython(opt)) return;
            bool ok = false;
            const QString objs = QInputDialog::getText(this, QString::fromUtf8("움직일 수 있는 오브젝트"), QString::fromUtf8("오브젝트 이름 (쉼표로 구분, 비우면 파일 안의 objects 설정을 따름, * 는 전부)"), QLineEdit::Normal, QString(), &ok);
            if (!ok) return;
            logDock_->show();
            registerScript(simulqt::FromQ(f), simulqt::FromQ(objs.trimmed()));
        });
        QObject::connect(info, &QAction::triggered, this, [this, opt]() {
            if (!ensurePython(opt)) return;
            logDock_->show();
            simul::String err;
            py_->RunSnippet("Main.hub.refresh(); print(Main.hub.describe())", &err);
        });
        if (fileMenu_) {   // 프로젝트: 장면 + python 을 폴더 하나로 저장 / 열기 / 실행 파일 하나로 내보내기
            fileMenu_->addSeparator();
            QAction* pOpen = fileMenu_->addAction(QString::fromUtf8("프로젝트 열기..."));
            QAction* pSave = fileMenu_->addAction(QString::fromUtf8("프로젝트로 저장..."));
            QAction* pExport = fileMenu_->addAction(QString::fromUtf8("실행 파일로 내보내기..."));
            QObject::connect(pOpen, &QAction::triggered, this, [this]() {
                const QString f = QFileDialog::getOpenFileName(this, QString::fromUtf8("프로젝트 열기"), QString(), QString::fromUtf8("프로젝트 (project.simul *.simul)"));
                if (!f.isEmpty()) runProject(f);
            });
            QObject::connect(pSave, &QAction::triggered, this, [this]() { saveProjectAs(); });
            QObject::connect(pExport, &QAction::triggered, this, [this]() { exportExecutable(); });
        }
        if (o.objPanel) objDock_->show();
        if (o.pyEditor) {
            pyDock_->show();
            if (!o.pyScripts.empty()) pyPanel_->Open(simulqt::ToQ(o.pyScripts[0].first));
        }
        if (!o.pyEnabled) return;
        if (!ensurePython(o)) return;
        for (size_t i = 0; i < o.pyScripts.size(); i++) registerScript(o.pyScripts[i].first, o.pyScripts[i].second);
        if (o.pySteps > 0) {
            runner_->Start();
            for (int i = 0; i < o.pySteps; i++) runner_->StepOnce();
            runner_->Stop();
            printf("시뮬레이션 %d 틱 (시계 %.3f s)\n", o.pySteps, engine_->Time());
        } else if (o.pyRun) {
            logDock_->show();
            runner_->Start();
        }
    }
    // ---- 프로젝트 저장 / 열기 / 내보내기 ----
    void rememberScript(const QString& file, const QString& objects) {
        const QString abs = QFileInfo(file).absoluteFilePath();
        for (int i = 0; i < regScripts_.size(); i++)
            if (QFileInfo(regScripts_[i].first).absoluteFilePath() == abs) { regScripts_[i].second = objects; return; }
        regScripts_.push_back(qMakePair(file, objects));
    }

    static QString SafeName(const QString& n) {
        QString out;
        for (int i = 0; i < n.size(); i++) {
            const QChar c = n[i];
            out += (c.isLetterOrNumber() || c == QLatin1Char('.') || c == QLatin1Char('_') || c == QLatin1Char('-')) ? c : QLatin1Char('_');
        }
        return out.isEmpty() ? QStringLiteral("model") : out;
    }

    // 지금 장면(모델마다 폴더, 배치)과 등록한 python 파일(권한)을 dirPath 에 project.simul 과 함께 쓴다
    bool writeProject(const QString& dirPath) {
        QDir base(dirPath);
        if (!base.mkpath(QStringLiteral("models")) || !base.mkpath(QStringLiteral("scripts"))) {
            logError(QString::fromUtf8("폴더를 만들 수 없습니다: %1").arg(dirPath));
            return false;
        }
        simul::project::Project p;
        p.name = simulqt::FromQ(base.dirName());
        QStringList used;
        for (int i = 0; i < viewer_->ModelCount(); i++) {
            QString safe = SafeName(viewer_->ModelName(i));
            if (safe != viewer_->ModelName(i)) log_->Warning(QString::fromUtf8("모델 이름 '%1' 을 파일 이름 '%2' 로 바꿔 저장합니다 (python 에서 쓰는 이름이 달라질 수 있음)").arg(viewer_->ModelName(i)).arg(safe));
            QString uniq = safe;
            for (int k = 2; used.contains(uniq); k++) uniq = safe + QStringLiteral("_") + QString::number(k);
            used << uniq;
            QString err;
            QStringList warns;
            if (!viewer_->SaveModelFolder(i, base.filePath(QStringLiteral("models/") + uniq), &err, &warns)) {
                logError(QString::fromUtf8("모델 '%1' 저장 실패: %2").arg(viewer_->ModelName(i)).arg(err));
                return false;
            }
            for (int w = 0; w < warns.size(); w++) log_->Warning(QString::fromUtf8("모델 저장: %1").arg(warns[w]));
            simul::project::ModelEntry m;
            m.file = simulqt::FromQ(QStringLiteral("models/") + uniq + QLatin1Char('/') + uniq + QStringLiteral(".urdf"));
            if (viewer_->ModelConnected(i)) {   // 다른 모델에 붙어 있는 것은 연결 대신 월드 자세로 저장한다
                const dhPose wp = dhPose::FromMat(viewer_->ModelWorldPose(i));
                double rpy[3];
                simul::urdf::RpyFromMat(viewer_->ModelWorldPose(i), rpy);
                m.position[0] = wp.position.x; m.position[1] = wp.position.y; m.position[2] = wp.position.z;
                for (int k = 0; k < 3; k++) m.rpy[k] = rpy[k];
                log_->Warning(QString::fromUtf8("모델 '%1' 은 다른 모델에 연결되어 있습니다: 연결 관계는 저장되지 않고 월드 자세로 저장합니다").arg(viewer_->ModelName(i)));
            } else {
                float v[6];
                viewer_->ModelPlacement6(i, v);
                for (int k = 0; k < 3; k++) { m.position[k] = v[k]; m.rpy[k] = v[3 + k] * 3.14159265358979323846 / 180.0; }
            }
            p.models.push_back(m);
        }
        QStringList scriptNames;
        for (int i = 0; i < regScripts_.size(); i++) {
            const QString src = QFileInfo(regScripts_[i].first).absoluteFilePath();
            if (pyPanel_ && pyPanel_->Modified() && QFileInfo(pyPanel_->FilePath()).absoluteFilePath() == src) pyPanel_->Save();   // 편집기에서 고친 것도 담는다
            QString bn = QFileInfo(src).fileName();
            QString uniq = bn;
            for (int k = 2; scriptNames.contains(uniq); k++) uniq = QFileInfo(bn).completeBaseName() + QStringLiteral("_") + QString::number(k) + QStringLiteral(".py");
            scriptNames << uniq;
            const QString dst = base.filePath(QStringLiteral("scripts/") + uniq);
            if (QFileInfo(dst).absoluteFilePath() != src) {
                QFile::remove(dst);
                if (!QFile::copy(src, dst)) {
                    logError(QString::fromUtf8("python 파일을 복사할 수 없습니다: %1 -> %2").arg(src).arg(dst));
                    return false;
                }
            }
            simul::project::ScriptEntry se;
            se.file = simulqt::FromQ(QStringLiteral("scripts/") + uniq);
            se.objects = simulqt::FromQ(regScripts_[i].second);
            p.scripts.push_back(se);
        }
        p.hz = runner_ ? runner_->Rate() : pyOpt_.pyHz;
        p.autostart = !p.scripts.empty();
        p.showEditor = pyDock_ && pyDock_->isVisible();
        simul::String err;
        const QString file = base.filePath(QStringLiteral("project.simul"));
        if (!simul::project::SaveProject(simulqt::FromQ(file), p, &err)) {
            logError(QString::fromUtf8("project.simul 저장 실패: %1").arg(simulqt::ToQ(err)));
            return false;
        }
        projectFile_ = file;
        setWindowTitle(QString::fromUtf8("%1 - Project Simul").arg(base.dirName()));
        const QString msg = QString::fromUtf8("프로젝트 저장: %1 (모델 %2개, python 파일 %3개)").arg(file).arg(p.models.size()).arg(p.scripts.size());
        log_->Info(msg);
        statusBar()->showMessage(msg);
        return true;
    }

    bool saveProjectAs() {
        if (viewer_->ModelCount() == 0) {
            QMessageBox::information(this, QString::fromUtf8("프로젝트로 저장"), QString::fromUtf8("저장할 모델이 없습니다. 먼저 모델을 여세요."));
            return false;
        }
        const QString start = projectFile_.isEmpty() ? QString() : QFileInfo(projectFile_).absolutePath();
        const QString dir = QFileDialog::getExistingDirectory(this, QString::fromUtf8("프로젝트를 저장할 폴더 (이 폴더에 project.simul, models, scripts 가 만들어집니다)"), start);
        if (dir.isEmpty()) return false;
        if (QFileInfo(QDir(dir).filePath(QStringLiteral("project.simul"))).exists() &&
            QMessageBox::question(this, QString::fromUtf8("프로젝트로 저장"), QString::fromUtf8("이 폴더에 이미 프로젝트가 있습니다. 덮어쓸까요?\n%1").arg(dir)) != QMessageBox::Yes)
            return false;
        logDock_->show();
        return writeProject(dir);
    }

    // project.simul 을 열어 지금 장면을 바꾸고 python 을 등록하고 (설정대로) 시작한다
    bool runProject(const QString& path) {
        if (runner_) runner_->Stop();
        if (py_ && pyReady_) {
            for (int i = 0; i < regScripts_.size(); i++) {
                simul::String f;
                const simul::String raw = simulqt::FromQ(regScripts_[i].first);
                for (size_t k = 0; k < raw.size(); k++) { if (raw[k] == '\\' || raw[k] == '\'') f += '\\'; f += raw[k]; }
                simul::String err;
                py_->RunSnippet(simul::String("import Main; Main.hub.unregister_file('") + f + "')", &err);
            }
        }
        regScripts_.clear();
        showLidar(false);
        viewer_->CloseAll();
        loggedWarnings_.clear();
        lidarSpec_ = LidarSpec();
        afterLoad(QString());
        Options o = pyOpt_;
        o.projectFile = simulqt::FromQ(path);
        o.pyScripts.clear();
        if (!loadProject(&o)) return false;
        projectFile_ = path;
        logDock_->show();
        if (!ensurePython(o)) return true;
        if (runner_) runner_->SetRate(o.pyHz);
        for (size_t i = 0; i < o.pyScripts.size(); i++) registerScript(o.pyScripts[i].first, o.pyScripts[i].second);
        if (o.objPanel) objDock_->show();
        if (o.pyEditor) {
            pyDock_->show();
            if (!o.pyScripts.empty()) pyPanel_->Open(simulqt::ToQ(o.pyScripts[0].first));
        }
        if (o.pyRun && runner_) runner_->Start();
        return true;
    }

    // 프로젝트를 이 프로그램의 실행 파일 뒤에 붙여 실행 파일 하나로 만든다
    bool exportExecutable() {
        if (projectFile_.isEmpty()) {
            if (QMessageBox::question(this, QString::fromUtf8("실행 파일로 내보내기"), QString::fromUtf8("먼저 프로젝트로 저장해야 합니다. 지금 저장할까요?")) != QMessageBox::Yes) return false;
            if (!saveProjectAs()) return false;
        } else {
            const QMessageBox::StandardButton b = QMessageBox::question(
                this, QString::fromUtf8("실행 파일로 내보내기"),
                QString::fromUtf8("지금 장면과 python 파일을 프로젝트 폴더에 다시 저장한 뒤 내보낼까요?\n(아니오: 저장된 프로젝트 그대로 내보냅니다)\n%1").arg(projectFile_),
                QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
            if (b == QMessageBox::Cancel) return false;
            if (b == QMessageBox::Yes && !writeProject(QFileInfo(projectFile_).absolutePath())) return false;
        }
        const QFileInfo pf(projectFile_);
        QString suggest = pf.absoluteDir().absoluteFilePath(pf.absoluteDir().dirName()
#ifdef _WIN32
                                                            + QStringLiteral(".exe")
#endif
                                                            );
        const QString out = QFileDialog::getSaveFileName(this, QString::fromUtf8("실행 파일로 내보내기"), suggest
#ifdef _WIN32
                                                         , QString::fromUtf8("실행 파일 (*.exe)")
#endif
                                                         );
        if (out.isEmpty()) return false;
        QString dataDir = QCoreApplication::applicationDirPath() + QStringLiteral("/Data");
#ifdef SIMUL_DATA_DIR
        if (!QFileInfo(dataDir + QStringLiteral("/Python/Main.py")).isFile()) dataDir = QString::fromUtf8(SIMUL_DATA_DIR);
#endif
        QString err;
        int n = 0;
        logDock_->show();
        if (!projectpack::PackProject(projectFile_, dataDir, QCoreApplication::applicationFilePath(), out, &err, &n)) {
            logError(QString::fromUtf8("내보내기 실패: %1").arg(err));
            QMessageBox::warning(this, QString::fromUtf8("내보내기 실패"), err);
            return false;
        }
        const QString msg = QString::fromUtf8("실행 파일로 내보냈습니다 (파일 %1개): %2").arg(n).arg(out);
        log_->Info(msg);
        statusBar()->showMessage(msg);
        return true;
    }
    bool pythonQuitRequested() const { return py_ && py_->QuitRequested(); }
#else
    void setupPython(const Options&) {}
    bool pythonQuitRequested() const { return false; }
#endif

    // project.simul 을 열어 모델을 올리고 배치하고, python 등록/설정을 Options 에 채운다 (setupPython 이 이어서 처리한다)
    bool loadProject(Options* o) {
        simul::project::Project p;
        simul::String err;
        if (!simul::project::LoadProject(o->projectFile, &p, &err)) {
            logError(simulqt::ToQ(err));
            QMessageBox::warning(this, QString::fromUtf8("프로젝트를 열 수 없음"), simulqt::ToQ(err));
            return false;
        }
        const QDir base = QFileInfo(simulqt::ToQ(o->projectFile)).absoluteDir();
        setWindowTitle((p.name.empty() ? QString::fromUtf8("프로젝트") : simulqt::ToQ(p.name)) + QStringLiteral(" - Project Simul"));
        log_->Info(QString::fromUtf8("프로젝트: %1 (%2)").arg(simulqt::ToQ(p.name)).arg(simulqt::ToQ(o->projectFile)));
        for (size_t i = 0; i < p.models.size(); i++) {
            const simul::project::ModelEntry& m = p.models[i];
            if (!openFile(base.filePath(simulqt::ToQ(m.file)))) continue;
            if (m.Placed()) {
                const float v[6] = {static_cast<float>(m.position[0]), static_cast<float>(m.position[1]), static_cast<float>(m.position[2]),
                                    static_cast<float>(dhDeg(m.rpy[0])), static_cast<float>(dhDeg(m.rpy[1])), static_cast<float>(dhDeg(m.rpy[2]))};
                viewer_->SetModelPlacement6(viewer_->ModelCount() - 1, v);
            }
        }
        for (size_t i = 0; i < p.scripts.size(); i++)
            o->pyScripts.push_back(simul::MakePair(simulqt::FromQ(base.filePath(simulqt::ToQ(p.scripts[i].file))), p.scripts[i].objects));
        o->pyEnabled = true;
        o->pyHz = p.hz;
        o->pyRun = p.autostart;
        o->pyEditor = p.showEditor;
        o->objPanel = o->objPanel || p.showEditor;
        if (!p.mainPy.empty()) o->pyMain = simulqt::FromQ(base.filePath(simulqt::ToQ(p.mainPy)));
        else if (!o->embeddedDir.empty()) o->pyMain = o->embeddedDir + "/__simul/Main.py";
        viewer_->FitView();
        return true;
    }

    bool openFile(const QString& pathIn) {
        QString path = pathIn;
        if (QFileInfo(path).isDir()) {   // 폴더로 저장한 것: 폴더이름.urdf, 없으면 안에 URDF 가 하나뿐일 때 그것
            const QDir d(path);
            QString found;
            if (QFileInfo(d.filePath(d.dirName() + ".urdf")).isFile()) found = d.filePath(d.dirName() + ".urdf");
            else {
                const QStringList list = d.entryList(QStringList() << "*.urdf", QDir::Files);
                if (list.size() == 1) found = d.filePath(list[0]);
            }
            if (found.isEmpty()) {
                const QString e = QString::fromUtf8("폴더에서 URDF 를 찾을 수 없습니다 (%1.urdf 또는 *.urdf 하나)").arg(d.dirName());
                logError(QString::fromUtf8("열 수 없음: %1 - %2").arg(path).arg(e));
                QMessageBox::warning(this, QString::fromUtf8("열 수 없음"), e);
                return false;
            }
            path = found;
        }
        const bool beamsWere = viewer_->LidarEnabled() && viewer_->LidarDrawn();
        const bool graphWere = lidarDock_ && lidarDock_->isVisible();
        QString err;
        if (!viewer_->AddFile(path, &err)) {
            logError(QString::fromUtf8("열 수 없음: %1 - %2").arg(path).arg(err));
            QMessageBox::warning(this, QString::fromUtf8("열 수 없음"), err);
            return false;
        }
        afterLoad(path);
        log_->Info(QString::fromUtf8("열기: %1  (모델 %2개, %3)").arg(path).arg(viewer_->ModelCount()).arg(viewer_->Summary()));
        for (int i = 0; i < viewer_->Warnings().size(); i++) {   // 합친 뒤의 경고 중 아직 기록하지 않은 것
            if (loggedWarnings_.contains(viewer_->Warnings()[i])) continue;
            loggedWarnings_.insert(viewer_->Warnings()[i]);
            log_->Warning(viewer_->Warnings()[i]);
        }
        applyLidar(cli_);  // URDF 의 라이다 설정을 읽어 둔다
        applyCamera(cli_);
        if (beamsWere && lidarSpec_.link >= 0) showLidarBeams(true);  // 모델을 더해도 켜 둔 라이다 표시는 유지
        if (graphWere && lidarSpec_.link >= 0) showLidarGraph(true);
        return true;
    }

    // --place: 모델 이름으로 월드 위치를 정한다
    void applyConnects(const Options& o) {
        for (size_t i = 0; i < o.connects.size(); i++) {
            const QStringList top = simulqt::ToQ(o.connects[i]).split('=');
            const QStringList rest = top.size() == 2 ? top[1].split(':') : QStringList();
            int child = -1;
            if (top.size() == 2) for (int m = 0; m < viewer_->ModelCount(); m++) if (viewer_->ModelName(m) == top[0]) child = m;
            const int link = rest.isEmpty() ? -1 : viewer_->Robot().FindLink(simulqt::FromQ(rest[0]));
            if (child < 0 || link < 0) {
                const QString m = QString::fromUtf8("--connect: 자식 모델 또는 부모 링크를 찾지 못했습니다 (%1)").arg(simulqt::ToQ(o.connects[i]));
                fprintf(stderr, "경고: %s\n", m.toUtf8().constData());
                log_->Warning(m);
                continue;
            }
            simul::urdf::AttachSpec spec;
            const QString kind = rest.size() > 1 ? rest[1] : QString("fixed");
            spec.type = kind == "revolute" ? simul::urdf::kJointRevolute : kind == "continuous" ? simul::urdf::kJointContinuous : kind == "prismatic" ? simul::urdf::kJointPrismatic : simul::urdf::kJointFixed;
            if (rest.size() > 2) {
                const QStringList ax = rest[2].split(',');
                if (ax.size() == 3) spec.axis = dhVector(ax[0].toFloat(), ax[1].toFloat(), ax[2].toFloat());
            }
            if (spec.type == simul::urdf::kJointRevolute || spec.type == simul::urdf::kJointPrismatic) {
                spec.hasLimit = true;
                spec.lower = spec.type == simul::urdf::kJointRevolute ? static_cast<float>(dhRad(-90.0)) : 0.0f;
                spec.upper = spec.type == simul::urdf::kJointRevolute ? static_cast<float>(dhRad(90.0)) : 0.5f;
                if (rest.size() > 3) {
                    const QStringList lim = rest[3].split(',');
                    if (lim.size() == 2) {
                        const double k = spec.type == simul::urdf::kJointRevolute ? dhRad(1.0) : 1.0;
                        spec.lower = static_cast<float>(lim[0].toDouble() * k);
                        spec.upper = static_cast<float>(lim[1].toDouble() * k);
                    }
                }
            }
            QString err;
            if (viewer_->ConnectModel(child, link, spec, nullptr, &err)) log_->Info(QString::fromUtf8("관절로 연결: %1 -> %2 (%3)").arg(top[0]).arg(rest[0]).arg(kind));
            else logError(QString::fromUtf8("--connect 실패: %1").arg(err));
        }
        if (!o.connects.empty()) afterLoad(QString::fromUtf8("관절 연결"));
    }

    void applyPlaces(const Options& o) {
        for (size_t i = 0; i < o.places.size(); i++) {
            int found = -1;
            for (int m = 0; m < viewer_->ModelCount(); m++) if (viewer_->ModelName(m) == simulqt::ToQ(o.places[i].model)) found = m;
            if (found < 0) { fprintf(stderr, "경고: 모델 '%s' 이 없습니다 (--place)\n", o.places[i].model.c_str()); log_->Warning(QString::fromUtf8("--place: 모델 '%1' 이 없습니다").arg(simulqt::ToQ(o.places[i].model))); continue; }
            viewer_->SetModelPlacement(found, static_cast<float>(o.places[i].v[0]), static_cast<float>(o.places[i].v[1]), static_cast<float>(o.places[i].v[2]), static_cast<float>(o.places[i].v[3]));
        }
        if (!o.places.empty()) afterLoad(QString::fromUtf8("배치 적용"));
    }

    // 명령행의 --scale 모델=배율
    void applyFloor(const Options& o) {
        if (o.floorModel.empty()) return;
        const QString name = simulqt::ToQ(o.floorModel);
        int found = -1;
        for (int m = 0; m < viewer_->ModelCount(); m++) if (viewer_->ModelName(m) == name) found = m;
        if (found < 0) {
            fprintf(stderr, "경고: --floor %s 모델을 찾지 못했습니다\n", o.floorModel.c_str());
            logError(QString::fromUtf8("바닥 지정 실패: %1").arg(name));
            return;
        }
        viewer_->SetFloorModel(found);
    }

    void applyScales(const Options& o) {
        for (size_t i = 0; i < o.scales.size(); i++) {
            const QString name = simulqt::ToQ(o.scales[i].model);
            int found = -1;
            for (int m = 0; m < viewer_->ModelCount(); m++) if (viewer_->ModelName(m) == name) found = m;
            if (found < 0 || !viewer_->SetModelScale(found, o.scales[i].factor)) {
                fprintf(stderr, "경고: --scale %s 를 적용하지 못했습니다 (모델 이름을 확인하세요)\n", o.scales[i].model.c_str());
                logError(QString::fromUtf8("크기 배율 적용 실패: %1").arg(name));
            }
        }
        if (!o.scales.empty()) { viewer_->FitView(); afterLoad(QString::fromUtf8("크기 배율")); }   // 시작할 때의 배율이면 화면도 새 크기에 맞춘다
    }

    // 명령행의 --add-box/--add-cylinder/--add-sphere 를 장면에 올린다
    void applyShapes(const Options& o) {
        for (size_t i = 0; i < o.shapes.size(); i++) {
            const Options::Shape& s = o.shapes[i];
            const double height = s.kind == 'b' ? s.a[2] : (s.kind == 'c' ? s.a[1] : 2.0 * s.a[0]);
            const float pose[6] = {static_cast<float>(s.pos[0]), static_cast<float>(s.pos[1]), s.pos[2] < -1e29 ? static_cast<float>(height / 2.0) : static_cast<float>(s.pos[2]), 0, 0, 0};
            const QString name = simulqt::ToQ(s.name);
            QString err;
            int idx = -1;
            if (s.kind == 'b') idx = viewer_->AddBox(name, s.a[0], s.a[1], s.a[2], pose, QColor(120, 160, 220), 5.0, &err);
            else if (s.kind == 'c') idx = viewer_->AddCylinder(name, s.a[0], s.a[1], pose, QColor(220, 150, 110), 5.0, &err);
            else idx = viewer_->AddSphere(name, s.a[0], pose, QColor(130, 200, 150), 5.0, &err);
            if (idx < 0) { fprintf(stderr, "경고: 도형 '%s' 추가 실패: %s\n", s.name.c_str(), err.toUtf8().constData()); logError(QString::fromUtf8("도형 추가 실패: %1").arg(err)); }
        }
        if (!o.shapes.empty()) afterLoad(QString::fromUtf8("도형 추가"));
    }

    // 명령행에서 받은 관절 값을 적용한다 (각도 관절은 degree 로 받아 rad 로 바꾼다)
    void applyJoints(const Options& o) {
        for (size_t i = 0; i < o.joints.size(); i++) {
            const int ji = viewer_->Robot().FindJoint(o.joints[i].first);
            if (ji < 0) { fprintf(stderr, "경고: 관절 '%s' 이 없습니다\n", o.joints[i].first.c_str()); log_->Warning(QString::fromUtf8("관절 '%1' 이 없습니다").arg(simulqt::ToQ(o.joints[i].first))); continue; }
            panel_->SetValue(simulqt::ToQ(o.joints[i].first), viewer_->Robot().joints[static_cast<size_t>(ji)].FromDisplay(o.joints[i].second));
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

    // 센서 카메라: URDF 의 <gazebo><sensor type="camera"> 또는 --camera 로 지정한 링크에 단다 (명령행이 우선). 영상 창은 메뉴 센서 > 카메라 로 켜고 끈다.
    void applyCamera(const Options& o) {
        showCamera(false);
        viewer_->ClearSensorCamera();
        cameraSpec_ = CameraSpec();
        if (!viewer_->HasRobot()) return;
        const simul::urdf::Robot& robot = viewer_->Robot();
        CameraSpec sp;
        bool fromUrdf = false;
        if (!o.cameraLink.empty()) {  // 명령행
            sp.link = robot.FindLink(o.cameraLink);
            if (sp.link < 0) { fprintf(stderr, "경고: 링크 '%s' 이 없습니다 (카메라를 달지 않음)\n", o.cameraLink.c_str()); return; }
            sp.mount = dhMat::Trans(o.cameraMount[0], o.cameraMount[1], o.cameraMount[2]);
            sp.settings.width = o.cameraW;
            sp.settings.height = o.cameraH;
            sp.settings.horizontalFov = static_cast<float>(dhRad(o.cameraFov));
            sp.settings.nearClip = o.cameraRange[0];
            sp.settings.farClip = o.cameraRange[1];
            for (int i = 0; i < 5; i++) sp.settings.distortion[i] = o.cameraDistort[i];
            sp.settings.colorNoiseStdDev = o.cameraNoise[0];
            sp.settings.noiseStdDev = o.cameraNoise[1];
        } else if (!robot.cameras.empty()) {  // URDF 의 <gazebo><sensor type="camera">
            const simul::urdf::CameraDef& d = robot.cameras[0];
            if (robot.cameras.size() > 1) fprintf(stderr, "알림: 카메라가 %d 개 있어 첫 번째('%s')만 씁니다\n", static_cast<int>(robot.cameras.size()), d.name.c_str());
            sp.link = robot.FindLink(d.link);
            if (sp.link < 0) return;
            sp.mount = d.pose;
            sp.settings.width = d.width;
            sp.settings.height = d.height;
            sp.settings.horizontalFov = d.horizontalFov;
            sp.settings.nearClip = d.nearClip;
            sp.settings.farClip = d.farClip;
            for (int i = 0; i < 5; i++) sp.settings.distortion[i] = o.cameraDistortSet ? o.cameraDistort[i] : d.distortion[i];
            sp.settings.colorNoiseStdDev = o.cameraNoiseSet ? o.cameraNoise[0] : d.colorNoise;
            sp.settings.noiseStdDev = o.cameraNoiseSet ? o.cameraNoise[1] : d.depthNoise;
            fromUrdf = true;
        } else {
            return;
        }
        sp.settings.antialias = o.cameraAa;
        cameraSpec_ = sp;
        viewer_->SetSensorCamera(sp.link, sp.settings, sp.mount);
        viewer_->SetSensorCameraDrawn(false);
        log_->Info(QString::fromUtf8("카메라: 링크 '%1', %2x%3, 시야각 %4도%5").arg(QString::fromUtf8(robot.links[static_cast<size_t>(sp.link)].name.c_str())).arg(sp.settings.width).arg(sp.settings.height)
                       .arg(dhDeg(static_cast<double>(sp.settings.horizontalFov)), 0, 'f', 1).arg(fromUrdf ? QString::fromUtf8(" (URDF)") : QString()));
        if (!fromUrdf || !o.cameraSave.empty()) showCamera(true);  // URDF 의 카메라는 메뉴 센서 > 카메라 영상 으로 켠다 (저장을 요청했으면 바로)
    }

    // 카메라 영상 도크 (오른쪽). 보이는 동안에만 0.2 초마다 찍어서 갱신한다
    void showCamera(bool on) {
        if (on && cameraSpec_.link < 0) return;
        if (!on) {
            if (cameraDock_) cameraDock_->hide();
            if (viewer_->SensorCameraEnabled()) viewer_->SetSensorCameraDrawn(false);
            return;
        }
        if (!cameraView_) {
            cameraView_ = new CameraView;
            QWidget* box = new QWidget;
            QVBoxLayout* lay = new QVBoxLayout(box);
            lay->setContentsMargins(0, 0, 0, 0);
            QCheckBox* depth = new QCheckBox(QString::fromUtf8("깊이 보기"));
            QObject::connect(depth, &QCheckBox::toggled, cameraView_, &CameraView::SetShowDepth);
            lay->addWidget(depth);
            QHBoxLayout* aaRow = new QHBoxLayout;
            aaRow->addWidget(new QLabel(QString::fromUtf8("안티앨리어싱 (1 = 끔, n = n x n)")));
            QSpinBox* aa = new QSpinBox;
            aa->setRange(1, 4);
            aa->setValue(cameraSpec_.settings.antialias);
            aa->setToolTip(QString::fromUtf8("픽셀을 가로세로 n 배로 나눠 찍어 평균합니다. 경계가 부드러워지고 시간은 n^2 배 걸립니다"));
            QObject::connect(aa, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, [this](int v) {
                if (cameraSpec_.link < 0) return;
                cameraSpec_.settings.antialias = v;
                viewer_->SetSensorCamera(cameraSpec_.link, cameraSpec_.settings, cameraSpec_.mount);
                log_->Info(QString::fromUtf8("카메라 안티앨리어싱: %1").arg(v));
            });
            aaRow->addWidget(aa);
            lay->addLayout(aaRow);
            QHBoxLayout* distRow = new QHBoxLayout;
            distRow->addWidget(new QLabel(QString::fromUtf8("렌즈 왜곡 k1,k2,p1,p2,k3")));
            QLineEdit* dist = new QLineEdit;
            {
                const float* dv = cameraSpec_.settings.distortion;
                dist->setText(QString("%1, %2, %3, %4, %5").arg(dv[0]).arg(dv[1]).arg(dv[2]).arg(dv[3]).arg(dv[4]));
            }
            dist->setToolTip(QString::fromUtf8("OpenCV 와 같은 왜곡 계수. 예: -0.3, 0.1, 0, 0, 0 (통 모양), 0.3 (실패 모양). 숫자가 모자라면 0. 엔터로 적용"));
            QObject::connect(dist, &QLineEdit::returnPressed, this, [this, dist]() {
                if (cameraSpec_.link < 0) return;
                QString flat = dist->text();
                flat.replace(',', ' ');
                const QStringList p = flat.simplified().split(' ', QString::SkipEmptyParts);
                float f[5] = {0, 0, 0, 0, 0};
                bool good = p.size() >= 1 && p.size() <= 5;
                for (int i = 0; good && i < p.size(); i++) {
                    bool ok = false;
                    f[i] = p[i].toFloat(&ok);
                    good = ok;
                }
                if (!good) {
                    log_->Info(QString::fromUtf8("왜곡 계수는 숫자 1~5개입니다 (k1, k2, p1, p2, k3)"));
                    return;
                }
                for (int i = 0; i < 5; i++) cameraSpec_.settings.distortion[i] = f[i];
                viewer_->SetSensorCamera(cameraSpec_.link, cameraSpec_.settings, cameraSpec_.mount);
                log_->Info(QString::fromUtf8("카메라 렌즈 왜곡: k1=%1 k2=%2 p1=%3 p2=%4 k3=%5").arg(f[0]).arg(f[1]).arg(f[2]).arg(f[3]).arg(f[4]));
            });
            distRow->addWidget(dist);
            lay->addLayout(distRow);
            QHBoxLayout* noiseRow = new QHBoxLayout;
            noiseRow->addWidget(new QLabel(QString::fromUtf8("잡음 색(0~1), 깊이(m)")));
            QLineEdit* noise = new QLineEdit(QString("%1, %2").arg(cameraSpec_.settings.colorNoiseStdDev).arg(cameraSpec_.settings.noiseStdDev));
            noise->setToolTip(QString::fromUtf8("색 잡음은 만점 대비 비율(0.02 = 약 5/255), 깊이 잡음은 미터. 0, 0 이면 잡음 없음. 엔터로 적용"));
            QObject::connect(noise, &QLineEdit::returnPressed, this, [this, noise]() {
                if (cameraSpec_.link < 0) return;
                QString flat = noise->text();
                flat.replace(',', ' ');
                const QStringList p = flat.simplified().split(' ', QString::SkipEmptyParts);
                bool ok1 = false, ok2 = false;
                const float c = p.size() == 2 ? p[0].toFloat(&ok1) : 0.0f, d = p.size() == 2 ? p[1].toFloat(&ok2) : 0.0f;
                if (!(ok1 && ok2) || c < 0.0f || d < 0.0f) {
                    log_->Info(QString::fromUtf8("잡음은 '색, 깊이' 두 숫자(0 이상)입니다"));
                    return;
                }
                cameraSpec_.settings.colorNoiseStdDev = c;
                cameraSpec_.settings.noiseStdDev = d;
                viewer_->SetSensorCamera(cameraSpec_.link, cameraSpec_.settings, cameraSpec_.mount);
                log_->Info(QString::fromUtf8("카메라 잡음: 색 %1, 깊이 %2 m").arg(c).arg(d));
            });
            noiseRow->addWidget(noise);
            lay->addLayout(noiseRow);
            lay->addWidget(cameraView_, 1);
            cameraDock_ = new QDockWidget(QString::fromUtf8("카메라"), this);
            cameraDock_->setWidget(box);
            addDockWidget(Qt::RightDockWidgetArea, cameraDock_);
            configureDock(cameraDock_);
            QTimer* t = new QTimer(this);
            QObject::connect(t, &QTimer::timeout, this, [this]() {
                if (!viewer_->SensorCameraEnabled() || !cameraDock_->isVisible()) return;
                cameraView_->SetDepthRange(viewer_->SensorCameraSettings().nearClip, viewer_->SensorCameraSettings().farClip);
                cameraView_->SetImage(viewer_->RenderSensorCamera());
            });
            t->start(200);
        }
        viewer_->SetSensorCameraDrawn(true);
        cameraDock_->show();
    }

    // 라이다: URDF 의 <gazebo><sensor type="ray"> 에서 설정을 읽어 둔다 (여럿이면 첫 번째).
    // 화면에는 그 링크를 마우스 오른쪽 버튼으로 눌러 "라이다 스캔 표시" 를 켜면 나온다.
    // --lidar 링크 / --lidar-channels 는 명령행에서 덮어쓰거나 URDF 에 없을 때 쓰며, 이때는 바로 켠다.
    void applyLidar(const Options& o) {
        showLidar(false);
        lidarSpec_ = LidarSpec();
        if (!viewer_->HasRobot()) return;
        const simul::urdf::Robot& robot = viewer_->Robot();
        LidarSpec sp;
        sp.mount = dhMat::Trans(o.lidarMount[0], o.lidarMount[1], o.lidarMount[2]);
        if (!o.lidarLink.empty()) {  // 명령행
            sp.link = robot.FindLink(o.lidarLink);
            if (sp.link < 0) { fprintf(stderr, "경고: 링크 '%s' 이 없습니다 (라이다를 달지 않음)\n", o.lidarLink.c_str()); return; }
            sp.settings.channels = o.lidarChannels;
            sp.settings.elevationMin = static_cast<float>(dhRad(o.lidarElev[0]));
            sp.settings.elevationMax = static_cast<float>(dhRad(o.lidarElev[1]));
        } else if (!robot.lidars.empty()) {  // URDF
            const simul::urdf::LidarDef& d = robot.lidars[0];
            if (robot.lidars.size() > 1) fprintf(stderr, "알림: 라이다가 %d 개 있어 첫 번째('%s')만 씁니다\n", static_cast<int>(robot.lidars.size()), d.name.c_str());
            sp.link = robot.FindLink(d.link);
            if (sp.link < 0) return;
            simul::sensor::LidarSettings& ls = sp.settings;
            ls.angleMin = d.hMin;
            ls.angleMax = d.hMax;
            ls.angleIncrement = d.hSamples > 1 ? (d.hMax - d.hMin) / static_cast<float>(d.hSamples - 1) : 1.0f;
            ls.channels = d.vSamples;
            ls.elevationMin = d.vMin;
            ls.elevationMax = d.vMax;
            ls.rangeMin = d.rangeMin;
            ls.rangeMax = d.rangeMax;
            ls.noiseStdDev = d.noiseStdDev;
            sp.mount = d.pose;
            sp.fromUrdf = true;
        } else {
            return;
        }
        lidarSpec_ = sp;
        if (!sp.fromUrdf || !o.lidarCsv.empty()) showLidar(true);  // URDF 의 라이다는 오른쪽 버튼으로 켠다 (CSV 저장을 요청했으면 바로)
    }

    // 센서를 달아 둔다 (아직 안 달렸으면). 광선 표시와 그래프는 아래 두 함수로 따로 켜고 끈다
    // 새로 달 때 레이저 표시는 꺼진 채로 시작한다 (켜는 쪽이 따로 켠다)
    bool ensureLidar() {
        if (lidarSpec_.link < 0) return false;
        if (!viewer_->LidarEnabled()) {
            viewer_->SetLidar(lidarSpec_.link, lidarSpec_.settings, lidarSpec_.mount);
            viewer_->SetLidarDrawn(false);
        }
        return true;
    }

    // 3D 화면의 레이저(광선 + 맞은 점) 표시
    void showLidarBeams(bool on) {
        if (on && !ensureLidar()) return;
        if (on != (viewer_->LidarEnabled() && viewer_->LidarDrawn())) log_->Info(on ? QString::fromUtf8("라이다 레이저 표시 켬") : QString::fromUtf8("라이다 레이저 표시 끔"));   // 바뀔 때만
        if (viewer_->LidarEnabled()) viewer_->SetLidarDrawn(on);  // ensureLidar 뒤라서 항상 달려 있다 (끄기 때는 안 달렸을 수도)
    }

    // 스캔 그래프 도크 (오른쪽). 보이는 동안에만 0.1 초마다 스캔해서 갱신한다
    void showLidarGraph(bool on) {
        if (on && !ensureLidar()) return;
        if (on != (lidarDock_ && lidarDock_->isVisible())) log_->Info(on ? QString::fromUtf8("라이다 그래프 켬") : QString::fromUtf8("라이다 그래프 끔"));
        if (!on) {
            if (lidarDock_) lidarDock_->hide();
            return;
        }
        if (!lidarPlot_) {
            lidarPlot_ = new LidarPlot;
            lidarDock_ = new QDockWidget(QString::fromUtf8("라이다 그래프"), this);
            lidarDock_->setWidget(lidarPlot_);
            addDockWidget(Qt::RightDockWidgetArea, lidarDock_);
            configureDock(lidarDock_);
            // 3D 화면이 그려지는지(레이저 표시가 켜져 있는지)와 상관없이 그래프가 나오도록 직접 스캔한다
            QTimer* t = new QTimer(this);
            QObject::connect(t, &QTimer::timeout, this, [this]() {
                if (!viewer_->LidarEnabled() || !lidarDock_->isVisible()) return;
                const simul::sensor::LidarScan& sc = viewer_->ScanLidar();
                lidarPlot_->SetScan(sc, viewer_->LidarOrigin(), viewer_->LidarHeading());
            });
            t->start(100);
        }
        lidarDock_->show();
    }

    // 레이저 표시와 그래프를 한꺼번에 켜고 끈다 (명령행 --lidar, 끄기)
    void showLidar(bool on) {
        showLidarBeams(on);
        showLidarGraph(on);
        if (!on) viewer_->ClearLidar();
    }

    // 구조 트리에서 링크를 오른쪽 버튼으로 눌렀을 때: 라이다가 달린 링크면 레이저 표시와 그래프를 따로 켜고 끈다  (exec 가 끝나면 체크 상태가 이미 바뀌어 있다)
    void onLinkContextMenu(int link, const QPoint& globalPos) {
        const int model = viewer_->ModelOfRoot(link);   // 모델의 루트 링크를 눌렀는가
        const bool lidar = link >= 0 && link == lidarSpec_.link;
        if (model < 0 && !lidar) return;
        QMenu menu;
        QAction *beams = nullptr, *graph = nullptr, *place = nullptr, *remove = nullptr, *floor = nullptr, *connect = nullptr, *disconnect = nullptr;
        if (lidar) {
            beams = menu.addAction(QString::fromUtf8("레이저 표시 (3D 화면)"));
            beams->setCheckable(true);
            beams->setChecked(viewer_->LidarEnabled() && viewer_->LidarDrawn());
            graph = menu.addAction(QString::fromUtf8("그래프 표시"));
            graph->setCheckable(true);
            graph->setChecked(lidarDock_ && lidarDock_->isVisible());
        }
        if (model >= 0) {
            if (lidar) menu.addSeparator();
            place = menu.addAction(QString::fromUtf8("위치 지정... (%1)").arg(viewer_->ModelName(model)));
            if (viewer_->ModelCount() >= 2) connect = menu.addAction(QString::fromUtf8("관절로 연결... (%1)").arg(viewer_->ModelName(model)));   // 다른 모델의 링크에 관절로 붙인다
            if (viewer_->ModelConnected(model)) disconnect = menu.addAction(QString::fromUtf8("연결 끊기 (%1)").arg(viewer_->ModelName(model)));
            floor = menu.addAction(QString::fromUtf8("바닥으로 지정 (%1)").arg(viewer_->ModelName(model)));   // 다른 모델이 이 모델의 윗면 위로 떨어져 멈춘다
            floor->setCheckable(true);
            floor->setChecked(viewer_->FloorModel() == model);
            remove = menu.addAction(QString::fromUtf8("제거 (%1)").arg(viewer_->ModelName(model)));
        }
        QAction* chosen = menu.exec(globalPos);   // 끝나면 체크 상태는 이미 바뀌어 있다
        if (!chosen) return;
        if (chosen == beams) showLidarBeams(beams->isChecked());
        else if (chosen == graph) showLidarGraph(graph->isChecked());
        else if (chosen == place) askPlacement(model);
        else if (chosen == connect) askConnect(model);
        else if (chosen == disconnect) {
            if (viewer_->DisconnectModel(model)) log_->Info(QString::fromUtf8("연결 끊기: %1 (월드에 고정, 자리는 그대로)").arg(viewer_->ModelName(model)));
        }
        else if (chosen == floor) {
            viewer_->SetFloorModel(floor->isChecked() ? model : -1);
            log_->Info(floor->isChecked() ? QString::fromUtf8("바닥으로 지정: %1").arg(viewer_->ModelName(model)) : QString::fromUtf8("바닥 지정 해제"));
        }
        else if (chosen == remove) removeModel(model);
    }

    // 모델을 다른 모델의 링크에 관절로 연결한다 (대화상자)
    void askConnect(int model) {
        JointDialog dlg(viewer_, model, viewer_->SelectedLink(), this);
        if (dlg.ParentLinkCount() == 0) {
            QMessageBox::information(this, QString::fromUtf8("관절로 연결"), QString::fromUtf8("연결할 수 있는 부모 링크가 없습니다. 다른 모델을 먼저 올려 주세요."));
            return;
        }
        if (dlg.exec() != QDialog::Accepted) return;
        QString err;
        const int parentLink = dlg.ParentLink();
        if (!dlg.Apply(&err)) {
            logError(QString::fromUtf8("관절로 연결 실패: %1").arg(err));
            QMessageBox::warning(this, QString::fromUtf8("관절로 연결"), err);
            return;
        }
        static const char* kinds[4] = {"fixed", "revolute", "continuous", "prismatic"};
        const simul::urdf::AttachSpec& a = viewer_->ModelAttach(model);
        const int k = a.type == simul::urdf::kJointRevolute ? 1 : a.type == simul::urdf::kJointContinuous ? 2 : a.type == simul::urdf::kJointPrismatic ? 3 : 0;
        QString jointName;
        const simul::urdf::Robot& rb = viewer_->Robot();
        for (size_t i = 0; i < rb.links.size(); i++)
            if (viewer_->ModelOfRoot(static_cast<int>(i)) == model)
                for (size_t j = 0; j < rb.joints.size(); j++)
                    if (rb.joints[j].child == rb.links[i].name) jointName = simulqt::ToQ(rb.joints[j].name);
        log_->Info(QString::fromUtf8("관절로 연결: %1 -> %2 (%3, 관절 %4)")
                       .arg(viewer_->ModelName(model)).arg(simulqt::ToQ(rb.links[static_cast<size_t>(parentLink)].name)).arg(kinds[k]).arg(jointName));
        updateSelectionLabel();
    }

    // 모델의 월드 위치와 자세를 입력받는다
    void askPlacement(int model) {
        float v[6];
        viewer_->ModelPlacement6(model, v);
        bool ok = false;
        const QString text = QInputDialog::getText(this, QString::fromUtf8("위치 지정"),
                                                   QString::fromUtf8("%1\nx, y, z (m), roll, pitch, yaw (도). 자세는 생략하면 0\n(Rz(yaw)·Ry(pitch)·Rx(roll))%2").arg(viewer_->ModelName(model)).arg(viewer_->ModelConnected(model) ? QString::fromUtf8("\n관절로 연결된 모델이라 부모 링크 기준의 관절 위치입니다") : QString()),
                                                   QLineEdit::Normal, QString("%1, %2, %3, %4, %5, %6").arg(v[0]).arg(v[1]).arg(v[2]).arg(v[3]).arg(v[4]).arg(v[5]), &ok);
        if (!ok) return;
        QString flat = text;
        flat.replace(',', ' ');
        const QStringList p = flat.simplified().split(' ', QString::SkipEmptyParts);
        double d[6] = {0, 0, 0, 0, 0, 0};
        bool good = p.size() == 3 || p.size() == 6;
        for (int i = 0; good && i < p.size(); i++) d[i] = p[i].toDouble(&good);
        if (!good) {
            QMessageBox::warning(this, QString::fromUtf8("위치 지정"), QString::fromUtf8("숫자 3개(x, y, z) 또는 6개(+ roll, pitch, yaw)를 넣어 주세요."));
            return;
        }
        float f[6];
        for (int i = 0; i < 6; i++) f[i] = static_cast<float>(d[i]);
        viewer_->SetModelPlacement6(model, f);
        log_->Info(QString::fromUtf8("%1 위치 지정: %2, %3, %4 m, RPY %5°, %6°, %7°").arg(viewer_->ModelName(model)).arg(d[0]).arg(d[1]).arg(d[2]).arg(d[3]).arg(d[4]).arg(d[5]));
        updateCollisionLabel();
    }

    void removeModel(int model) {
        const bool beamsWere = viewer_->LidarEnabled() && viewer_->LidarDrawn();
        const bool graphWere = lidarDock_ && lidarDock_->isVisible();
        const QString name = viewer_->ModelName(model);
        showLidar(false);
        viewer_->RemoveModel(model);
        log_->Info(QString::fromUtf8("제거: %1  (남은 모델 %2개)").arg(name).arg(viewer_->ModelCount()));
        afterLoad(QString::fromUtf8("%1 제거").arg(name));
        applyLidar(cli_);
        applyCamera(cli_);
        if (beamsWere && lidarSpec_.link >= 0) showLidarBeams(true);
        if (graphWere && lidarSpec_.link >= 0) showLidarGraph(true);
    }

    void applyMove(const Options& o) {
        if (o.hasMove) viewer_->SetObjectPosition(o.move[0], o.move[1], o.move[2]);
    }

    // 확인용: 모터를 끈 시뮬레이션을 seconds 초만큼 (1/60 초씩) 진행하고 관절 값을 출력한다
    void runSimulation(double seconds) {
        if (!viewer_->HasRobot() || !motorsOff_) return;
        for (double t = 0; t < seconds - 1e-9; t += 1.0 / 60.0) stepSimulation(1.0 / 60.0);
        const simul::urdf::JointValues q = sim_.Values();
        const simul::urdf::Robot& robot = viewer_->Robot();
        printf("시뮬레이션 %.2f 초 후 (모터 꺼짐, 중력 %.3f, %.3f, %.3f):\n", seconds, gravity_.x, gravity_.y, gravity_.z);
        const simul::Array<int> act = robot.ActuatedJoints();
        for (size_t i = 0; i < act.size(); i++) {
            const simul::urdf::Joint& j = robot.joints[static_cast<size_t>(act[i])];
            const simul::urdf::JointValues::const_iterator it = q.find(j.name);
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
        for (int i = 0; i < urls.size(); i++) openFile(urls[i].toLocalFile());
    }

private:
    // 모터 끄기: 켜면 지금 관절 값에서 시작해서 중력, 관성, 마찰로 움직인다. 끄면(모터 켜짐) 지금 자세를 유지한다.
    void setMotorsOff(bool on) {
        motorsOff_ = on;
        log_->Info(on ? QString::fromUtf8("모터 끔: 관절이 중력으로 움직입니다") : QString::fromUtf8("모터 켬"));
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
        simul::dynamics::DynamicsOptions opt;
        opt.defaultDamping = 0.2;  // URDF 에 감쇠가 없는 관절도 영원히 흔들리지 않도록 약한 점성 감쇠를 준다
        sim_.SetOptions(opt);
    }

    void stepSimulation(double dt) {
        if (!sim_.IsReady()) return;
        sim_.Step(dt);
        const simul::urdf::JointValues q = sim_.Values();
        viewer_->SetJointValues(q);
        panel_->ShowValues(q);
        // 모터 토크와 구조 트리의 글자는 사람이 읽는 속도면 충분하다 (초당 10번). 매 프레임 계산하고 문자열을 만들면 관절이 많은 장면에서 프레임이 떨어진다.
        if (!torqueClock_.isValid() || torqueClock_.elapsed() >= 100) {
            updateTorques(q);
            torqueClock_.start();
        }
        updateSelectionLabel();
    }

    // 좌표계 OBJ 를 찾아서 뷰어에 넣는다: 소스의 Data 폴더(개발 중), 실행 파일 옆 Data 폴더(배포) 순서
    void loadGizmo() {
        QStringList dirs;
#ifdef SIMUL_DATA_DIR
        dirs << QString::fromUtf8(SIMUL_DATA_DIR);
#endif
        if (!cli_.embeddedDir.empty()) dirs << simulqt::ToQ(cli_.embeddedDir) + QStringLiteral("/__simul/Data");   // --pack 으로 묶은 실행 파일
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
            name = simulqt::ToQ(viewer_->Robot().links[static_cast<size_t>(viewer_->SelectedLink())].name);
        const dhPose pose = dhPose::FromMat(f).Normalized();
        const simul::units::RpyDeg r = simul::units::RpyDegFromQuat(pose.orientation);
        statusBar()->showMessage(QString::fromUtf8("선택 %1 | 위치 %2, %3, %4 m | RPY %5°, %6°, %7° | 쿼터니언 %8, %9, %10, %11  | %12")
                                     .arg(name)
                                     .arg(static_cast<double>(pose.position.x), 0, 'f', 3).arg(static_cast<double>(pose.position.y), 0, 'f', 3).arg(static_cast<double>(pose.position.z), 0, 'f', 3)
                                     .arg(r.roll, 0, 'f', 1).arg(r.pitch, 0, 'f', 1).arg(r.yaw, 0, 'f', 1)
                                     .arg(static_cast<double>(pose.orientation.x), 0, 'f', 3).arg(static_cast<double>(pose.orientation.y), 0, 'f', 3)
                                     .arg(static_cast<double>(pose.orientation.z), 0, 'f', 3).arg(static_cast<double>(pose.orientation.w), 0, 'f', 3)
                                     .arg(viewer_->CanMoveSelection() ? QString::fromUtf8("화살표를 끌어 이동, 링을 끌어 회전") : QString::fromUtf8("관절로 움직이는 링크")));
    }

    // 현재 자세와 중력으로 관절별 중력 하중을 다시 계산해서 관절 패널(모터 토크)과 구조 트리(모든 관절)에 보여준다
    void updateTorques() { updateTorques(panel_->Values()); }
    void updateTorques(const simul::urdf::JointValues& q) {
        if (!viewer_->HasRobot()) return;
        const simul::urdf::Robot& robot = viewer_->Robot();
        const simul::dynamics::JointTorques motor = simul::dynamics::GravityTorques(robot, q, gravity_);
        panel_->SetTorques(motor);
        const QString dockTitle = QString::fromUtf8("관절  (중력 %1, %2, %3 m/s²)").arg(gravity_.x, 0, 'g', 5).arg(gravity_.y, 0, 'g', 5).arg(gravity_.z, 0, 'g', 5);
        if (dock_->windowTitle() != dockTitle) dock_->setWindowTitle(dockTitle);

        const simul::Array<simul::dynamics::JointLoad> loads = simul::dynamics::GravityLoads(robot, q, gravity_);
        simul::Map<simul::String, simul::Pair<QString, bool> > notes;
        for (size_t i = 0; i < robot.joints.size(); i++) {
            const simul::urdf::Joint& j = robot.joints[i];
            const simul::dynamics::JointLoad& l = loads[i];
            QString text;
            bool warn = false;
            if (!j.IsMovable()) {
                const double f = simul::Sqrt(l.force[0] * l.force[0] + l.force[1] * l.force[1] + l.force[2] * l.force[2]);
                const double m = simul::Sqrt(l.moment[0] * l.moment[0] + l.moment[1] * l.moment[1] + l.moment[2] * l.moment[2]);
                text = QString::fromUtf8("체결 하중 힘 %1 N, 모멘트 %2 N·m").arg(f, 0, 'f', 2).arg(m, 0, 'f', 2);
            } else if (j.hasMimic) {
                text = QString::fromUtf8("자기 몫 %1 %2 (원본 %3 에 합산)").arg(l.torque, 0, 'f', 3).arg(j.TorqueUnit(), simulqt::ToQ(j.mimicJoint));
            } else {
                const double tau = motor.find(j.name)->second;
                text = QString::fromUtf8("모터 %1 %2").arg(tau, 0, 'f', 3).arg(j.TorqueUnit());
                if (j.effort > 0.0f && simul::Abs(tau) > static_cast<double>(j.effort)) {
                    text += QString::fromUtf8("  (한계 %1 초과)").arg(static_cast<double>(j.effort), 0, 'g', 4);
                    warn = true;
                }
            }
            notes[j.name] = simul::MakePair(text, warn);
        }
        tree_->SetJointNotes(notes);
    }

    // 모든 도크를 떼어 내(플로팅) 아무 데나 놓거나 다시 붙이고, 닫을 수 있게 한다
    void configureDock(QDockWidget* d) {
        if (!d) return;
        d->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetClosable);
        d->setAllowedAreas(Qt::AllDockWidgetAreas);
    }
    void configureDocks() {
        configureDock(dock_);
        configureDock(propDock_);
        configureDock(urdfDock_);
        configureDock(folderDock_);
        configureDock(logDock_);
        configureDock(lidarDock_);
    }

    // 3D 화면만 남긴다: 메뉴, 상태바, 모든 도크(떼어 낸 것 포함)를 숨겼다가, 끄면 보이던 것만 되살린다
    void setUiHidden(bool on) {
        if (on == uiHidden_) return;
        uiHidden_ = on;
        view_->SetMenus(nativeMenus_);   // 같은 메뉴(동작, 체크 상태 포함)를 3D 화면 위에 띄운다
        view_->EnableFeature(SimulView::kMenu, on);
        const QList<QDockWidget*> docks = findChildren<QDockWidget*>();
        if (on) {
            hiddenDocks_.clear();
            for (int i = 0; i < docks.size(); i++) {
                if (docks[i]->isVisible()) hiddenDocks_.push_back(docks[i]);
                docks[i]->hide();
            }
            menuWasVisible_ = menuBar()->isVisible();
            statusWasVisible_ = statusBar()->isVisible();
            menuBar()->hide();
            statusBar()->hide();
        } else {
            for (int i = 0; i < hiddenDocks_.size(); i++) hiddenDocks_[i]->show();
            hiddenDocks_.clear();
            menuBar()->setVisible(menuWasVisible_);
            statusBar()->setVisible(statusWasVisible_);
        }
        const bool b = menuAct_->blockSignals(true);
        menuAct_->setChecked(on ? false : menuWasVisible_);
        menuAct_->blockSignals(b);
        const bool b2 = onlyViewAct_->blockSignals(true);
        onlyViewAct_->setChecked(on);
        onlyViewAct_->blockSignals(b2);
    }

    void logError(const QString& text) {
        log_->Error(text);
        logDock_->show();   // 오류는 바로 보이게 한다
    }

    void updateCollisionLabel() {
        if (!viewer_->HasRobot()) {
            collisionLabel_->clear();
            return;
        }
        const QStringList d = viewer_->CollisionDescriptions();
        if (d != lastCollisions_) {   // 충돌 쌍이 바뀔 때만 기록한다
            if (!d.isEmpty()) log_->Warning(QString::fromUtf8("OBB 충돌 %1건: %2").arg(d.size()).arg(d.join(", ")));
            else if (!lastCollisions_.isEmpty()) log_->Info(QString::fromUtf8("충돌 해소"));
            lastCollisions_ = d;
        }
        const int cand = static_cast<int>(viewer_->AabbCandidates().size());
        const bool hit = !d.isEmpty();
        const QString text = hit ? QString::fromUtf8("OBB 충돌 %1건: %2 (AABB 후보 %3건)").arg(d.size()).arg(d.join(", ")).arg(cand)
                                 : QString::fromUtf8("충돌 없음 (AABB 후보 %1건)").arg(cand);
        if (collisionLabel_->text() != text) collisionLabel_->setText(text);
        if (hit != collisionLabelHit_) {   // 스타일시트를 바꾸면 위젯을 다시 꾸미므로 상태가 바뀔 때만 한다
            collisionLabel_->setStyleSheet(hit ? QString("color: #d33; font-weight: bold;") : QString());
            collisionLabelHit_ = hit;
        }
    }

    void afterLoad(const QString& path) {
        if (viewer_->HasRobot()) {
            tree_->SetRobot(viewer_->Robot());
            tree_->SetCollidingLinks(viewer_->CollidingLinks());
            panel_->SetRobot(viewer_->Robot());
            {   // 모델을 더하거나 뺄 때 이름이 남아 있는 관절 값은 그대로 둔다
                const simul::urdf::JointValues& q = viewer_->CurrentJointValues();
                panel_->blockSignals(true);
                for (simul::urdf::JointValues::const_iterator it = q.begin(); it != q.end(); ++it) panel_->SetValue(simulqt::ToQ(it->first), it->second);
                panel_->blockSignals(false);
            }
            updateTorques();
        } else {
            if (viewer_->HasScene()) tree_->SetScene(viewer_->StaticScene());
            else tree_->Clear();
            dock_->hide();
        }
        updateCollisionLabel();
        QString msg = path + "  -  " + viewer_->Summary();
        if (!viewer_->Warnings().isEmpty()) msg += QString::fromUtf8("  (경고 %1건: %2)").arg(viewer_->Warnings().size()).arg(viewer_->Warnings()[0]);
        resetSimulation();
        infoMessage_ = msg;
        statusBar()->showMessage(msg);
        updateSelectionLabel();
        setWindowTitle(viewer_->ModelCount() > 1 ? QString("Project Simul - %1").arg(QString::fromUtf8("모델 %1개").arg(viewer_->ModelCount())) : QString("Project Simul - ") + path);
    }

    Options cli_;  // 명령행 옵션 (파일을 새로 열 때도 쓴다)
    struct LidarSpec {
        int link;
        simul::sensor::LidarSettings settings;
        dhMat mount;
        bool fromUrdf;
        LidarSpec() : link(-1), fromUrdf(false) {}
    };
    LidarSpec lidarSpec_;
    struct CameraSpec {
        int link;
        simul::sensor::CameraSettings settings;
        dhMat mount;
        CameraSpec() : link(-1) {}
    };
    CameraSpec cameraSpec_;
    CameraView* cameraView_ = nullptr;
    QDockWidget* cameraDock_ = nullptr;
    ModelViewer* viewer_;
    LidarPlot* lidarPlot_ = nullptr;
    QDockWidget* lidarDock_ = nullptr;
    JointPanel* panel_;
    QList<QMenu*> nativeMenus_;
    QAction* menuAct_ = nullptr;
    QAction* onlyViewAct_ = nullptr;
    bool uiHidden_ = false;           // 3D 화면만 보는 중
    bool menuWasVisible_ = true;
    bool statusWasVisible_ = true;
    QList<QDockWidget*> hiddenDocks_;   // 3D 화면만 보기 전에 보이던 도크
    LogView* log_ = nullptr;
#ifdef SIMUL_HAS_PYTHON
    ViewerBackend* backend_ = nullptr;
    simul::sim::Engine* engine_ = nullptr;
    SimulRunner* runner_ = nullptr;
    simul::py::Runtime* py_ = nullptr;
    bool pyReady_ = false;
    PythonPanel* pyPanel_ = nullptr;
    QDockWidget* pyDock_ = nullptr;
    ObjectInfoPanel* objPanel_ = nullptr;
    QMenu* fileMenu_ = nullptr;
    Options pyOpt_;
    QString projectFile_;                          // 지금 열었거나 저장한 project.simul
    QList<QPair<QString, QString> > regScripts_;   // 등록한 python 파일과 권한 (프로젝트로 저장할 때 담는다)
    QDockWidget* objDock_ = nullptr;
#endif
    QDockWidget* logDock_ = nullptr;
    QDockWidget* propDock_ = nullptr;
    QDockWidget* urdfDock_ = nullptr;
    QDockWidget* folderDock_ = nullptr;
    FolderView* folderView_ = nullptr;
    SunPanel* sunPanel_ = nullptr;
    QDockWidget* sunDock_ = nullptr;
    QSet<QString> loggedWarnings_;
    QStringList lastCollisions_;
    bool collisionLabelHit_ = false;
    SimulView* view_ = nullptr;
    QDockWidget* dock_;
    StructurePanel* tree_;
    QLabel* collisionLabel_;
    simul::dynamics::Simulator sim_;
    QTimer* simTimer_;
    QElapsedTimer simClock_;
    QElapsedTimer torqueClock_;   // 시뮬레이션 중 토크 표시를 마지막으로 갱신한 때
    bool motorsOff_;
    bool simManual_;   // --simulate: 타이머 없이 runSimulation 으로만 진행
    QString infoMessage_;   // 선택이 없을 때 상태바에 보이는 파일 정보
    simul::dynamics::Gravity gravity_;
};

}  // namespace

int main(int argc, char** argv) {
    InitConsole();
    Options o;
    const int pr = ParseArgs(argc, argv, &o);
    if (pr == 1) return 0;
    if (pr == 2) return 2;
    if (!o.makeMarker.empty()) return MakeMarkerFiles(o);
    if (o.info) return PrintInfo(o);
    if (!o.packProject.empty()) {   // --pack: 창 없이 하나의 실행 파일로 만든다
        QCoreApplication core(argc, argv);
        QString out = simulqt::ToQ(o.packOut), err;
        const QFileInfo pf(simulqt::ToQ(o.packProject));
        if (out.isEmpty()) out = pf.absoluteDir().absoluteFilePath(pf.absoluteDir().dirName()
#ifdef _WIN32
                                                                   + QStringLiteral(".exe")
#endif
                                                                   );
        QString dataDir = QCoreApplication::applicationDirPath() + QStringLiteral("/Data");
#ifdef SIMUL_DATA_DIR
        if (!QFileInfo(dataDir + QStringLiteral("/Python/Main.py")).isFile()) dataDir = QString::fromUtf8(SIMUL_DATA_DIR);
#endif
        int n = 0;
        if (!projectpack::PackProject(simulqt::ToQ(o.packProject), dataDir, QCoreApplication::applicationFilePath(), out, &err, &n)) {
            fprintf(stderr, "묶기 실패: %s\n", err.toUtf8().constData());
            return 2;
        }
        printf("프로젝트를 하나의 실행 파일로 묶었습니다 (파일 %d개): %s\n", n, out.toUtf8().constData());
        return 0;
    }

    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    {   // 이 실행 파일에 프로젝트가 붙어 있으면(--pack 으로 만든 것) 그것을 풀어서 연다
        QString dir, perr;
        const projectpack::Found found = projectpack::ExtractEmbedded(QCoreApplication::applicationFilePath(), &dir, &perr);
        if (found == projectpack::kFailed) {
            QMessageBox::critical(nullptr, QString::fromUtf8("프로젝트를 열 수 없음"), perr);
            return 1;
        }
        if (found == projectpack::kExtracted) {
            o.embeddedDir = simulqt::FromQ(dir);
            o.projectFile = simulqt::FromQ(dir + QStringLiteral("/project.simul"));
        }
    }
    MainWindow w(o);
    w.show();
    if (!o.projectFile.empty()) {
        w.loadProject(&o);
    } else if (!o.file.empty()) {
        if (!w.openFile(QString::fromLocal8Bit(o.file.c_str()))) {
            if (!o.shot.empty() || !o.shotWindow.empty()) return 1;
        } else {
            for (size_t i = 0; i < o.moreFiles.size(); i++) w.openFile(QString::fromLocal8Bit(o.moreFiles[i].c_str()));
            w.applyScales(o);
            w.applyPlaces(o);
            w.applyFloor(o);
            w.applyJoints(o);
            w.applySelect(o);
            w.applyMove(o);
        }
    }
    if (!o.shapes.empty()) w.applyShapes(o);
    w.applyConnects(o);   // 도형까지 올라간 뒤에 연결한다
    if (!o.exportDir.empty()) {   // 확인용/배치용: 합친 장면을 폴더로 저장하고 끝낸다
        if (!w.exportFolder(QString::fromLocal8Bit(o.exportDir.c_str()), o.exportFormat)) return 1;
        printf("폴더로 저장: %s\n", o.exportDir.c_str());
        if (o.exportUrdf.empty()) return 0;
    }
    if (!o.exportUrdf.empty()) {   // 확인용/배치용: 합친 장면을 URDF 로 저장하고 끝낸다
        if (!w.exportUrdf(QString::fromLocal8Bit(o.exportUrdf.c_str()))) return 1;
        printf("URDF 저장: %s\n", o.exportUrdf.c_str());
        return 0;
    }
    w.startHidden(o);
    w.setupPython(o);
#ifdef SIMUL_HAS_PYTHON
    if (!o.saveProjectDir.empty()) {   // 확인용/배치용: 프로젝트 폴더로 저장하고 끝낸다
        if (!w.writeProject(QString::fromLocal8Bit(o.saveProjectDir.c_str()))) return 1;
        printf("프로젝트 저장: %s\n", o.saveProjectDir.c_str());
        return 0;
    }
#endif
    if (!o.shot.empty() || !o.shotWindow.empty()) {
        const simul::String shot = o.shot, shotWindow = o.shotWindow;
        ModelViewer* v = w.viewer();
        MainWindow* win = &w;
        const int cx = o.clickX, cy = o.clickY;
        const bool settle = o.settle;
        const double simSeconds = o.simulate;
        const simul::String lidarCsv = o.lidarCsv, cameraSave = o.cameraSave;
        const int d0x = o.dragX0, d0y = o.dragY0, d1x = o.dragX1, d1y = o.dragY1;
        QTimer::singleShot(600, &app, [&app, v, win, shot, shotWindow, cx, cy, d0x, d0y, d1x, d1y, settle, simSeconds, lidarCsv, cameraSave]() {
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
                    const simul::Array<int> act = v->Robot().ActuatedJoints();
                    for (size_t i = 0; i < act.size(); i++) {
                        const simul::urdf::Joint& j = v->Robot().joints[static_cast<size_t>(act[i])];
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
                const simul::sensor::LidarScan& sc = v->ScanLidar();
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
                    simul::String csv = "index,channel,azimuth_deg,elevation_deg,hit,range_m,x,y,z,link\n";
                    for (size_t i = 0; i < sc.size(); i++) {
                        const bool hit = sc.IsHit(i);
                        csv += simul::Format("%zu,%zu,%.4f,%.4f,%d,", i, sc.Channel(i), dhDeg(static_cast<double>(sc.Angle(i))),
                                           dhDeg(static_cast<double>(sc.Elevation(sc.Channel(i)))), hit ? 1 : 0);
                        if (hit)
                            csv += simul::Format("%.5f,%.5f,%.5f,%.5f,%s\n", static_cast<double>(sc.ranges[i]), static_cast<double>(sc.points[i].x),
                                               static_cast<double>(sc.points[i].y), static_cast<double>(sc.points[i].z),
                                               v->Robot().links[static_cast<size_t>(sc.targetIds[i])].name.c_str());
                        else
                            csv += "inf,,,,\n";
                    }
                    if (simul::WriteFileText(lidarCsv, csv)) printf("스캔을 저장했습니다: %s\n", lidarCsv.c_str());
                    else fprintf(stderr, "스캔 저장 실패: %s\n", lidarCsv.c_str());
                }
            }
            if (v->SensorCameraEnabled()) {   // 확인용: 영상 요약, --camera-save 면 저장
                const simul::sensor::CameraImage& im = v->RenderSensorCamera();
                size_t hits = 0;
                float lo = 1e30f, hi = 0;
                for (size_t i = 0; i < im.depth.size(); i++)
                    if (im.targetIds[i] >= 0) {
                        hits++;
                        if (im.depth[i] < lo) lo = im.depth[i];
                        if (im.depth[i] > hi) hi = im.depth[i];
                    }
                printf("카메라: %dx%d, 픽셀 %zu 개 중 %zu 개가 물체", im.width, im.height, im.depth.size(), hits);
                if (hits) printf(", 깊이 %.3f ~ %.3f m", static_cast<double>(lo), static_cast<double>(hi));
                printf("\n");
                if (!cameraSave.empty()) {
                    const QString base = QString::fromLocal8Bit(cameraSave.c_str());
                    const QString stem = base.endsWith(".png", Qt::CaseInsensitive) ? base.left(base.size() - 4) : base;
                    const simul::sensor::CameraSettings& cs = v->SensorCameraSettings();
                    bool okc = CameraView::ColorToQImage(im).save(stem + ".png");
                    okc = CameraView::DepthToQImage(im, cs.nearClip, cs.farClip).save(stem + "_depth.png") && okc;
                    QImage d16(im.width, im.height, QImage::Format_Grayscale16);
                    for (int y = 0; y < im.height; y++) {
                        quint16* row = reinterpret_cast<quint16*>(d16.scanLine(y));
                        for (int x = 0; x < im.width; x++) {
                            const float mm = im.Depth(x, y) * 1000.0f;
                            row[x] = static_cast<quint16>(mm > 65535.0f ? 65535.0f : mm);
                        }
                    }
                    okc = d16.save(stem + "_depth16.png") && okc;
                    if (okc) printf("카메라 영상을 저장했습니다: %s(.png, _depth.png, _depth16.png)\n", qPrintable(stem));
                    else fprintf(stderr, "카메라 영상 저장 실패: %s\n", cameraSave.c_str());
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
    if (!o.makeMarker.empty()) return MakeMarkerFiles(o);
    if (o.file.empty()) {
        PrintUsage(argv[0]);
        printf("\n(이 빌드는 Qt 없이 만들어져 3D 창이 없습니다. -DSIMUL_BUILD_QT=ON 으로 다시 빌드하세요.)\n");
        return 0;
    }
    return PrintInfo(o);
}

#endif
