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
#include "CSimul/Patrol.h"

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
    struct Scale { simul::String model; double factor; };   // --scale 모델=배율
    simul::Array<Scale> scales;
    simul::String floorModel;   // --floor 모델: 바닥으로 지정할 모델 이름
    simul::String patrolFile;   // --patrol 파일: 지점 파일대로 이름이 같은 모델이 걷는다 (막히면 A* 로 비킴)
    int servePort = 0;          // --serve 포트: 웹 접속 게이트웨이(web/server.js)가 붙는 포트. 웹 접속자가 엔진의 오브젝트로 올라온다
    simul::String dataDir;      // 마을 데이터(Data/Object)가 있는 폴더 (마을을 열 때 정해진다)
    bool npc = false;           // --npc: 마을에 주민(NPC) 4명도 같이 연다 (기본은 마을 지도만)
    bool empty = false;         // --empty: 아무것도 열지 않고 시작 (기본은 파일을 안 주면 마을을 연다)
    bool noChat = false;        // --no-chat: 주민끼리 대화하지 않는다
    bool noLlm = false;         // --no-llm: LLM 을 쓰지 않고 정해 둔 대사로만 말한다
    simul::String llmUrl = "http://127.0.0.1:11434";   // --llm-url: Ollama 서버 주소
    simul::String llmModel = "qwen2.5:1.5b";           // --llm-model: Ollama 모델 이름
    bool onlyView;             // 기본: 메뉴/도크/상태바 없이 3D 화면만 (메뉴, 구조 트리, 오브젝트 뷰는 3D 화면 안에 뜬다). --classic 이면 예전 창 구성
    bool hideMenu;             // --hide-menu: 메뉴만 숨김
    double uiScale = 1.0;      // --ui-scale: 3D 화면 안 UI 크기 배율
    int shading;               // 조명 방식 0 Lambert, 1 Blinn-Phong, 2 PBR (기본)
    bool info;                 // 창 없이 정보만 출력
    bool objPanel;             // --object-panel: 오브젝트 / 관절 값 도크를 연 채로 시작
    simul::String projectFile; // --project 파일: project.simul 을 열어 장면, 스크립트 등록, 자동 시작까지 한다
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
    Options() : onlyView(true), hideMenu(false), shading(2), info(false), objPanel(false), aabb(false), obb(false), torque(false), dragX0(-1), dragY0(-1), dragX1(-1), dragY1(-1), motorsOff(false), simulate(0), noFall(false), settle(false), hasMove(false), clickX(-1), clickY(-1), width(1100), height(720) {
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
        "  --object-panel     오브젝트 / 관절 값 도크를 연 채로 시작 (메뉴 보기에서도 켠다, Ctrl+J)\n"
        "  --project 파일     project.simul 을 연다 (모델과 배치를 복원)\n"
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
        "  --patrol 파일       지점 파일(Data/Object/villagers/waypoints.txt 형식)대로 이름이 같은 모델이 지점을 차례로 걷는다 (막히면 A* 로 비킨다)\n"
        "  --serve [포트]     웹 접속을 받는다 (기본 9000, web/server.js 가 붙는다). 웹 접속자가 엔진의 오브젝트로 올라와 이 화면에도 보인다\n"
        "  --npc              마을에 주민(NPC) 4명도 연다 (걷고, 마주치면 이야기한다). 기본은 마을 지도만 열린다\n"
        "  --empty            파일을 안 줬을 때도 마을을 열지 않고 빈 뷰어로 시작 (기본은 Data/Object 의 마을 지도만 열린다)\n"
        "  --no-chat          주민끼리 대화하지 않는다 (기본은 마주치면 멈춰서 이야기한다)\n"
        "  --no-llm           LLM 없이 정해 둔 대사로만 말한다\n"
        "  --llm-url URL      LLM 서버 (Ollama) 주소 (기본 http://127.0.0.1:11434)\n"
        "  --llm-model 이름   LLM 모델 이름 (기본 qwen2.5:1.5b)\n"
        "  --floor 모델        그 모델을 바닥으로 지정 (다른 모델이 z=0 평면 대신 그 윗면 위로 떨어져 멈춘다). 예: --floor table\n"
        "  --scale 모델=배율   모델 전체 크기를 원래의 몇 배로 (길이와 질량이 함께 바뀐다). 예: --scale arm=0.5\n"
        "  --classic            예전 창 구성(Qt 메뉴줄과 도크)으로 시작한다. 기본은 3D 화면만 (메뉴/구조 트리/오브젝트 뷰가 화면 안에 뜬다)\n"
        "  --only-view / --hide-menu  3D 화면만(기본) / 메뉴만 숨김 (실행 중에는 F11 / F10, 3D 만 보기는 Esc 로도 돌아옴)\n"
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
        if (a == "--object-panel") { o->objPanel = true; continue; }
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
        if (a == "--classic") { o->onlyView = false; continue; }
        if (a == "--ui-scale" && i + 1 < argc) { o->uiScale = atof(argv[++i]); continue; }
        if (a == "--hide-menu") { o->hideMenu = true; continue; }
        if (a == "--floor" && hasNext) { o->floorModel = argv[++i]; continue; }
        if (a == "--patrol" && hasNext) { o->patrolFile = argv[++i]; continue; }
        if (a == "--empty") { o->empty = true; continue; }
        if (a == "--npc") { o->npc = true; continue; }
        if (a == "--serve") { o->servePort = (hasNext && atoi(argv[i + 1]) > 0) ? atoi(argv[++i]) : 9000; continue; }
        if (a == "--no-chat") { o->noChat = true; continue; }
        if (a == "--no-llm") { o->noLlm = true; continue; }
        if (a == "--llm-url" && hasNext) { o->llmUrl = argv[++i]; continue; }
        if (a == "--llm-model" && hasNext) { o->llmModel = argv[++i]; continue; }
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
#include "CoreQt/ObjectInfoPanel.h"
#include "CoreQt/LogView.h"
#include <QSet>
#include <QKeyEvent>
#include <QLineEdit>
#include <QLabel>
#include <QMainWindow>
#include <QMenuBar>
#include <QCheckBox>
#include <QDateTime>
#include <QPushButton>
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
#include "CoreQt/JointPanel.h"
#include "CoreQt/CameraView.h"
#include "CoreQt/LidarPlot.h"
#include "CoreQt/AStarView.h"
#include "CoreQt/LlmClient.h"
#include "CoreQt/SpeechBubbles.h"
#include "CSimul/Chat.h"
#include "CSimul/Players.h"
#include "CoreQt/GameLink.h"
#include "CoreQt/SunPanel.h"
#include "CoreQt/ModelViewer.h"
#include "CoreQt/StructurePanel.h"

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
        addDockWidget(Qt::BottomDockWidgetArea, logDock_);
        logDock_->hide();
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
        file->addSeparator();
        QAction* quit = file->addAction(QString::fromUtf8("종료"));
        quit->setShortcut(QKeySequence::Quit);
        QObject::connect(quit, &QAction::triggered, this, &QWidget::close);
        QMenu* view = menuBar()->addMenu(QString::fromUtf8("보기"));      // 화면에 보이는 것
        QMenu* phys = menuBar()->addMenu(QString::fromUtf8("물리"));      // 중력, 모터, 낙하
        QMenu* sensorMenu = menuBar()->addMenu(QString::fromUtf8("센서"));  // 라이다 표시
        QMenu* help = menuBar()->addMenu(QString::fromUtf8("도움말"));
        nativeMenus_ << file << view << phys << sensorMenu << help;   // 3D 화면만 보기에서는 이 메뉴들을 3D 화면 안의 메뉴줄에 그대로 보여 준다
        QAction* fit = view->addAction(QString::fromUtf8("화면에 맞춤"));
        fit->setShortcut(Qt::CTRL + Qt::Key_F);
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
        structure->setShortcut(Qt::CTRL + Qt::Key_T);
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
        QAction* logAct = logDock_->toggleViewAction();
        logAct->setText(QString::fromUtf8("로그"));
        logAct->setShortcut(Qt::CTRL + Qt::Key_L);
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
        sunAct->setShortcut(Qt::CTRL + Qt::Key_S);
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
        QObject::connect(esc, &QShortcut::activated, this, [this]() {
            if (fpModel_ >= 0) { leaveFirstPerson(); return; }
            if (uiHidden_) onlyViewAct_->setChecked(false);
        });
        view->insertSeparator(viewSep);
        configureDocks();
        QAction* frame = new QAction(QString::fromUtf8("선택 좌표계"), this);
        view->insertAction(viewSep, frame);
        frame->setShortcut(Qt::CTRL + Qt::Key_G);
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
        motors->setShortcut(Qt::CTRL + Qt::Key_M);
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
        obbs->setShortcut(Qt::CTRL + Qt::SHIFT + Qt::Key_O);
        obbs->setCheckable(true);
        obbs->setChecked(o.obb);
        viewer_->SetShowObbs(o.obb);
        QObject::connect(obbs, &QAction::toggled, viewer_, &ModelViewer::SetShowObbs);
        QAction* boxes = colMenu->addAction(QString::fromUtf8("AABB (축에 맞춘 박스, 빠른 1차 검사)"));
        boxes->setShortcut(Qt::CTRL + Qt::Key_B);
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
                "  Ctrl+O 열기   Ctrl+F 화면에 맞춤   Ctrl+T 구조 트리   Ctrl+G 선택 좌표계\n"
                "  Ctrl+M 모터 끄기   Ctrl+Shift+O OBB 박스   Ctrl+B AABB 박스\n"
                "  Ctrl+L 로그   Ctrl+S 태양   Ctrl+J 오브젝트 값   Ctrl+K A* 보기   F1 이 도움말"));
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

    // 파일을 열어 장면에 추가한다 (이전 모델은 지우지 않는다). 지우려면 구조 트리에서 우클릭 -> 제거, 또는 파일 -> 모두 닫기.
    ~MainWindow() override {}

    // 지점 파일을 읽어 순찰을 시작한다: 엔진의 move_to 로 정해 둔 지점들을 차례로 걷는다 (길 찾기 없음).
    // 모델은 먼저 열어 둔 상태여야 한다 (이름이 같은 모델만 움직인다).
    void startPatrol(const Options& o) {
        if (o.patrolFile.empty()) return;
        simul::Array<unsigned char> bytes;
        simul::String err;
        if (!simul::ReadAllBytes(o.patrolFile, &bytes, &err) || bytes.empty()) {
            logError(QString::fromUtf8("지점 파일을 읽을 수 없음: %1 %2").arg(simulqt::ToQ(o.patrolFile)).arg(simulqt::ToQ(err)));
            return;
        }
        patrol_ = new simul::sim::Patrol();
        if (!patrol_->Parse(simul::String(reinterpret_cast<const char*>(&bytes[0]), bytes.size()), &err)) {
            logError(simulqt::ToQ(err));
            delete patrol_;
            patrol_ = nullptr;
            return;
        }
        {   // 지점 파일과 같은 폴더의 obstacles.txt (나무, 집 등)가 있으면 그것도 피한다
            const size_t cut = o.patrolFile.find_last_of("/\\");
            const simul::String obsFile = (cut == simul::String::npos ? simul::String() : o.patrolFile.substr(0, cut + 1)) + "obstacles.txt";
            simul::Array<unsigned char> ob;
            simul::String oerr;
            if (simul::ReadAllBytes(obsFile, &ob, &oerr) && !ob.empty()) {
                if (!patrol_->ParseObstacles(simul::String(reinterpret_cast<const char*>(&ob[0]), ob.size()), &oerr))
                    logError(simulqt::ToQ(oerr));
            }
        }
        if (!backend_) backend_ = new ViewerBackend(viewer_);
        engine_ = new simul::sim::Engine(backend_);
        engine_->SetListener(patrol_);
        runner_ = new SimulRunner(engine_, this);
        runner_->Start();
        // 주민을 클릭하면 그 주민 시점으로 본다 (Esc 로 원래 시점으로)
        fpTimer_ = new QTimer(this);
        fpTimer_->setInterval(16);
        fpBack_ = new QPushButton(QString::fromUtf8("◀ 원래 시점으로  (Esc)"), viewer_);   // 주민 시점 중에만 3D 화면 오른쪽 위에 보인다
        fpBack_->setCursor(Qt::PointingHandCursor);
        fpBack_->setStyleSheet("QPushButton { background: rgba(20,24,32,215); color: #ffffff; border: 1px solid rgba(255,255,255,90); border-radius: 6px; padding: 7px 14px; font-weight: bold; }"
                               "QPushButton:hover { background: rgba(60,110,200,230); }");
        fpBack_->adjustSize();
        fpBack_->hide();
        QObject::connect(fpBack_, &QPushButton::clicked, this, [this]() { leaveFirstPerson(); });
        QObject::connect(fpTimer_, &QTimer::timeout, this, [this]() { updateFirstPerson(); });
        QObject::connect(viewer_, &ModelViewer::selectionPicked, this, [this](int link, bool) {
            if (link < 0 || !patrol_) return;
            const int m = viewer_->ModelOfLink(link);
            if (m < 0) return;
            const QString name = viewer_->ModelName(m);
            for (int i = 0; i < patrol_->RouteCount(); i++)
                if (simulqt::ToQ(patrol_->Route(i).name) == name) { enterFirstPerson(m); return; }
        });
        qApp->installEventFilter(this);   // 주민 시점의 WASD
        setupChat(o);
        // A* 시각화 도크 (Ctrl+K): 주민이 서로 비켜 가려고 길을 새로 짤 때마다 그 탐색 과정을 보여 준다
        astarView_ = new AStarView(this);
        astarAuto_ = new QCheckBox(QString::fromUtf8("재생이 끝나면 가장 최근 길 계산으로 바꿔 보기"), this);
        astarAuto_->setChecked(true);
        QWidget* box = new QWidget(this);
        QVBoxLayout* boxLay = new QVBoxLayout(box);
        boxLay->setContentsMargins(0, 0, 0, 0);
        boxLay->addWidget(astarView_, 1);
        boxLay->addWidget(astarAuto_);
        astarDock_ = new QDockWidget(QString::fromUtf8("A* 시각화"), this);
        astarDock_->setWidget(box);
        addDockWidget(Qt::RightDockWidgetArea, astarDock_);
        astarDock_->hide();
        QAction* astarAct = astarDock_->toggleViewAction();
        astarAct->setText(QString::fromUtf8("A* 시각화"));
        astarAct->setShortcut(Qt::CTRL + Qt::Key_K);
        if (fileMenu_) fileMenu_->addAction(astarAct);
        QObject::connect(astarDock_, &QDockWidget::visibilityChanged, this, [this](bool on) {
            if (patrol_) patrol_->SetTracing(on);   // 안 볼 때는 기록을 남기지 않는다
        });
        astarTimer_ = new QTimer(this);
        astarTimer_->setInterval(100);
        QObject::connect(astarTimer_, &QTimer::timeout, this, [this]() {
            if (astarView_->IsPlaying()) astarIdle_.restart();
            if (!patrol_ || patrol_->TraceSerial() == astarSerial_) return;
            // 재생 중이거나 막 끝났을 때(2.5 초)는 기다렸다가 가장 최근 계산으로 바꾼다 (계산이 0.25 초마다 나와서 바로 바꾸면 끝까지 볼 수 없다)
            if (astarView_->HasTrace() && (!astarAuto_->isChecked() || astarView_->IsPlaying() || astarIdle_.elapsed() < 2500)) return;
            astarSerial_ = patrol_->TraceSerial();
            astarView_->SetTrace(patrol_->LastTrace(), QString::fromUtf8("%1 이(가) 비켜 가는 길을 계산").arg(simulqt::ToQ(patrol_->LastTraceName())));
            astarIdle_.restart();
        });
        astarIdle_.start();
        astarTimer_->start();
        log_->Info(QString::fromUtf8("순찰 시작: %1 (길 %2개, 장애물 %3개)").arg(simulqt::ToQ(o.patrolFile)).arg(patrol_->RouteCount()).arg(patrol_->ObstacleCount()));
    }

    // 주민 대화: 마주치면 멈춰서 번갈아 말한다. 대사는 로컬 LLM (Ollama) 에 물어보고, 서버가 없거나 실패하면 정해 둔 대사로 대신한다.
    void setupChat(const Options& o) {
        if (o.noChat || !patrol_ || !engine_ || !runner_) return;
        chat_ = new simul::sim::Chat();
        chat_->SetPatrol(patrol_);
        const size_t cut = o.patrolFile.find_last_of("/\\");
        const simul::String dir = cut == simul::String::npos ? simul::String() : o.patrolFile.substr(0, cut + 1);
        simul::Array<unsigned char> bytes;
        simul::String err;
        if (simul::ReadAllBytes(dir + "personas.txt", &bytes, &err) && !bytes.empty() &&
            !chat_->ParsePersonas(simul::String(reinterpret_cast<const char*>(&bytes[0]), bytes.size()), &err))
            logError(simulqt::ToQ(err));
        chat_->AddMissingPersonas(*patrol_);
        llm_ = new LlmClient(this);
        llm_->SetServer(simulqt::ToQ(o.llmUrl), simulqt::ToQ(o.llmModel));
        useLlm_ = !o.noLlm;
        bubbles_ = new SpeechBubbles(viewer_, this);
        QObject::connect(llm_, &LlmClient::answered, this, [this](int id, const QString& text) { chat_->Deliver(id, simulqt::FromQ(text)); });
        QObject::connect(llm_, &LlmClient::failed, this, [this](int id, const QString& why) {
            chat_->Fail(id);   // 정해 둔 대사로 대신한다
            llmDownUntil_ = QDateTime::currentMSecsSinceEpoch() + 60000;   // 1분 동안은 묻지 않고 바로 정해 둔 대사
            if (!llmWarned_) {
                llmWarned_ = true;
                log_->Warning(QString::fromUtf8("LLM 서버(%1, 모델 %2)에서 답을 못 받아 정해 둔 대사로 말합니다: %3 (Ollama 를 켜고 `ollama pull %2` 를 해 두면 1분 뒤부터 다시 시도합니다)")
                                  .arg(llm_->Url()).arg(llm_->Model()).arg(why));
            }
        });
        QObject::connect(runner_, &SimulRunner::ticked, this, [this](double t) {
            chat_->Tick(*engine_, t);
            simul::sim::ChatRequest r;
            while (chat_->PopRequest(&r)) {
                if (useLlm_ && QDateTime::currentMSecsSinceEpoch() >= llmDownUntil_) llm_->Ask(r.id, simulqt::ToQ(simul::sim::Chat::BuildPrompt(r)));
                else chat_->Fail(r.id);
            }
            const simul::Array<simul::String> lines = chat_->TakeLog();
            for (size_t i = 0; i < lines.size(); i++) log_->Info(simulqt::ToQ(lines[i]));
            npcBubbles_ = chat_->Bubbles();
            updateBubbles();
        });
        log_->Info(QString::fromUtf8("주민 대화: %1 (%2)").arg(o.noLlm ? QString::fromUtf8("정해 둔 대사만") : QString::fromUtf8("LLM %1 / %2").arg(simulqt::ToQ(o.llmUrl)).arg(simulqt::ToQ(o.llmModel)))
                       .arg(QString::fromUtf8("주민 %1명").arg(chat_->PersonaCount())));
    }

    void updateBubbles() {
        if (!bubbles_) return;
        simul::Array<simul::sim::Bubble> all = npcBubbles_;
        for (size_t i = 0; i < webBubbles_.size(); i++) all.push_back(webBubbles_[i]);
        bubbles_->Update(all);
    }

    // 웹 접속: web/server.js 가 이 포트로 붙어서 접속/입력을 보내 주면, 접속자가 엔진의 오브젝트(주민 모델)로 올라온다.
    // 그래서 이 3D 화면에도 접속자가 보이고, 웹에는 (--npc 로 연) 순찰 주민도 같이 보인다.
    void setupGame(const Options& o) {
        if (o.servePort <= 0) return;
        if (o.dataDir.empty()) { logError(QString::fromUtf8("--serve: 마을 데이터 폴더를 찾지 못했습니다 (파일을 직접 열었으면 --serve 는 쓸 수 없습니다)")); return; }
        simul::String err, text;
        simul::Array<unsigned char> b;
        game_ = new simul::sim::Players();
        const simul::String dir = o.dataDir;
        if (simul::ReadAllBytes(dir + "/villagers/obstacles.txt", &b, &err) && !b.empty()) {
            game_->World().ParseObstacles(simul::String(reinterpret_cast<const char*>(&b[0]), b.size()), &err);
        }
        b.clear();
        if (simul::ReadAllBytes(dir + "/villagers/layout.txt", &b, &err) && !b.empty()) {   // 여러 층 건물 (바닥판/계단/벽)
            if (!game_->World().ParseLayout(simul::String(reinterpret_cast<const char*>(&b[0]), b.size()), &err))
                logError(QString::fromUtf8("layout.txt: %1").arg(simulqt::ToQ(err)));
        }
        b.clear();
        if (simul::ReadAllBytes(dir + "/villagers/portals.txt", &b, &err) && !b.empty()) {   // 건물 내부 (B 키로 들어가고 나온다)
            if (!game_->World().ParsePortals(simul::String(reinterpret_cast<const char*>(&b[0]), b.size()), &err))
                logError(QString::fromUtf8("portals.txt: %1").arg(simulqt::ToQ(err)));
        }
        b.clear();
        if (!simul::ReadAllBytes(dir + "/ground.txt", &b, &err) || b.empty() ||
            !game_->World().LoadGround(simul::String(reinterpret_cast<const char*>(&b[0]), b.size()), &err)) {
            logError(QString::fromUtf8("땅 높이 표(ground.txt)를 읽을 수 없습니다: %1  (node tools/make_factory_map.js)").arg(simulqt::ToQ(err)));
            delete game_;
            game_ = nullptr;
            return;
        }
        spawner_.owner = this;
        spawner_.dir = simulqt::ToQ(dir);
        game_->SetSpawner(&spawner_);
        game_->World().SetMaxPlayers(30);
        if (patrol_) {
            game_->SetNext(patrol_);
            simul::Array<simul::String> names;
            for (int i = 0; i < patrol_->RouteCount(); i++) names.push_back(patrol_->Route(i).name);
            game_->SetNpcNames(names);
        }
        if (!engine_) {   // --npc 가 없으면 엔진이 아직 없다: 접속자만 움직이는 엔진을 만든다
            if (!backend_) backend_ = new ViewerBackend(viewer_);
            engine_ = new simul::sim::Engine(backend_);
            runner_ = new SimulRunner(engine_, this);
            engine_->SetListener(game_);
            runner_->Start();
        } else {
            engine_->SetListener(game_);
        }
        link_ = new GameLink(this);
        QString lerr;
        if (!link_->Listen(static_cast<quint16>(o.servePort), &lerr)) {
            logError(QString::fromUtf8("웹 접속 포트 %1 을 열 수 없습니다: %2").arg(o.servePort).arg(lerr));
            return;
        }
        QObject::connect(link_, &GameLink::lineReceived, this, [this](const QString& line) {
            const QByteArray u = line.toUtf8();
            const bool join = line.startsWith(QLatin1String("join "));
            const bool ok = game_->Command(simul::String(u.constData(), static_cast<size_t>(u.size())));
            if (join) log_->Info(QString::fromUtf8("웹 접속: %1 -> %2 (지금 %3명)").arg(line.mid(5)).arg(ok ? QString::fromUtf8("들어옴") : QString::fromUtf8("못 들어옴")).arg(game_->World().Count()));
            else if (line.startsWith(QLatin1String("leave "))) log_->Info(QString::fromUtf8("웹 퇴장: %1 (지금 %2명)").arg(line.mid(6)).arg(game_->World().Count()));
        });
        QObject::connect(link_, &GameLink::connectionsChanged, this, [this](int n) {
            log_->Info(n > 0 ? QString::fromUtf8("웹 게이트웨이 연결됨") : QString::fromUtf8("웹 게이트웨이 연결이 끊김"));
        });
        if (!bubbles_) bubbles_ = new SpeechBubbles(viewer_, this);
        QObject::connect(runner_, &SimulRunner::ticked, this, [this](double t) {
            webBubbles_ = game_->Bubbles();   // 웹에서 한 말은 이 화면에도 말풍선으로 보인다
            updateBubbles();
            if (++gameTick_ % 3 != 0 || link_->Connections() == 0) return;   // 3틱(약 50 ms)마다 모두의 상태를 보낸다
            const simul::String j = game_->StateJson(*engine_, t);
            link_->Broadcast(QByteArray(j.c_str(), static_cast<int>(j.size())));
        });
        log_->Info(QString::fromUtf8("웹 접속 대기: 127.0.0.1:%1 (최대 30명). web/server.js --connect 127.0.0.1:%1").arg(o.servePort));
    }

    // 주민 시점: 눈 높이(발 + 0.78 m)에서 앞쪽(모델 -Y 방향)을 본다. 매 프레임 따라가고, 방향은 부드럽게 돌린다.
    void enterFirstPerson(int model) {
        if (fpModel_ < 0) {   // 처음 들어갈 때만 원래 시점을 기억한다 (다른 주민을 눌러 바꿀 때는 유지)
            fpSavedTarget_ = viewer_->CameraTarget();
            fpSavedDist_ = viewer_->CameraDistance();
            fpSavedYaw_ = viewer_->CameraYaw();
            fpSavedPitch_ = viewer_->CameraPitch();
        }
        if (fpModel_ >= 0 && fpModel_ != model) stopDriving();   // 다른 주민으로 바꿀 때는 전 주민을 다시 걷게 한다
        fpModel_ = model;
        fpYawInit_ = false;
        fpCamYaw_ = fpSavedYaw_;
        startDriving(model);
        updateFirstPerson();
        fpTimer_->start();
        fpBack_->show();
        fpBack_->raise();
        statusBar()->showMessage(QString::fromUtf8("%1 탑뷰 - WASD 이동, Shift 달리기, Q/E 화면 돌리기 (Esc: 돌아가기)").arg(viewer_->ModelName(model)));
    }
    void leaveFirstPerson() {
        if (fpModel_ < 0) return;
        stopDriving();
        fpModel_ = -1;
        fpKeys_.clear();
        fpTimer_->stop();
        fpBack_->hide();
        viewer_->SetCamera(fpSavedTarget_, fpSavedDist_, fpSavedYaw_, fpSavedPitch_);
        statusBar()->clearMessage();
    }
    // WASD 로 직접 움직이기: 주민 시점 동안 그 주민을 Hold 로 세워 두고(순찰/대화에서 빠진다) 이 쪽에서 위치를 정한다.
    void startDriving(int model) {
        fpDrive_ = false;
        if (!patrol_ || !engine_) return;
        const QString name = viewer_->ModelName(model);
        for (int i = 0; i < patrol_->RouteCount(); i++)
            if (simulqt::ToQ(patrol_->Route(i).name) == name) fpRoute_ = patrol_->Route(i).name;
        const dhPose p = viewer_->ModelPlacementPose(model);
        const dhVector f = p.orientation.Rotate(dhVector(0, -1, 0));
        if (!patrol_->Hold(*engine_, fpRoute_, static_cast<double>(p.position.x + f.x), static_cast<double>(p.position.y + f.y))) return;
        fpPos_ = p.position;
        fpBody_ = atan2(static_cast<double>(f.x), -static_cast<double>(f.y));   // 모델 앞쪽 = (sin, -cos)
        fpDrive_ = true;
        fpClock_.start();
    }
    void stopDriving() {
        if (fpDrive_ && patrol_ && engine_) patrol_->Release(*engine_, fpRoute_);
        fpDrive_ = false;
    }
    // 가까운 동그라미(나무/집 등 + 다른 주민)에서 밀어낸다
    void pushOut(double* x, double* y, double cx, double cy, double r) const {
        const double dx = *x - cx, dy = *y - cy, d2 = dx * dx + dy * dy;
        if (d2 >= r * r) return;
        const double d = sqrt(d2);
        if (d < 1e-6) { *x = cx + r; return; }
        *x = cx + dx / d * r;
        *y = cy + dy / d * r;
    }
    void drive() {
        if (!fpDrive_) return;
        double dt = static_cast<double>(fpClock_.restart()) / 1000.0;
        if (dt > 0.1) dt = 0.1;
        const double turn = ((fpKeys_.contains(Qt::Key_Q) || fpKeys_.contains(Qt::Key_Left)) ? 1.0 : 0.0) -
                            ((fpKeys_.contains(Qt::Key_E) || fpKeys_.contains(Qt::Key_Right)) ? 1.0 : 0.0);
        fpCamYaw_ += turn * 1.6 * dt;   // Q/E: 화면 돌리기
        const double fwd = (fpKeys_.contains(Qt::Key_W) ? 1.0 : 0.0) - (fpKeys_.contains(Qt::Key_S) ? 1.0 : 0.0);
        const double side = (fpKeys_.contains(Qt::Key_D) ? 1.0 : 0.0) - (fpKeys_.contains(Qt::Key_A) ? 1.0 : 0.0);
        if (fwd != 0.0 || side != 0.0) {
            const double fx = -cos(fpCamYaw_), fy = -sin(fpCamYaw_);   // 화면 위쪽
            const double rx = -sin(fpCamYaw_), ry = cos(fpCamYaw_);    // 화면 오른쪽
            double vx = fx * fwd + rx * side, vy = fy * fwd + ry * side;
            const double n = sqrt(vx * vx + vy * vy);
            const double want = atan2(vx, -vy);   // 가는 쪽을 보게 (모델 앞쪽 = (sin, -cos))
            double d = want - fpBody_;
            while (d > 3.14159265358979) d -= 6.28318530717958;
            while (d < -3.14159265358979) d += 6.28318530717958;
            fpBody_ += d * (1.0 - exp(-14.0 * dt));
            const double speed = (fpKeys_.contains(Qt::Key_Shift) ? 5.0 : 2.5) * dt;
            double x = static_cast<double>(fpPos_.x) + vx / n * speed, y = static_cast<double>(fpPos_.y) + vy / n * speed;
            for (int it = 0; it < 2; it++) {   // 겹치는 것이 하나 더 있어도 밀려 나오도록 두 번
                for (int i = 0; i < patrol_->ObstacleCount(); i++) {
                    const simul::sim::Circle2& c = patrol_->Obstacle(i);
                    pushOut(&x, &y, c.x, c.y, c.r + 0.4);
                }
                for (int i = 0; i < viewer_->ModelCount(); i++) {   // 다른 주민
                    if (i == fpModel_) continue;
                    bool villager = false;
                    for (int k = 0; k < patrol_->RouteCount(); k++)
                        if (simulqt::ToQ(patrol_->Route(k).name) == viewer_->ModelName(i)) villager = true;
                    if (!villager) continue;
                    const dhPose o = viewer_->ModelPlacementPose(i);
                    pushOut(&x, &y, static_cast<double>(o.position.x), static_cast<double>(o.position.y), 0.8);
                }
            }
            fpPos_.x = static_cast<float>(x);
            fpPos_.y = static_cast<float>(y);
        }
        const simul::sim::Handle h = engine_->Find(fpRoute_);
        if (h.slot >= 0) engine_->SetPose(h, dhPose(fpPos_, dhQuat::FromAxisAngle(static_cast<float>(fpBody_), dhVector(0, 0, 1))));
    }
    bool eventFilter(QObject* obj, QEvent* ev) override {
        if (fpModel_ >= 0 && fpDrive_ && (ev->type() == QEvent::KeyPress || ev->type() == QEvent::KeyRelease || ev->type() == QEvent::ShortcutOverride)) {
            QKeyEvent* k = static_cast<QKeyEvent*>(ev);
            const int key = k->key();
            const bool ours = key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D || key == Qt::Key_Q ||
                              key == Qt::Key_E || key == Qt::Key_Left || key == Qt::Key_Right || key == Qt::Key_Shift;
            if (ours && !k->isAutoRepeat()) {
                if (ev->type() == QEvent::ShortcutOverride) { ev->accept(); return false; }   // 단축키보다 먼저 받는다
                if (ev->type() == QEvent::KeyPress) fpKeys_.insert(key);
                else fpKeys_.remove(key);
                return true;
            }
            if (ours) return true;   // 키 반복은 무시
        }
        return QMainWindow::eventFilter(obj, ev);
    }
    void focusOutEvent(QFocusEvent* e) override { fpKeys_.clear(); QMainWindow::focusOutEvent(e); }

    void updateFirstPerson() {
        if (fpModel_ < 0) return;
        if (fpModel_ >= viewer_->ModelCount()) { leaveFirstPerson(); return; }
        fpBack_->move(viewer_->width() - fpBack_->width() - 14, 14);   // 창 크기가 바뀌어도 오른쪽 위에 둔다
        drive();
        const dhPose pose = fpDrive_ ? dhPose(fpPos_, dhQuat::FromAxisAngle(static_cast<float>(fpBody_), dhVector(0, 0, 1))) : viewer_->ModelPlacementPose(fpModel_);
        const dhVector target(pose.position.x, pose.position.y, static_cast<float>(pose.position.z + 0.8));   // 캐릭터 머리 근처를 위에서 내려다본다
        viewer_->SetCamera(target, 14.0, fpCamYaw_, 1.2);
    }

    // 오른쪽 "오브젝트 / 관절 값" 도크 (Ctrl+J): 장면에서 오브젝트 이름과 관절 값을 바로 읽어 보여 준다
    void setupObjectPanel(const Options& o) {
        if (!backend_) backend_ = new ViewerBackend(viewer_);
        objPanel_ = new ObjectInfoPanel(backend_);
        objDock_ = new QDockWidget(QString::fromUtf8("오브젝트 / 관절 값"), this);
        objDock_->setWidget(objPanel_);
        addDockWidget(Qt::RightDockWidgetArea, objDock_);
        objDock_->hide();
        QAction* t = objDock_->toggleViewAction();
        t->setText(QString::fromUtf8("오브젝트 / 관절 값"));
        t->setShortcut(Qt::CTRL + Qt::Key_J);
        if (fileMenu_) fileMenu_->addAction(t);
        if (o.objPanel) objDock_->show();
    }

    // project.simul 을 열어 모델을 올리고 배치한다
    bool loadProject(const Options* o) {
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
        viewer_->FitView();
        return true;
    }

    bool openFile(const QString& path) {
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
        QAction *beams = nullptr, *graph = nullptr, *place = nullptr, *remove = nullptr, *floor = nullptr;
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
        else if (chosen == floor) {
            viewer_->SetFloorModel(floor->isChecked() ? model : -1);
            log_->Info(floor->isChecked() ? QString::fromUtf8("바닥으로 지정: %1").arg(viewer_->ModelName(model)) : QString::fromUtf8("바닥 지정 해제"));
        }
        else if (chosen == remove) removeModel(model);
    }

    // 모델의 월드 위치와 자세를 입력받는다
    void askPlacement(int model) {
        float v[6];
        viewer_->ModelPlacement6(model, v);
        bool ok = false;
        const QString text = QInputDialog::getText(this, QString::fromUtf8("위치 지정"),
                                                   QString::fromUtf8("%1\nx, y, z (m), roll, pitch, yaw (도). 자세는 생략하면 0\n(Rz(yaw)·Ry(pitch)·Rx(roll))").arg(viewer_->ModelName(model)),
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
    ViewerBackend* backend_ = nullptr;
    simul::sim::Engine* engine_ = nullptr;   // --patrol: 엔진 + 순찰 + 틱 타이머
    simul::sim::Patrol* patrol_ = nullptr;
    SimulRunner* runner_ = nullptr;
    AStarView* astarView_ = nullptr;         // 보기: A* 시각화
    QCheckBox* astarAuto_ = nullptr;
    QDockWidget* astarDock_ = nullptr;
    QTimer* astarTimer_ = nullptr;
    int astarSerial_ = 0;
    QElapsedTimer astarIdle_;                // 마지막으로 재생 중이던 때부터 지난 시간
    simul::sim::Chat* chat_ = nullptr;        // 주민 대화
    LlmClient* llm_ = nullptr;
    SpeechBubbles* bubbles_ = nullptr;
    simul::Array<simul::sim::Bubble> npcBubbles_, webBubbles_;   // 주민 대화와 웹 채팅의 말풍선을 합쳐서 보여 준다
    bool useLlm_ = false;
    bool llmWarned_ = false;
    qint64 llmDownUntil_ = 0;
    QPushButton* fpBack_ = nullptr;          // "원래 시점으로" 버튼
    QTimer* fpTimer_ = nullptr;              // 주민 시점으로 보는 중
    int fpModel_ = -1;
    bool fpYawInit_ = false;
    double fpYaw_ = 0.0;
    bool fpDrive_ = false;                   // WASD 로 움직이는 중 (주민을 Hold 로 잡고 있다)
    simul::String fpRoute_;
    dhVector fpPos_;
    double fpBody_ = 0.0;
    double fpCamYaw_ = 0.0;                  // 탑뷰 카메라가 보는 방향                    // 몸 방향(yaw)
    QSet<int> fpKeys_;
    QElapsedTimer fpClock_;
    dhVector fpSavedTarget_;
    double fpSavedDist_ = 10.0, fpSavedYaw_ = 0.0, fpSavedPitch_ = 0.5;
    // 웹 접속자를 장면의 모델로 올리고 내리는 쪽
    struct CamSave {
        ModelViewer* v;
        dhVector target;
        double dist, yaw, pitch;
        explicit CamSave(ModelViewer* viewer) : v(viewer), target(viewer->CameraTarget()), dist(viewer->CameraDistance()), yaw(viewer->CameraYaw()), pitch(viewer->CameraPitch()) {}
        ~CamSave() { v->SetCamera(target, dist, yaw, pitch); }
    };
    struct WebSpawner : simul::sim::PlayerSpawner {
        MainWindow* owner = nullptr;
        QString dir;
        bool AddModel(int id, simul::String* modelName) override {
            const int n = 1 + ((id > 0 ? id - 1 : 0) % 4);   // 접속 순서대로 주민 모델 4종을 돌려 쓴다
            const int before = owner->viewer_->ModelCount();
            CamSave cam(owner->viewer_);   // 모델을 올리면 화면이 전체 맞춤으로 바뀌므로 보던 시점을 되돌린다
            // 접속자는 무기를 쥘 오른팔을 따로 올리므로, 몸에서 오른팔을 뺀 모델(_p)이 있으면 그것을 쓴다
            QString file = dir + QString("/villagers/villager_%1_p.glb").arg(n);
            if (!QFileInfo(file).isFile()) file = dir + QString("/villagers/villager_%1.glb").arg(n);
            if (!owner->openFile(file) || owner->viewer_->ModelCount() <= before) return false;
            *modelName = owner->viewer_->ModelName(owner->viewer_->ModelCount() - 1).toUtf8().constData();
            return true;
        }
        bool AddPart(const simul::String& file, simul::String* modelName) override {   // 팔/무기 부품
            const int before = owner->viewer_->ModelCount();
            CamSave cam(owner->viewer_);
            if (!owner->openFile(dir + "/villagers/" + QString::fromUtf8(file.c_str())) || owner->viewer_->ModelCount() <= before) return false;
            *modelName = owner->viewer_->ModelName(owner->viewer_->ModelCount() - 1).toUtf8().constData();
            return true;
        }
        void RemoveModel(const simul::String& modelName) override {
            const QString q = QString::fromUtf8(modelName.c_str());
            for (int m = 0; m < owner->viewer_->ModelCount(); m++)
                if (owner->viewer_->ModelName(m) == q) { CamSave cam(owner->viewer_); owner->removeModel(m); return; }
        }
    };
    WebSpawner spawner_;
    simul::sim::Players* game_ = nullptr;
    GameLink* link_ = nullptr;
    int gameTick_ = 0;
    ObjectInfoPanel* objPanel_ = nullptr;
    QDockWidget* objDock_ = nullptr;
    QMenu* fileMenu_ = nullptr;
    QDockWidget* logDock_ = nullptr;
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

// 파일을 안 줬을 때 열 기본 장면: Data/Object 의 마을 지도와, 지점 파일에 이름이 나오는 주민 모델.
// 주민은 엔진이 직접 움직이므로 중력 낙하는 끈다 (끄지 않으면 z=0 평면까지 떨어진다).
bool FindVillage(Options* o) {
    QStringList dirs;
#ifdef SIMUL_DATA_DIR
    dirs << QString::fromUtf8(SIMUL_DATA_DIR);
#endif
    dirs << QCoreApplication::applicationDirPath() + "/Data" << QCoreApplication::applicationDirPath() + "/../Data";
    for (int i = 0; i < dirs.size(); i++) {
        const QDir obj(QDir(dirs[i]).filePath("Object"));
        const QString map = obj.filePath("factory_map.glb"), wp = obj.filePath("villagers/waypoints.txt");
        if (!QFileInfo(map).isFile() || !QFileInfo(wp).isFile()) continue;
        simul::Array<unsigned char> bytes;
        simul::String err;
        simul::sim::Patrol patrol;
        if (!simul::ReadAllBytes(wp.toLocal8Bit().constData(), &bytes, &err) || bytes.empty() ||
            !patrol.Parse(simul::String(reinterpret_cast<const char*>(&bytes[0]), bytes.size()), &err)) {
            fprintf(stderr, "경고: 마을 지점 파일을 읽을 수 없습니다 (%s): %s\n", wp.toLocal8Bit().constData(), err.c_str());
            continue;
        }
        o->file = map.toLocal8Bit().constData();
        o->dataDir = obj.absolutePath().toLocal8Bit().constData();
        if (o->npc) {   // --npc 일 때만 주민을 같이 연다
            for (int r = 0; r < patrol.RouteCount(); r++) {
                const QString m = obj.filePath(QString("villagers/") + simulqt::ToQ(patrol.Route(r).name) + ".glb");
                if (QFileInfo(m).isFile()) o->moreFiles.push_back(m.toLocal8Bit().constData());
            }
            o->patrolFile = wp.toLocal8Bit().constData();
        }
        o->noFall = true;
        return true;
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    InitConsole();
    Options o;
    const int pr = ParseArgs(argc, argv, &o);
    if (pr == 1) return 0;
    if (pr == 2) return 2;
    if (!o.makeMarker.empty()) return MakeMarkerFiles(o);
    if (o.info) return PrintInfo(o);
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    const bool village = o.projectFile.empty() && o.file.empty() && !o.empty && FindVillage(&o);   // 아무것도 안 줬으면 마을을 연다
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
    if (village && !o.viewSet) w.viewer()->SetCamera(dhVector(0.0f, 0.0f, 3.0f), 80.0, -1.0471976, 0.6);   // 마을 전체가 보이고 주민도 알아볼 거리
    w.startHidden(o);
    w.setupObjectPanel(o);
    w.startPatrol(o);
    w.setupGame(o);
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
