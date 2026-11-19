#ifndef CORECQT_MODELVIEWER_H
#define CORECQT_MODELVIEWER_H

#include <QMatrix4x4>
#include <QElapsedTimer>
#include <QHash>
#include <QLabel>
#include <QImage>
#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QSharedPointer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPoint>
#include <QElapsedTimer>
#include <QColor>
#include <QString>
#include <QTimer>
#include <QVector3D>
#include <QVector4D>
#include "CBase/Util.h"
#include "CBase/Array.h"

#include <QStringList>

#include "3d/dhMat.h"
#include "3d/dhPose.h"
#include "CCollision/RobotCollision.h"
#include "CMesh/Mesh.h"
#include "CCollision/Bvh.h"
#include "CSensor/Lidar.h"
#include "CURDF/Urdf.h"
#include "CURDF/Primitives.h"
#include "CURDF/World.h"

// 3D 모델 / URDF 로봇 뷰어 (QOpenGLWidget, OpenGL 2.1 호환, Qt 5.12).
//   마우스: 왼쪽 드래그 = 회전, 오른쪽/가운데 드래그 = 이동, 휠 = 확대/축소, 더블클릭 = 화면에 맞춤
//   좌표계: Z-up (URDF 규약). GLB(Y-up)는 LoadFile 에서 Z-up 으로 맞춘다.
class ModelViewer : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit ModelViewer(QWidget* parent = nullptr);
    ~ModelViewer() override;

    // 확장자로 판별: .urdf 는 로봇(visual 메시 포함), 나머지(.glb .gltf .stl .obj .dae)는 정적 모델.
    // 실패하면 false 와 error. 성공 시 카메라를 모델에 맞춘다.
    // 모델을 한 장면에 여러 개 올린다. 파일을 열면 이전 것을 지우지 않고 추가한다 (URDF, 또는 STL/OBJ/DAE/glTF 한 개).
    // 둘 이상이면 가상 "world" 링크 아래에 모델마다 고정 관절로 매달리고, 이름이 겹치는 링크/관절에는 "<모델>/" 이 붙는다.
    bool AddFile(const QString& path, QString* error = nullptr);
    // 파일 없이 기본 도형을 장면에 모델로 추가한다 (색, 질량 포함). 돌려주는 값은 모델 번호이고 실패하면 -1.
    // pose 는 x, y, z (m), roll, pitch, yaw (도) 이고 null 이면 원점이다. 중력 낙하가 켜져 있으면 바닥으로 떨어진다.
    int AddBox(const QString& name, double sx, double sy, double sz, const float* pose6 = nullptr, const QColor& color = QColor(150, 165, 190), double mass = 1.0, QString* error = nullptr);
    int AddCylinder(const QString& name, double radius, double length, const float* pose6 = nullptr, const QColor& color = QColor(150, 165, 190), double mass = 1.0, QString* error = nullptr);
    int AddSphere(const QString& name, double radius, const float* pose6 = nullptr, const QColor& color = QColor(150, 165, 190), double mass = 1.0, QString* error = nullptr);
    bool LoadFile(const QString& path, QString* error = nullptr);  // 모두 지우고 이 파일 하나만 올린다
    void CloseAll();
    void RemoveModel(int index);
    // 지금 장면(모델 여러 개와 각자의 위치/회전)을 URDF 파일 하나로 저장한다. 모델이 여럿이면 가상 링크 world 아래에
    // 모델마다 <이름>_attach 고정 관절로 매달린다. 현재 관절 값은 파일에 담기지 않는다 (URDF 에는 자세가 없다).
    bool SaveUrdf(const QString& path, QString* error = nullptr) const;
    // 폴더로 저장: dir/<폴더이름>.urdf + dir/obj/ (GLB 는 바이너리 STL 로 바꿔 넣는다). warnings 에는 STL 로 옮기지 못한 색/텍스처 같은 것이 들어간다.
    bool SaveUrdfFolder(const QString& dir, QString* error = nullptr, QStringList* warnings = nullptr) const;
    int ModelCount() const { return static_cast<int>(world_.size()); }
    QString ModelName(int index) const;
    int ModelOfLink(int link) const { return world_.EntryOfLink(robot_, link); }  // 링크가 속한 모델 (world 링크는 -1)
    int ModelOfRoot(int link) const { return world_.EntryOfRoot(robot_, link); }  // 링크가 어떤 모델의 루트이면 그 모델 번호, 아니면 -1
    // 모델의 월드 위치(m)와 z축 회전(도). 모델이 하나면 오브젝트 위치로 움직인다 (회전은 둘 이상일 때만).
    void SetModelPlacement(int index, float x, float y, float z, float yawDeg);
    void ModelPlacement(int index, float* x, float* y, float* z, float* yawDeg) const;
    // 위치 x,y,z (m) 와 자세 roll,pitch,yaw (도, 고정축 z-y-x 순: R = Rz(yaw) Ry(pitch) Rx(roll)). 회전이 있으면 모델 하나도 world 아래에 매단다.
    void SetModelPlacement6(int index, const float v[6]);
    void ModelPlacement6(int index, float v[6]) const;
    void SetModelPose(int index, const dhMat& pose);
    dhMat ModelPose(int index) const;
    // 위치(m) + 단위 쿼터니언으로 주고받는 같은 함수 (엔진의 표준 자세 형식. degree 는 사람이 보는 화면에서만 쓴다)
    void SetModelPlacement(int index, const dhPose& pose) { SetModelPose(index, pose.Normalized().ToMat()); }
    dhPose ModelPlacementPose(int index) const { return dhPose::FromMat(ModelPose(index)); }
    // 모델 파일의 길이 단위를 맞춘다: 메시를 s 배로 한다 (mm 로 만든 파일은 0.001, cm 는 0.01, inch 는 0.0254). 1.0 이 원래 크기. 기본 도형은 바뀌지 않는다.
    bool SetModelUnitScale(int index, double s);
    double ModelUnitScale(int index) const;
    // 모델 전체의 크기 배율: 길이, 관절 위치, 도형, 메시가 모두 s 배 (1.0 이 원래 크기, 0.1 ~ 10 같은 값). massScales 이면 질량이 s^3, 관성이 s^5 으로 따라가
    // 작게 만들면 가벼워지고 크게 만들면 무거워진다 (false 면 질량은 그대로). 원래 크기 기준의 절대 배율이라 여러 번 불러도 겹쳐지지 않는다. 기본 도형도 된다.
    bool SetModelScale(int index, double s, bool massScales = true);
    double ModelScale(int index) const;

    // ---- 카메라 (m, radian, 쿼터니언). 마우스로 돌리는 중심 궤도 카메라: target 을 distance 만큼 떨어져서 본다.
    // yaw 는 target 둘레 z 축 회전, pitch 는 올려다보는 각 (+ 이면 카메라가 위에 있다). 둘 다 radian, pitch 는 +-89 도로 제한한다.
    void SetCamera(const dhVector& target, double distance, double yaw, double pitch);
    dhVector CameraTarget() const { return dhVector(target_.x(), target_.y(), target_.z()); }
    double CameraDistance() const { return static_cast<double>(dist_); }
    double CameraYaw() const { return static_cast<double>(yaw_); }
    double CameraPitch() const { return static_cast<double>(pitch_); }
    // 카메라 자신의 자세: 위치는 눈의 월드 좌표(m), 방향은 x 가 앞(보는 쪽), z 가 위인 단위 쿼터니언 (ROS 와 같은 규약)
    dhPose CameraPose() const;
    // 화면 왼쪽 아래에 "격자 한 칸 = 10 cm, 카메라까지 1.2 m" 같은 실제 길이 기준을 보인다 (모델 크기에 맞춰 화면이 자동으로 맞춰지므로 크기 감각을 잡아 준다)
    void SetScaleInfoVisible(bool on);
    bool ScaleInfoVisible() const { return scaleInfoVisible_; }
    double GridCell() const { return static_cast<double>(gridCell_); }   // 격자 한 칸 (m)
    // 프레임 정보: 화면 왼쪽 아래 표시에 "60 FPS, 그리기 3.2 ms" 를 덧붙인다 (프레임이 떨어지는지, 그리기 때문인지 다른 일 때문인지 가늠하는 용도)
    void SetFrameStatsVisible(bool on);
    bool FrameStatsVisible() const { return frameStatsVisible_; }
    double FramesPerSecond() const { return fps_; }       // 최근 약 0.5 초 동안 평균 (그린 프레임 수 / 시간)
    double PaintMilliseconds() const { return paintMs_; }  // 그리기 한 번에 걸린 평균 시간 (ms, GL 명령을 보내는 데 걸린 시간)
    bool SelectionScreenPos(QPoint* out) const;   // 선택한 것의 원점이 3D 화면 어디에 보이는가 (화면 안이 아니어도 값은 나온다)
    bool CanRotateSelection() const;
    bool worldAxes() const { return !ringLocal_ && CanRotateSelection(); }  // 화살표와 링을 월드 축에 둔다
    void SetRingLocal(bool local) { ringLocal_ = local; update(); }   // 회전 링을 월드 축(기본)/오브젝트 축에 붙인다
    bool RingLocal() const { return ringLocal_; }  // 선택한 링크가 모델의 루트 링크라서 회전 링이 나오는가

    void SetScene(const simul::Scene& scene);  // 정적 모델 (이미 Z-up 이어야 한다)
    void SetRobot(const simul::urdf::Robot& robot, const simul::urdf::RobotMeshes& meshes);
    void Clear();

    bool HasRobot() const { return hasRobot_; }
    const simul::urdf::Robot& Robot() const { return robot_; }
    void SetJointValues(const simul::urdf::JointValues& q);  // 관절 값(rad, m)으로 링크 위치 갱신
    const simul::urdf::JointValues& CurrentJointValues() const { return q_; }
    simul::urdf::LoadOptions& LoadOptions() { return options_; }  // package:// 경로 등

    void FitView();                    // 모델 전체가 보이도록 카메라를 맞춘다
    void SetShowGrid(bool show);

    // 자기충돌 검사 (로봇 링크 사이). 2단계로 검사한다: 링크 AABB 로 후보를 거르고, 후보 링크의 도형끼리 OBB 로 다시 확인.
    // <collision> 이 없는 링크는 <visual> 로 대신한다. 관절 값이 바뀔 때마다 다시 계산한다.
    void SetShowCollisionBoxes(bool show);  // 링크별 AABB (노랑, 후보이면 주황)
    bool ShowCollisionBoxes() const { return showBoxes_; }
    void SetShowObbs(bool show);            // 도형별 OBB (하늘색, 충돌하면 빨강)
    bool ShowObbs() const { return showObbs_; }
    const simul::Array<simul::CollisionPair>& SelfCollisions() const { return report_.collisions; }     // OBB 까지 겹친 링크 쌍
    const simul::Array<simul::CollisionPair>& AabbCandidates() const { return report_.aabbCandidates; }  // AABB 만 겹친 후보
    simul::Array<int> CollidingLinks() const;    // 충돌한 링크 인덱스 (중복 없음, 오름차순)
    QStringList CollisionDescriptions() const;  // "링크A <-> 링크B"

    // 링크 강조: 선택한 링크의 모양을 밝게 칠한다. -1 이면 해제.
    void SetSelectedLink(int link);
    void SetMouseEditEnabled(bool on) { mouseEdit_ = on; update(); }   // 끄면 화살표/링/IK 끌기 없이 보기와 선택만
    bool MouseEditEnabled() const { return mouseEdit_; }
    int SelectedLink() const { return selectedLink_; }

    // 선택과 좌표계: 화면에서 물체를 왼쪽 클릭하면 그 링크(정적 모델이면 모델 전체)가 선택되고, 선택한 것의 좌표계가
    // 화면에 나온다. 빈 곳을 클릭하면 선택이 풀린다. 좌표계 모양은 LoadGizmo 로 읽은 OBJ (가운데 구 + 화살표 3개,
    // 화살표 끝까지 길이 1)이며, 읽지 못했거나 읽기 전에는 선 3개로 대신 그린다.
    bool LoadGizmo(const QString& objPath, QString* error = nullptr);
    bool HasGizmo() const { return !gizmo_.empty(); }
    void SetShowSelectionFrame(bool show);
    bool ShowSelectionFrame() const { return showFrame_; }
    void SelectModel(bool selected);            // 정적 모델 전체를 선택 (SetSelectedLink 와 함께 쓰면 링크 선택은 풀린다)
    bool ModelSelected() const { return selectedModel_; }
    bool HasSelection() const { return selectedLink_ >= 0 || selectedModel_; }
    bool SelectedFrame(dhMat* out) const;       // 선택한 것의 월드 기준 좌표계 (링크면 링크 원점/방향, 정적 모델이면 단위행렬)
    // 화면 좌표 pos 에 보이는 물체: 링크 번호를 돌려주고, 정적 모델이면 *model = true 와 -1, 아무것도 없으면 -1 과 false.
    int PickAt(const QPoint& pos, bool* model = nullptr) const;

    // 이동: 선택한 것의 좌표계 화살표를 끌면 그 축 방향으로 움직인다. 움직일 수 있는 것은 정적 모델(전체)과 로봇의 루트 링크
    // (로봇 전체가 AMR 위치처럼 같이 움직인다). 팔 중간 링크는 관절에 묶여 있어서 위치를 직접 바꾸지 못하고, 아래 CanIkSelection 처럼 IK 로 관절 값을 바꿔서 움직인다.
    // 위치는 월드 기준 m. 관절 값, 충돌 검사, 중력 계산은 로봇 자신의 좌표계(루트 기준)에서 하므로 이동의 영향을 받지 않는다.
    bool CanMoveSelection() const;
    // 팔 중간 링크(루트가 아닌 로봇 링크)를 선택하고 좌표계 화살표를 끌면 역기구학(CKinematics, 위치만 맞춤)으로 그 링크의 원점이
    // 화살표 방향으로 따라오도록 관절 값이 바뀐다. 닿지 못하면 가장 가까운 자세에서 멈춘다 (관절 한계 준수).
    // 관절 값이 바뀔 때마다 jointValuesDragged 가 나가므로 슬라이더 등을 그 값으로 갱신하면 된다.
    bool CanIkSelection() const;

    // 2D 라이다: 로봇 링크 link 에 달고 (mount = 그 링크 좌표계에서 센서까지의 변환, 센서의 +x 가 앞, +z 가 스캔 평면의 법선),
    // 그 링크 자신을 뺀 나머지 메시(로봇의 다른 링크, 정적 모델)를 스캔한다. 매 프레임 다시 스캔해서 맞은 점(빨강)과 광선(옅은 청록)을
    // 그린다. 정적 모델만 열려 있으면 link = -1 (월드 기준)로 달 수 있다. ClearLidar 로 끈다.
    void SetLidar(int link, const simul::sensor::LidarSettings& settings, const dhMat& mount = dhMat());
    void ClearLidar();
    bool LidarEnabled() const { return lidarOn_; }  // 센서가 달려 있는가 (스캔은 ScanLidar 로 언제든 할 수 있다)
    // 3D 화면에 광선(청록 선)과 맞은 점(빨간 십자)을 그릴지. SetLidar 직후는 true. 끄면 그리는 비용도 들지 않는다.
    // 조명 방식: 0 = Lambert(무광), 1 = Blinn-Phong(하이라이트), 2 = PBR(물리 기반, 기본). 실행 중에 바꿀 수 있다.
    enum Shading { kLambert = 0, kBlinnPhong = 1, kPbr = 2 };
    void SetShading(int mode) { shading_ = (mode < 0 || mode > 2) ? 2 : mode; update(); }
    int GetShading() const { return shading_; }
    void SetLidarDrawn(bool on) { lidarDraw_ = on; update(); }
    bool LidarDrawn() const { return lidarDraw_; }
    dhVector LidarOrigin() const { return lidarOrigin_; }  // 마지막 스캔의 센서 원점/방향각 (ScanLidar 뒤에 유효)
    float LidarHeading() const { return lidarHeading_; }
    const simul::sensor::LidarScan& ScanLidar();  // 지금 자세로 한 번 스캔한 결과 (그리기도 같은 결과를 쓴다)
    void SetObjectPosition(float x, float y, float z);
    dhVector ObjectPosition() const;
    void ResetObjectPosition() { SetObjectPosition(0, 0, 0); }

    // 낙하: 켜져 있으면(기본) 물체(정적 모델 전체 또는 로봇 전체)가 바닥(z = 0 인 격자 평면)보다 높이 있을 때 중력으로 떨어진다.
    // 이동으로 들어 올린 물체를 놓으면 떨어지고, 바닥에 닿으면 조금 튕기다 멈춘다. 가장 낮은 점은 보이는 모양(visual)의 꼭짓점으로 잡는다.
    // 중력은 수직(z) 성분만 쓰고(위로 향하거나 0 이면 떨어지지 않는다), 회전과 수평 이동은 없다. 이미 바닥 아래에 있는 물체는 건드리지 않는다.
    void SetFallEnabled(bool enabled);
    bool FallEnabled() const { return fallEnabled_; }
    void SetGravity(double gx, double gy, double gz);  // 월드 기준 중력 가속도 (m/s^2). 낙하에는 gz 만 쓴다
    bool StepFall(double dt);                          // dt 초만큼 낙하를 진행한다. 아직 움직이는 중이면 true (타이머가 부르고, 확인용으로 직접 불러도 된다)
    double SimulateFall(double maxSeconds);            // 멈출 때까지 (또는 maxSeconds 까지) 1/240 초씩 진행하고 걸린 시간(초)을 돌려준다
    float ObjectBottom() const;                        // 오브젝트의 가장 낮은 점의 월드 높이 (m)

    // 바닥으로 지정한 모델: 둘 이상 올렸을 때, 다른 모델들이 z=0 평면 대신 이 모델의 윗면 위로 떨어져 멈춘다.
    // 물체의 꼭짓점마다 바로 아래에 있는 바닥 모델의 윗면(위를 향한 삼각형) 높이를 찾아, 어느 한 점이라도 닿으면 멈춘다.
    // 평면이든 곡면이든 같은 방식이다. 바닥 모델 바깥으로 나간 점은 z=0 평면까지 내려간다.
    // 미끄러짐, 회전, 마찰은 없다 (닿은 자리에서 멈춘다). 바닥 모델 자신은 지금처럼 z=0 평면에 놓인다. -1 이면 지정 해제.
    void SetFloorModel(int model);
    int FloorModel() const { return floorModel_; }
    bool SetFloorByLink(const QString& linkName, QString* error = nullptr);   // 링크가 속한 모델을 바닥으로 지정한다
    // 바닥 모델의 윗면 높이 (x, y 는 월드 m). 그 자리에 윗면이 없으면 false.
    bool FloorHeightAt(double x, double y, double* z) const;

    // 정적 모델(.glb 등)을 열었으면 그 장면
    bool HasScene() const { return hasScene_; }
    const simul::Scene& StaticScene() const { return scene_; }

    QString Summary() const;           // "삼각형 N개, 크기 ..." 같은 한 줄 설명
    const QStringList& Warnings() const { return warnings_; }

signals:
    void modelChanged();
    void modelDragFinished(int model, bool rotated);   // 화살표/링 끌기가 끝났다 (model 은 모델 번호)
    void structureChanged();  // 링크 구성이 바뀌었다 (모델 하나를 처음 돌리면 world 가 생긴다): 트리/패널을 다시 채운다
    void collisionsChanged();  // 겹치는 링크 쌍이 달라졌을 때
    void jointValuesDragged(const simul::urdf::JointValues& q);  // 링크를 끌어서 (IK 로) 관절 값이 바뀌었을 때
    void lidarScanned(const simul::sensor::LidarScan& scan, const dhVector& origin, float heading);  // 그릴 때마다 (센서 +x 의 월드 xy 방향각 rad)
    void objectMoved();  // 좌표계 화살표를 끌어서 위치가 바뀌었을 때
    void selectionChanged(int link);   // SetSelectedLink 로 선택이 바뀌었을 때 (-1 이면 해제)
    void robotChanged();               // 로봇(모델 구성)이 새로 올라왔다
    void jointValuesChanged();         // SetJointValues 로 관절 값이 바뀌었다
    // 화면에서 오른쪽 버튼을 끌지 않고 눌렀다 뗐다. (x, y) 는 그 자리의 바닥(z=0) 위 월드 좌표 m. SimulView 는 이걸로 "추가" 메뉴를 띄운다.
    void viewContextMenuRequested(const QPoint& globalPos, double x, double y);
    void selectionPicked(int link, bool model);  // 마우스 클릭으로 선택이 바뀌었을 때 (link -1, model false 이면 해제)

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    void mouseDoubleClickEvent(QMouseEvent* e) override;

private:
    struct Item {                       // primitive 하나
        simul::Array<float> data;        // 위치(3) + 법선(3) 교차 배열
        simul::Array<uint32_t> indices;
        dhMat base;                     // 링크(또는 월드)에서 이 primitive 까지의 변환
        int link;                       // 로봇 링크 인덱스, 정적 모델이면 -1
        bool bvhOk;                     // 아래 BVH 를 만들었는가 (라이다 스캔 가속용, 처음 스캔할 때 만든다. 메시는 바뀌지 않는다)
        simul::TriangleBvh bvh;
        float color[4];
        float metallic, roughness;      // PBR 재질 (glTF 에서 읽거나 기본값)
        simul::Array<float> extra;      // 정점마다 uv(2) + 색 rgba(4). 텍스처나 정점 색이 있을 때만 채운다
        QImage image;                   // 기본 색 텍스처 (디코딩한 것, 비어 있으면 없음). 같은 이미지를 쓰는 primitive 끼리 같은 데이터를 나눠 쓴다
        QString imageKey;               // 같은 이미지인지 알아보는 이름 (파일 경로 또는 내용의 해시)
        bool hasVertexColor;
        int alphaMode;                  // 재질의 알파 모드 (0 불투명, 1 마스크, 2 블렌드)
        float alphaCutoff;
        QOpenGLBuffer vbo;
        QOpenGLBuffer ibo;
        QOpenGLBuffer evbo;             // extra 를 올린 버퍼
        QSharedPointer<QOpenGLTexture> tex;
        Item() : link(-1), bvhOk(false), hasVertexColor(false), alphaMode(0), alphaCutoff(0.5f), vbo(QOpenGLBuffer::VertexBuffer), ibo(QOpenGLBuffer::IndexBuffer), evbo(QOpenGLBuffer::VertexBuffer) {
            color[0] = color[1] = color[2] = 0.75f;
            color[3] = 1.0f;
            metallic = 0.0f;
            roughness = 0.45f;
        }
    };

    typedef simul::Array<simul::UniquePtr<Item> > ItemList;
    void addScene(const simul::Scene& scene, int link, const dhMat& local, const float* colorOverride, ItemList* dst = nullptr);
    void updateBounds();
    void rebuildGrid();
    void uploadBuffers();
    static void uploadItems(ItemList& list);
    static void releaseItems(ItemList& list);
    void releaseBuffers();
    QMatrix4x4 projMatrix() const;
    bool mouseRay(const QPoint& pos, QVector3D* origin, QVector3D* dir) const;  // 화면 좌표의 월드 광선 (dir 은 길이 임의)
    bool selectionGizmo(QVector3D* origin, QVector3D axis[3], float* scale) const;  // 좌표계의 월드 원점, 축 방향(단위), 화면상 크기
    int hitGizmoAxis(const QPoint& pos, float* u) const;  // 마우스 아래의 화살표 (0 x, 1 y, 2 z, 없으면 -1) 와 그 축 위의 위치
    dhMat itemWorld(const Item& it) const;
    float objectMinZ() const;  // 이동량을 뺀 오브젝트 좌표계에서 가장 낮은 점의 높이 (캐시)
    void drawLidar();
    void drawSelectionFrame(const QMatrix4x4& proj, const QMatrix4x4& view);
    void recomputeCollisions();
    void drawBoxEdges(const dhVector c[8], const QVector4D& color);
    QMatrix4x4 viewMatrix() const;

    simul::Array<simul::UniquePtr<Item> > items_;
    QHash<QString, QImage> imageCache_;   // 디코딩한 텍스처 (장면을 다시 합칠 때마다 큰 이미지를 다시 풀지 않게). 모두 닫으면 비운다
    bool buffersDirty_;
    bool hasRobot_;
    simul::urdf::Robot robot_;
    simul::urdf::LoadOptions options_;
    simul::Array<dhMat> linkT_;
    simul::urdf::JointValues q_;
    simul::RobotCollisionModel colModel_;
    simul::SelfCollisionReport report_;
    int selectedLink_;
    bool selectedModel_;
    bool showFrame_;
    ItemList gizmo_;        // 좌표계 모양 (LoadGizmo)
    bool gizmoDirty_;
    QPoint pressPos_;
    bool dragged_;
    dhVector objectPos_;   // 이동량 (월드, m)
    bool lidarOn_;
    bool lidarDraw_;
    simul::urdf::World world_;      // 올려 둔 모델들 (합친 결과가 robot_)
    bool rebuildWorld(QString* error, bool keepView = false);
    bool commitAdd(bool hadOne, QString* error);                 // World 에 모델을 추가한 뒤 합치기 (실패하면 추가를 취소)
    int addPrimitive(const simul::urdf::PrimitiveSpec& spec, const QString& name, const float* pose6, QString* error);
    bool ensureComposite();                 // 모델 하나를 world 아래로 옮긴다 (이동량은 attach 관절로)
    int hitRing(const QPoint& pos, float* angle) const;   // 마우스 아래의 회전 링 (0 x, 1 y, 2 z, 없으면 -1)
    void drawRings(const QMatrix4x4& proj, const QMatrix4x4& view);
    bool ringBasis(int axis, QVector3D* origin, QVector3D* a, QVector3D* u, QVector3D* v, float* scale) const;
    int shading_;
    int lidarLink_;
    dhMat lidarMount_;
    simul::sensor::Lidar lidar_;
    simul::sensor::LidarScan lidarScan_;
    dhVector lidarOrigin_;  // 마지막 스캔의 센서 원점 (월드)
    float lidarHeading_;    // 마지막 스캔에서 센서 +x 가 월드 xy 평면에서 향한 각 (rad)
    int dragAxis_;         // 좌표계 화살표를 끄는 중이면 0..2
    bool dragIk_;          // 그 끌기가 IK(관절 값 변경)인가, 아니면 물체 이동인가
    QVector3D dragIkOrigin_;  // IK 끌기를 시작한 순간의 좌표계 원점/축 (월드)
    QVector3D dragIkAxes_[3];
    float dragU0_;
    dhVector dragStart_;
    bool mouseEdit_;          // 화살표/링/IK 끌기를 쓸 수 있는가
    int dragRing_;            // 끌고 있는 회전 링 (0 x, 1 y, 2 z, 아니면 -1)
    QPointF dragRingDir_;     // 링 끌기: 잡은 점에서 링 접선 방향의 화면상 단위 벡터
    float dragRingPx_;        // 링 끌기: 1 rad 돌리는 데 필요한 화면 픽셀
    int hoverRing_;           // 마우스가 올라가 있는 링 (밝게 보인다)
    bool ringLocal_;          // 링 기준: false 월드 축, true 오브젝트 축
    dhMat dragPose_;          // 끌기 시작 때의 모델 자세
    int dragModel_;           // 둘 이상 올렸을 때 끌고 있는 모델 번호 (아니면 -1)
    QTimer* fallTimer_;
    QElapsedTimer fallClock_;
    bool fallEnabled_;
    double gravityZ_;
    double fallVel_;      // 위쪽이 + (m/s)
    mutable float minZ_;
    mutable bool minZValid_;
    mutable simul::Array<float> modelMinZ_;   // world 아래에 올린 모델마다 가장 낮은 점의 월드 높이
    mutable bool modelMinZValid_;
    simul::Array<double> modelFallVel_;       // 모델마다 낙하 속도 (위쪽이 +)
    float modelBottom(int model) const;
    float modelClearance(int model) const;    // 모델이 바닥(지정한 바닥 모델 또는 z=0)까지 더 내려갈 수 있는 거리 (m)
    void buildFloor() const;
    int floorModel_;
    mutable bool floorValid_;
    mutable simul::Array<float> floorTris_;        // 바닥 모델의 위를 향한 삼각형 (월드 좌표, 9개씩)
    mutable simul::Array<simul::Array<int> > floorGrid_;   // xy 격자 칸마다 삼각형 번호
    mutable float floorMinX_, floorMinY_, floorCell_;
    mutable int floorNx_, floorNy_;
    mutable simul::Array<float> modelClear_;       // 모델마다 바닥까지 더 내려갈 수 있는 거리
    mutable simul::Array<bool> minZOk_, clearOk_;  // 모델마다 위 값이 지금 자세에서 구한 것인가
    mutable simul::Array<simul::Array<float> > modelSig_;   // 모델마다 마지막으로 계산했을 때의 링크 변환들
    mutable bool modelSigDirty_;                   // 자세나 관절이 바뀌었을 수 있다 (모델별로 다시 확인해야 한다)
    void modelSignature(size_t model, simul::Array<float>* sig) const;
    void syncModelCaches() const;
    bool stepFallModels(double dt);           // 모델 하나하나를 바닥(z=0)까지 떨어뜨린다
    simul::Scene scene_;
    bool hasScene_;
    QStringList warnings_;
    size_t triangleCount_;

    QVector3D center_;   // 모델 경계 상자의 중심/반지름 (월드)
    float radius_;
    QVector3D target_;   // 카메라가 보는 점
    float dist_, yaw_, pitch_;      // 카메라 거리 m, 방위각/고도각 radian
    QLabel* scaleInfo_;
    bool scaleInfoVisible_;
    bool frameStatsVisible_;
    double fps_, paintMs_;
    int statFrames_;
    double statPaintSum_;
    QElapsedTimer statClock_;
    void refreshScaleInfo();
    void paintScene();
    QPoint lastPos_;

    bool showBoxes_;
    bool showObbs_;
    bool showGrid_;
    simul::Array<float> gridData_;   // 격자 선분의 위치(3) 배열
    float extent_[3];               // 경계 상자의 크기 (x, y, z)
    float gridCell_;                // 격자 한 칸의 길이

    simul::UniquePtr<QOpenGLShaderProgram> prog_;
};

#endif  // CORECQT_MODELVIEWER_H
