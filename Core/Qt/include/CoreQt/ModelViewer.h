#ifndef CORECQT_MODELVIEWER_H
#define CORECQT_MODELVIEWER_H

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPoint>
#include <QString>
#include <QVector3D>
#include <QVector4D>
#include <memory>
#include <vector>

#include <QStringList>

#include "3d/dhMat.h"
#include "CCollision/RobotCollision.h"
#include "CMesh/Mesh.h"
#include "CURDF/Urdf.h"

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
    bool LoadFile(const QString& path, QString* error = nullptr);

    void SetScene(const amr::Scene& scene);  // 정적 모델 (이미 Z-up 이어야 한다)
    void SetRobot(const amr::urdf::Robot& robot, const amr::urdf::RobotMeshes& meshes);
    void Clear();

    bool HasRobot() const { return hasRobot_; }
    const amr::urdf::Robot& Robot() const { return robot_; }
    void SetJointValues(const amr::urdf::JointValues& q);  // 관절 값(rad, m)으로 링크 위치 갱신
    amr::urdf::LoadOptions& LoadOptions() { return options_; }  // package:// 경로 등

    void FitView();                    // 모델 전체가 보이도록 카메라를 맞춘다
    void SetShowGrid(bool show);

    // 자기충돌 검사 (로봇 링크 사이). 2단계로 검사한다: 링크 AABB 로 후보를 거르고, 후보 링크의 도형끼리 OBB 로 다시 확인.
    // <collision> 이 없는 링크는 <visual> 로 대신한다. 관절 값이 바뀔 때마다 다시 계산한다.
    void SetShowCollisionBoxes(bool show);  // 링크별 AABB (노랑, 후보이면 주황)
    bool ShowCollisionBoxes() const { return showBoxes_; }
    void SetShowObbs(bool show);            // 도형별 OBB (하늘색, 충돌하면 빨강)
    bool ShowObbs() const { return showObbs_; }
    const std::vector<amr::CollisionPair>& SelfCollisions() const { return report_.collisions; }     // OBB 까지 겹친 링크 쌍
    const std::vector<amr::CollisionPair>& AabbCandidates() const { return report_.aabbCandidates; }  // AABB 만 겹친 후보
    std::vector<int> CollidingLinks() const;    // 충돌한 링크 인덱스 (중복 없음, 오름차순)
    QStringList CollisionDescriptions() const;  // "링크A <-> 링크B"

    // 링크 강조: 선택한 링크의 모양을 밝게 칠한다. -1 이면 해제.
    void SetSelectedLink(int link);
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
    // (로봇 전체가 AMR 위치처럼 같이 움직인다). 팔 중간 링크는 관절에 묶여 있어서 관절 값으로만 움직인다.
    // 위치는 월드 기준 m. 관절 값, 충돌 검사, 중력 계산은 로봇 자신의 좌표계(루트 기준)에서 하므로 이동의 영향을 받지 않는다.
    bool CanMoveSelection() const;
    void SetObjectPosition(float x, float y, float z);
    dhVector ObjectPosition() const;
    void ResetObjectPosition() { SetObjectPosition(0, 0, 0); }

    // 정적 모델(.glb 등)을 열었으면 그 장면
    bool HasScene() const { return hasScene_; }
    const amr::Scene& StaticScene() const { return scene_; }

    QString Summary() const;           // "삼각형 N개, 크기 ..." 같은 한 줄 설명
    const QStringList& Warnings() const { return warnings_; }

signals:
    void modelChanged();
    void collisionsChanged();  // 겹치는 링크 쌍이 달라졌을 때
    void objectMoved();  // 좌표계 화살표를 끌어서 위치가 바뀌었을 때
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
        std::vector<float> data;        // 위치(3) + 법선(3) 교차 배열
        std::vector<uint32_t> indices;
        dhMat base;                     // 링크(또는 월드)에서 이 primitive 까지의 변환
        int link;                       // 로봇 링크 인덱스, 정적 모델이면 -1
        float color[4];
        QOpenGLBuffer vbo;
        QOpenGLBuffer ibo;
        Item() : link(-1), vbo(QOpenGLBuffer::VertexBuffer), ibo(QOpenGLBuffer::IndexBuffer) {
            color[0] = color[1] = color[2] = 0.75f;
            color[3] = 1.0f;
        }
    };

    typedef std::vector<std::unique_ptr<Item> > ItemList;
    void addScene(const amr::Scene& scene, int link, const dhMat& local, const float* colorOverride, ItemList* dst = nullptr);
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
    void drawSelectionFrame(const QMatrix4x4& proj, const QMatrix4x4& view);
    void recomputeCollisions();
    void drawBoxEdges(const dhVector c[8], const QVector4D& color);
    QMatrix4x4 viewMatrix() const;

    std::vector<std::unique_ptr<Item> > items_;
    bool buffersDirty_;
    bool hasRobot_;
    amr::urdf::Robot robot_;
    amr::urdf::LoadOptions options_;
    std::vector<dhMat> linkT_;
    amr::urdf::JointValues q_;
    amr::RobotCollisionModel colModel_;
    amr::SelfCollisionReport report_;
    int selectedLink_;
    bool selectedModel_;
    bool showFrame_;
    ItemList gizmo_;        // 좌표계 모양 (LoadGizmo)
    bool gizmoDirty_;
    QPoint pressPos_;
    bool dragged_;
    dhVector objectPos_;   // 이동량 (월드, m)
    int dragAxis_;         // 좌표계 화살표를 끄는 중이면 0..2
    float dragU0_;
    dhVector dragStart_;
    amr::Scene scene_;
    bool hasScene_;
    QStringList warnings_;
    size_t triangleCount_;

    QVector3D center_;   // 모델 경계 상자의 중심/반지름 (월드)
    float radius_;
    QVector3D target_;   // 카메라가 보는 점
    float dist_, yaw_, pitch_;
    QPoint lastPos_;

    bool showBoxes_;
    bool showObbs_;
    bool showGrid_;
    std::vector<float> gridData_;   // 격자 선분의 위치(3) 배열
    float extent_[3];               // 경계 상자의 크기 (x, y, z)
    float gridCell_;                // 격자 한 칸의 길이

    std::unique_ptr<QOpenGLShaderProgram> prog_;
};

#endif  // CORECQT_MODELVIEWER_H
