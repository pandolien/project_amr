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

    // AABB 충돌 (로봇 링크 자기충돌 후보). 링크마다 AABB 와이어 박스를 그리고, 겹친 링크는 빨갛게 그린다.
    // <collision> 이 없는 링크는 <visual> 로 대신한다. 관절 값이 바뀔 때마다 다시 계산한다.
    void SetShowCollisionBoxes(bool show);
    bool ShowCollisionBoxes() const { return showBoxes_; }
    const std::vector<amr::CollisionPair>& SelfCollisions() const { return selfCollisions_; }  // 링크 인덱스 쌍
    QStringList CollisionDescriptions() const;  // "링크A <-> 링크B"
    QString Summary() const;           // "삼각형 N개, 크기 ..." 같은 한 줄 설명
    const QStringList& Warnings() const { return warnings_; }

signals:
    void modelChanged();
    void collisionsChanged();  // 겹치는 링크 쌍이 달라졌을 때

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
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

    void addScene(const amr::Scene& scene, int link, const dhMat& local, const float* colorOverride);
    void updateBounds();
    void rebuildGrid();
    void uploadBuffers();
    void releaseBuffers();
    void recomputeCollisions();
    QMatrix4x4 viewMatrix() const;

    std::vector<std::unique_ptr<Item> > items_;
    bool buffersDirty_;
    bool hasRobot_;
    amr::urdf::Robot robot_;
    amr::urdf::LoadOptions options_;
    std::vector<dhMat> linkT_;
    amr::urdf::JointValues q_;
    amr::RobotCollisionModel colModel_;
    std::vector<amr::Aabb> linkBoxes_;
    std::vector<amr::CollisionPair> selfCollisions_;
    QStringList warnings_;
    size_t triangleCount_;

    QVector3D center_;   // 모델 경계 상자의 중심/반지름 (월드)
    float radius_;
    QVector3D target_;   // 카메라가 보는 점
    float dist_, yaw_, pitch_;
    QPoint lastPos_;

    bool showBoxes_;
    bool showGrid_;
    std::vector<float> gridData_;   // 격자 선분의 위치(3) 배열
    float extent_[3];               // 경계 상자의 크기 (x, y, z)
    float gridCell_;                // 격자 한 칸의 길이

    std::unique_ptr<QOpenGLShaderProgram> prog_;
};

#endif  // CORECQT_MODELVIEWER_H
