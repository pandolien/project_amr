#include "CoreQt/SimulView.h"

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QUrl>
#include <QVBoxLayout>

#include "3d/Units.h"
#include "3d/dhPose.h"
#include "CoreQt/QtString.h"

SimulView::SimulView(QWidget* parent, unsigned features) : QWidget(parent), viewer_(nullptr), overlay_(nullptr), placeTimer_(nullptr), features_(0), panelLink_(-1) {
    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    viewer_ = new ModelViewer(this);
    lay->addWidget(viewer_);
    overlay_ = new SelectionOverlay(viewer_);   // 3D 화면 위에 뜬다
    connect(overlay_, &SelectionOverlay::valueEdited, this, [this](const QString& name, double value) {
        simul::urdf::JointValues q = viewer_->CurrentJointValues();
        q[simulqt::FromQ(name)] = static_cast<float>(value);
        viewer_->SetJointValues(q);
        emit jointEdited(name, value);
    });
    placeTimer_ = new QTimer(this);
    connect(placeTimer_, &QTimer::timeout, this, &SimulView::placePanel);
    placeTimer_->start(40);

    // 선택이나 구성이 바뀌면 패널을 새로 채우고, 자세나 관절 값만 바뀌면 숫자만 갱신한다
    connect(viewer_, &ModelViewer::selectionChanged, this, &SimulView::refreshPanel);
    connect(viewer_, &ModelViewer::selectionPicked, this, &SimulView::refreshPanel);
    connect(viewer_, &ModelViewer::robotChanged, this, &SimulView::refreshPanel);
    connect(viewer_, &ModelViewer::structureChanged, this, &SimulView::refreshPanel);
    connect(viewer_, &ModelViewer::modelChanged, this, &SimulView::syncPanel);
    connect(viewer_, &ModelViewer::objectMoved, this, &SimulView::syncPanel);
    connect(viewer_, &ModelViewer::jointValuesChanged, this, &SimulView::syncPanel);
    connect(viewer_, &ModelViewer::jointValuesDragged, this, &SimulView::syncPanel);
    SetFeatures(features);
}

void SimulView::SetFeatures(unsigned features) {
    features_ = features;
    viewer_->SetMouseEditEnabled((features & kMouseEdit) != 0);
    setAcceptDrops((features & kDropFiles) != 0);
    if (features & kSelectionPanel) refreshPanel();
    else overlay_->hide();
}

void SimulView::dragEnterEvent(QDragEnterEvent* e) {
    if ((features_ & kDropFiles) && e->mimeData()->hasUrls()) e->acceptProposedAction();
}

void SimulView::dropEvent(QDropEvent* e) {
    if (!(features_ & kDropFiles)) return;
    QStringList paths;
    const QList<QUrl> urls = e->mimeData()->urls();
    for (int i = 0; i < urls.size(); i++) paths << urls[i].toLocalFile();
    if (receivers(SIGNAL(filesDropped(QStringList))) > 0) {
        emit filesDropped(paths);
    } else {   // 호스트가 따로 처리하지 않으면 그냥 장면에 추가한다
        for (int i = 0; i < paths.size(); i++) viewer_->AddFile(paths[i]);
    }
}

void SimulView::resizeEvent(QResizeEvent* e) {
    QWidget::resizeEvent(e);
    placePanel();
}

// 링크의 부모 관절이 움직이면 그 관절 하나, 모델의 루트 링크면 그 모델의 모든 관절을 슬라이더로 보인다
void SimulView::refreshPanel() {
    if (!(features_ & kSelectionPanel)) {
        overlay_->hide();
        panelLink_ = -1;
        return;
    }
    const int link = viewer_->SelectedLink();
    const simul::urdf::Robot& robot = viewer_->Robot();
    if (!viewer_->HasRobot() || link < 0 || static_cast<size_t>(link) >= robot.links.size()) {
        overlay_->hide();
        panelLink_ = -1;
        return;
    }
    panelLink_ = link;
    const simul::String& lname = robot.links[static_cast<size_t>(link)].name;
    const int model = viewer_->ModelOfLink(link);
    QVector<SelectionOverlay::Joint> joints;
    const simul::Array<int> act = robot.ActuatedJoints();
    for (size_t k = 0; k < act.size(); k++) {
        const simul::urdf::Joint& j = robot.joints[static_cast<size_t>(act[k])];
        const bool mine = viewer_->ModelOfRoot(link) >= 0 ? viewer_->ModelOfLink(robot.FindLink(j.child)) == model : j.child == lname;
        if (!mine) continue;
        SelectionOverlay::Joint o;
        o.name = simulqt::ToQ(j.name);
        o.angular = j.IsAngular();
        const bool unlimited = j.type == simul::urdf::kJointContinuous || !j.hasLimit;
        o.lower = unlimited ? (o.angular ? -3.14159265358979 : -1.0) : static_cast<double>(j.lower);
        o.upper = unlimited ? (o.angular ? 3.14159265358979 : 1.0) : static_cast<double>(j.upper);
        const simul::urdf::JointValues& q = viewer_->CurrentJointValues();
        const simul::urdf::JointValues::const_iterator it = q.find(j.name);
        o.value = it == q.end() ? 0.0 : static_cast<double>(it->second);
        joints.push_back(o);
    }
    overlay_->SetContent(simulqt::ToQ(lname), panelInfo(link), joints);
    overlay_->show();
    placePanel();
}

void SimulView::syncPanel() {
    if (!(features_ & kSelectionPanel)) return;
    if (viewer_->SelectedLink() != panelLink_) {   // 선택이 달라졌으면 새로 채운다
        refreshPanel();
        return;
    }
    if (!overlay_->isVisible()) return;
    overlay_->SetInfo(panelInfo(panelLink_));
    overlay_->ShowValues(viewer_->CurrentJointValues());
}

QString SimulView::panelInfo(int link) const {
    dhMat f;
    if (!viewer_->SelectedFrame(&f)) return QString();
    const dhPose pose = dhPose::FromMat(f).Normalized();
    const simul::units::RpyDeg r = simul::units::RpyDegFromQuat(pose.orientation);
    const QString model = viewer_->ModelName(viewer_->ModelOfLink(link));
    QString s = QString::fromUtf8("모델 %1<br>위치 %2, %3, %4 m<br>RPY %5°, %6°, %7°")
                    .arg(model.isEmpty() ? QString("-") : model)
                    .arg(static_cast<double>(pose.position.x), 0, 'f', 3).arg(static_cast<double>(pose.position.y), 0, 'f', 3).arg(static_cast<double>(pose.position.z), 0, 'f', 3)
                    .arg(r.roll, 0, 'f', 1).arg(r.pitch, 0, 'f', 1).arg(r.yaw, 0, 'f', 1);
    if (features_ & kMouseEdit) {
        if (viewer_->CanMoveSelection()) s += QString::fromUtf8("<br>화살표: 이동, 링: 회전");
        else if (viewer_->CanIkSelection()) s += QString::fromUtf8("<br>화살표를 끌면 관절이 따라옵니다");
    }
    return s;
}

void SimulView::placePanel() {
    if (!overlay_ || !overlay_->isVisible()) return;
    QPoint p;
    if (!viewer_->SelectionScreenPos(&p)) return;
    overlay_->PlaceNear(p);
}
