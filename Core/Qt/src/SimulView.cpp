#include "CoreQt/SimulView.h"

#include <QActionGroup>
#include <QDragEnterEvent>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QToolButton>
#include <QDropEvent>
#include <QMimeData>
#include <QUrl>
#include <QVBoxLayout>

#include "3d/Units.h"
#include "3d/dhPose.h"
#include "CoreQt/QtString.h"

SimulView::SimulView(QWidget* parent, unsigned features) : QWidget(parent), viewer_(nullptr), overlay_(nullptr), tree_(nullptr), treePanel_(nullptr), treePlaced_(false), menuBar_(nullptr), customMenus_(false), placeTimer_(nullptr), features_(0), panelLink_(-1) {
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
    buildMenuBar();
    // 구조 트리: 3D 화면 안에 뜨는 창. 선택과 충돌 표시는 뷰와 직접 맞춘다
    tree_ = new StructurePanel;
    tree_->SetDarkTheme(true);
    treePanel_ = new FloatingPanel(QString::fromUtf8("구조"), tree_, viewer_);
    connect(treePanel_, &FloatingPanel::closeRequested, this, [this]() { SetTreeVisible(false); });
    connect(tree_, &StructurePanel::linkSelected, this, [this](int link) { viewer_->SetSelectedLink(link); });
    connect(tree_, &StructurePanel::linkContextMenuRequested, this, [this](int link, const QPoint& p) {
        if (receivers(SIGNAL(linkContextMenuRequested(int, QPoint))) > 0) {
            emit linkContextMenuRequested(link, p);
            return;
        }
        const int model = viewer_->ModelOfRoot(link);
        if (model < 0) return;
        QMenu menu;
        QAction* remove = menu.addAction(QString::fromUtf8("제거 (%1)").arg(viewer_->ModelName(model)));
        if (menu.exec(p) == remove) viewer_->RemoveModel(model);
    });
    connect(viewer_, &ModelViewer::robotChanged, this, &SimulView::refreshTree);
    connect(viewer_, &ModelViewer::structureChanged, this, &SimulView::refreshTree);
    connect(viewer_, &ModelViewer::collisionsChanged, this, [this]() { tree_->SetCollidingLinks(viewer_->CollidingLinks()); });
    connect(viewer_, &ModelViewer::selectionChanged, this, [this](int link) { tree_->SelectLink(link); });
    connect(viewer_, &ModelViewer::selectionPicked, this, [this](int link, bool) { tree_->SelectLink(link); });
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
    const unsigned before = features_;
    features_ = features;
    viewer_->SetMouseEditEnabled((features & kMouseEdit) != 0);
    setAcceptDrops((features & kDropFiles) != 0);
    menuBar_->setVisible((features & kMenu) != 0);
    layoutMenuBar();
    updateTreeVisibility();
    if (features & kSelectionPanel) refreshPanel();
    else overlay_->hide();
    if (features != before) emit featuresChanged(features);
}

// ---- 구조 트리 창 ----

void SimulView::refreshTree() {
    if (viewer_->HasRobot()) {
        tree_->SetRobot(viewer_->Robot());
        tree_->SetCollidingLinks(viewer_->CollidingLinks());
    } else if (viewer_->HasScene()) {
        tree_->SetScene(viewer_->StaticScene());
    } else {
        tree_->Clear();
    }
    tree_->SelectLink(viewer_->SelectedLink());
    updateTreeVisibility();
}

// 켜져 있고 보여 줄 것이 있을 때만 보인다. 처음 보일 때 왼쪽 위에 놓는다.
void SimulView::updateTreeVisibility() {
    const bool show = (features_ & kTree) && tree_->topLevelItemCount() > 0;
    if (show && !treePlaced_ && viewer_->width() > 0) {
        treePanel_->SetTopInset((features_ & kMenu) ? menuBar_->height() : 0);
        treePanel_->PlaceDefault(QPoint(8, 8 + ((features_ & kMenu) ? menuBar_->height() : 0)), QSize(340, qMin(320, qMax(200, viewer_->height() - 60))));
        treePlaced_ = true;
    }
    if (show != treePanel_->isVisibleTo(viewer_)) {
        treePanel_->setVisible(show);
        if (show) treePanel_->raise();
    }
}

// ---- 3D 화면 안의 메뉴줄 ----

void SimulView::buildMenuBar() {
    menuBar_ = new QFrame(viewer_);
    menuBar_->setObjectName("simulMenuBar");
    menuBar_->setStyleSheet(
        "#simulMenuBar { background: rgba(28, 30, 38, 215); border-bottom: 1px solid rgba(255,255,255,50); }"
        "#simulMenuBar QToolButton { color: #e8eaf0; background: transparent; border: none; padding: 4px 12px; font-size: 12px; }"
        "#simulMenuBar QToolButton:hover, #simulMenuBar QToolButton:pressed { background: rgba(106,169,255,90); }"
        "#simulMenuBar QToolButton::menu-indicator { image: none; }");
    new QHBoxLayout(menuBar_);
    menuBar_->layout()->setContentsMargins(4, 0, 4, 0);
    menuBar_->layout()->setSpacing(0);
    menuBar_->hide();
    ownedMenus_ = defaultMenus();
    menus_ = ownedMenus_;
    applyMenus();
}

// 호스트 없이도 쓸 수 있는 기본 메뉴: ModelViewer 만으로 되는 것만 넣는다
QList<QMenu*> SimulView::defaultMenus() {
    QList<QMenu*> list;
    QMenu* file = new QMenu(QString::fromUtf8("파일"), this);
    QAction* open = file->addAction(QString::fromUtf8("열기 (추가)..."));
    connect(open, &QAction::triggered, this, [this]() {
        const QStringList paths = QFileDialog::getOpenFileNames(this, QString::fromUtf8("모델 또는 URDF 열기 (현재 장면에 추가됩니다)"), QString(),
                                                                QString::fromUtf8("3D 모델 / URDF (*.urdf *.glb *.gltf *.stl *.obj *.dae);;모든 파일 (*)"));
        if (paths.isEmpty()) return;
        if (receivers(SIGNAL(filesDropped(QStringList))) > 0) emit filesDropped(paths);
        else for (int i = 0; i < paths.size(); i++) viewer_->AddFile(paths[i]);
    });
    connect(file->addAction(QString::fromUtf8("모두 닫기")), &QAction::triggered, viewer_, &ModelViewer::CloseAll);
    list << file;

    QMenu* view = new QMenu(QString::fromUtf8("보기"), this);
    connect(view->addAction(QString::fromUtf8("화면에 맞춤")), &QAction::triggered, viewer_, &ModelViewer::FitView);
    QAction* grid = view->addAction(QString::fromUtf8("격자"));
    grid->setCheckable(true);
    grid->setChecked(true);
    connect(grid, &QAction::toggled, viewer_, &ModelViewer::SetShowGrid);
    QAction* frame = view->addAction(QString::fromUtf8("선택 좌표계"));
    frame->setCheckable(true);
    frame->setChecked(true);
    connect(frame, &QAction::toggled, viewer_, &ModelViewer::SetShowSelectionFrame);
    view->addSeparator();
    QMenu* shade = view->addMenu(QString::fromUtf8("조명 방식"));
    QActionGroup* shadeGroup = new QActionGroup(this);
    const char* shadeNames[3] = {"Lambert (무광)", "Blinn-Phong (하이라이트)", "PBR (물리 기반)"};
    for (int m = 0; m < 3; m++) {
        QAction* a = shade->addAction(QString::fromUtf8(shadeNames[m]));
        a->setCheckable(true);
        a->setChecked(m == 2);
        shadeGroup->addAction(a);
        connect(a, &QAction::triggered, this, [this, m]() { viewer_->SetShading(m); });
    }
    QMenu* box = view->addMenu(QString::fromUtf8("충돌 박스"));
    QAction* obb = box->addAction(QString::fromUtf8("OBB"));
    obb->setCheckable(true);
    connect(obb, &QAction::toggled, viewer_, &ModelViewer::SetShowObbs);
    QAction* aabb = box->addAction(QString::fromUtf8("AABB"));
    aabb->setCheckable(true);
    connect(aabb, &QAction::toggled, viewer_, &ModelViewer::SetShowCollisionBoxes);
    QMenu* ring = view->addMenu(QString::fromUtf8("회전 링 기준"));
    QActionGroup* ringGroup = new QActionGroup(this);
    const char* ringNames[2] = {"월드 축 (기본)", "오브젝트 축"};
    for (int k = 0; k < 2; k++) {
        QAction* a = ring->addAction(QString::fromUtf8(ringNames[k]));
        a->setCheckable(true);
        a->setChecked(k == 0);
        ringGroup->addAction(a);
        connect(a, &QAction::triggered, this, [this, k]() { viewer_->SetRingLocal(k == 1); });
    }
    list << view;

    QMenu* phys = new QMenu(QString::fromUtf8("물리"), this);
    QAction* fall = phys->addAction(QString::fromUtf8("중력 낙하"));
    fall->setCheckable(true);
    fall->setChecked(true);
    connect(fall, &QAction::toggled, viewer_, &ModelViewer::SetFallEnabled);
    connect(phys->addAction(QString::fromUtf8("오브젝트 위치 초기화")), &QAction::triggered, viewer_, &ModelViewer::ResetObjectPosition);
    list << phys;
    return list;
}

void SimulView::applyMenus() {
    QLayout* lay = menuBar_->layout();
    while (QLayoutItem* it = lay->takeAt(0)) {
        delete it->widget();
        delete it;
    }
    for (int i = 0; i < menus_.size(); i++) {
        QToolButton* b = new QToolButton(menuBar_);
        b->setText(menus_[i]->title());
        b->setPopupMode(QToolButton::InstantPopup);
        b->setMenu(menus_[i]);
        b->setCursor(Qt::ArrowCursor);
        lay->addWidget(b);
    }
    static_cast<QHBoxLayout*>(lay)->addStretch(1);
    layoutMenuBar();
}

void SimulView::layoutMenuBar() {
    menuBar_->adjustSize();
    menuBar_->setGeometry(0, 0, viewer_->width(), menuBar_->sizeHint().height() > 0 ? menuBar_->sizeHint().height() : 26);
    menuBar_->raise();
    overlay_->SetTopInset((features_ & kMenu) ? menuBar_->height() : 0);
    if (treePanel_) {
        treePanel_->SetTopInset((features_ & kMenu) ? menuBar_->height() : 0);
        treePanel_->Clamp();
    }
}

void SimulView::SetMenus(const QList<QMenu*>& menus) {
    customMenus_ = true;
    menus_ = menus;
    applyMenus();
}

QMenu* SimulView::AddMenu(const QString& title) {
    if (customMenus_) ResetMenus();
    QMenu* m = new QMenu(title, this);
    ownedMenus_ << m;
    menus_ = ownedMenus_;
    applyMenus();
    return m;
}

void SimulView::ResetMenus() {
    customMenus_ = false;
    menus_ = ownedMenus_;
    applyMenus();
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
    layoutMenuBar();
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
    if (tree_) updateTreeVisibility();   // 호스트가 트리를 직접 채웠을 수도 있어서 함께 본다
    if (!overlay_ || !overlay_->isVisible()) return;
    QPoint p;
    if (!viewer_->SelectionScreenPos(&p)) return;
    overlay_->PlaceNear(p);
}
