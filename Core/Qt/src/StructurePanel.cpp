#include "CoreQt/QtString.h"
#include "CoreQt/StructurePanel.h"

#include <QBrush>
#include <QFont>
#include <QHeaderView>
#include <QIcon>
#include <QPainter>
#include <QPixmap>
#include "CBase/Math.h"

namespace {

const int kLinkRole = Qt::UserRole;
const int kJointRole = Qt::UserRole + 1;  // 열 1 의 항목에 관절 이름을 넣어 둔다

QString Utf8(const char* s) { return QString::fromUtf8(s); }

QString JointKind(const simul::urdf::Joint& j) {
    const double deg = 180.0 / 3.14159265358979323846;
    switch (j.type) {
        case simul::urdf::kJointFixed: return Utf8("fixed");
        case simul::urdf::kJointRevolute:
            if (j.hasLimit)
                return QString("revolute [%1, %2] deg").arg(static_cast<double>(j.lower) * deg, 0, 'f', 1).arg(static_cast<double>(j.upper) * deg, 0, 'f', 1);
            return Utf8("revolute");
        case simul::urdf::kJointContinuous: return Utf8("continuous");
        case simul::urdf::kJointPrismatic:
            if (j.hasLimit) return QString("prismatic [%1, %2] m").arg(static_cast<double>(j.lower), 0, 'g', 4).arg(static_cast<double>(j.upper), 0, 'g', 4);
            return Utf8("prismatic");
        case simul::urdf::kJointFloating: return Utf8("floating");
        case simul::urdf::kJointPlanar: return Utf8("planar");
    }
    return QString();
}

// 관절 종류를 나타내는 작은 색 점 (회전 파랑, 직동 초록, 고정 회색, 루트 주황)
QIcon Dot(const QColor& color) {
    QPixmap pm(14, 14);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(Qt::NoPen);
    p.setBrush(color);
    p.drawEllipse(QRectF(2.5, 2.5, 9, 9));
    return QIcon(pm);
}

QColor DotColor(const simul::urdf::Joint* j) {
    if (!j) return QColor(240, 170, 80);
    switch (j->type) {
        case simul::urdf::kJointRevolute:
        case simul::urdf::kJointContinuous: return QColor(96, 165, 250);
        case simul::urdf::kJointPrismatic: return QColor(110, 210, 140);
        case simul::urdf::kJointFixed: return QColor(130, 138, 156);
        default: return QColor(190, 140, 240);
    }
}

const QColor kMuted(139, 147, 167);
const QColor kHit(255, 110, 110);

}  // namespace

void StructurePanel::SetDarkTheme(bool on) {
    dark_ = on;
    setAlternatingRowColors(!on);
    header()->setVisible(!on);
    setIconSize(QSize(14, 14));
    setColumnHidden(3, on);   // 종류/비고 열은 작은 창에서 가로로 넘치므로 숨긴다 (관절 종류는 색 점으로 보인다)
    setHorizontalScrollBarPolicy(on ? Qt::ScrollBarAlwaysOff : Qt::ScrollBarAsNeeded);
    setIndentation(on ? 16 : 20);
    setFrameShape(on ? QFrame::NoFrame : QFrame::StyledPanel);
    setUniformRowHeights(true);
    setStyleSheet(on ? QString::fromUtf8(
        "QTreeWidget { background: transparent; color: #e8eaf0; outline: 0; font-size: 12px; }"
        "QTreeWidget::item { padding: 4px 2px; }"
        "QTreeWidget::item:hover { background: rgba(255,255,255,18); }"
        "QTreeWidget::item:selected, QTreeWidget::item:selected:active, QTreeWidget::item:selected:!active { background: #2f4a72; color: white; }"
        "QTreeView::branch { background: transparent; }"
        "QTreeView::branch:hover { background: rgba(255,255,255,18); }"
        "QTreeView::branch:selected, QTreeView::branch:selected:active, QTreeView::branch:selected:!active, QTreeView::branch:has-children:selected { background: #2f4a72; }"
        "QScrollBar:vertical { background: transparent; width: 8px; margin: 2px; }"
        "QScrollBar::handle:vertical { background: rgba(255,255,255,60); border-radius: 3px; min-height: 24px; }"
        "QScrollBar:horizontal { background: transparent; height: 8px; margin: 2px; }"
        "QScrollBar::handle:horizontal { background: rgba(255,255,255,60); border-radius: 3px; min-width: 24px; }"
        "QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }"
        "QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }") : QString());
    // 이미 채워진 항목의 보조 열 색도 맞춘다
    for (size_t i = 0; i < linkItems_.size(); i++) {
        QTreeWidgetItem* it = linkItems_[i];
        if (!it) continue;
        for (int c = 1; c < columnCount(); c++) it->setForeground(c, on ? QBrush(kMuted) : QBrush());
    }
}

StructurePanel::StructurePanel(QWidget* parent) : QTreeWidget(parent) {
    setColumnCount(4);
    setHeaderLabels(QStringList() << Utf8("링크 / 노드") << Utf8("관절 / 메시") << Utf8("중력 하중") << Utf8("종류 / 비고"));
    setUniformRowHeights(true);
    setAlternatingRowColors(true);
    setSelectionMode(QAbstractItemView::SingleSelection);
    header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    header()->setStretchLastSection(true);
    connect(this, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* cur, QTreeWidgetItem*) { onCurrentChanged(cur); });
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, &QWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
        QTreeWidgetItem* it = itemAt(pos);
        emit linkContextMenuRequested(it ? it->data(0, kLinkRole).toInt() : -1, viewport()->mapToGlobal(pos));
    });
}

void StructurePanel::Clear() {
    blockSignals(true);
    clear();
    blockSignals(false);
    linkItems_.clear();
}

void StructurePanel::onCurrentChanged(QTreeWidgetItem* cur) {
    emit linkSelected(cur ? cur->data(0, kLinkRole).toInt() : -1);
}

void StructurePanel::SetRobot(const simul::urdf::Robot& robot) {
    Clear();
    linkItems_.assign(robot.links.size(), nullptr);
    if (robot.root < 0) return;

    simul::Array<simul::Array<int> > childJoints(robot.links.size());  // 링크 -> 그 링크가 부모인 관절들
    for (size_t j = 0; j < robot.joints.size(); j++) {
        const int p = robot.FindLink(robot.joints[j].parent);
        if (p >= 0) childJoints[static_cast<size_t>(p)].push_back(static_cast<int>(j));
    }

    blockSignals(true);
    addLink(nullptr, robot, robot.root, -1, childJoints);
    expandAll();
    blockSignals(false);
    setCurrentItem(nullptr);
}

void StructurePanel::addLink(QTreeWidgetItem* parent, const simul::urdf::Robot& robot, int link, int viaJoint,
                             const simul::Array<simul::Array<int> >& childJoints) {
    const simul::urdf::Link& l = robot.links[static_cast<size_t>(link)];
    QTreeWidgetItem* item = parent ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(this);
    item->setText(0, simulqt::ToQ(l.name));
    item->setData(0, kLinkRole, link);
    item->setIcon(0, Dot(DotColor(viaJoint >= 0 ? &robot.joints[static_cast<size_t>(viaJoint)] : nullptr)));
    if (dark_) for (int c = 1; c < columnCount(); c++) item->setForeground(c, QBrush(kMuted));
    if (viaJoint >= 0) {
        const simul::urdf::Joint& j = robot.joints[static_cast<size_t>(viaJoint)];
        item->setText(1, simulqt::ToQ(j.name));
        item->setData(1, kJointRole, simulqt::ToQ(j.name));
        QString kind = JointKind(j);
        if (j.hasMimic) kind += Utf8("  (mimic ") + simulqt::ToQ(j.mimicJoint) + ")";
        item->setText(3, kind);
        item->setToolTip(1, QString("%1\n%2 -> %3\naxis (%4, %5, %6)")
                                .arg(simulqt::ToQ(j.name), simulqt::ToQ(j.parent), simulqt::ToQ(j.child))
                                .arg(static_cast<double>(j.axis.x)).arg(static_cast<double>(j.axis.y)).arg(static_cast<double>(j.axis.z)));
    } else {
        item->setText(1, Utf8("(루트)"));
    }
    // 비고: visual / collision 개수와 질량
    QStringList note;
    if (!l.visuals.empty()) note << QString("visual %1").arg(l.visuals.size());
    if (!l.collisions.empty()) note << QString("collision %1").arg(l.collisions.size());
    if (l.hasInertial) note << QString("%1 kg").arg(static_cast<double>(l.inertial.mass), 0, 'g', 4);
    if (viaJoint < 0 || item->text(3).isEmpty()) {
        item->setText(3, note.join(", "));
    } else if (!note.isEmpty()) {
        item->setToolTip(0, note.join(", "));
    }
    linkItems_[static_cast<size_t>(link)] = item;

    const simul::Array<int>& cj = childJoints[static_cast<size_t>(link)];
    for (size_t k = 0; k < cj.size(); k++) {
        const int child = robot.FindLink(robot.joints[static_cast<size_t>(cj[k])].child);
        if (child >= 0) addLink(item, robot, child, cj[k], childJoints);
    }
}

void StructurePanel::SetScene(const simul::Scene& scene) {
    Clear();
    blockSignals(true);
    for (size_t r = 0; r < scene.roots.size(); r++) addNode(nullptr, scene, scene.roots[r]);
    expandAll();
    blockSignals(false);
    setCurrentItem(nullptr);
}

void StructurePanel::addNode(QTreeWidgetItem* parent, const simul::Scene& scene, int node) {
    if (node < 0 || static_cast<size_t>(node) >= scene.nodes.size()) return;
    const simul::SceneNode& n = scene.nodes[static_cast<size_t>(node)];
    QTreeWidgetItem* item = parent ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(this);
    item->setText(0, n.name.empty() ? Utf8("(이름 없음)") : simulqt::ToQ(n.name));
    item->setData(0, kLinkRole, -1);
    if (n.mesh >= 0 && static_cast<size_t>(n.mesh) < scene.meshes.size()) {
        const simul::Mesh& m = scene.meshes[static_cast<size_t>(n.mesh)];
        size_t tris = 0;
        for (size_t p = 0; p < m.primitives.size(); p++) tris += m.primitives[p].TriangleCount();
        item->setText(1, m.name.empty() ? Utf8("메시") : simulqt::ToQ(m.name));
        item->setText(3, QString("primitive %1, 삼각형 %2").arg(m.primitives.size()).arg(tris));
    }
    for (size_t c = 0; c < n.children.size(); c++) addNode(item, scene, n.children[c]);
}

void StructurePanel::SetCollidingLinks(const simul::Array<int>& links) {
    simul::Array<bool> hit(linkItems_.size(), false);
    for (size_t i = 0; i < links.size(); i++)
        if (links[i] >= 0 && static_cast<size_t>(links[i]) < hit.size()) hit[static_cast<size_t>(links[i])] = true;
    for (size_t i = 0; i < linkItems_.size(); i++) {
        QTreeWidgetItem* it = linkItems_[i];
        if (!it) continue;
        QFont f = it->font(0);
        f.setBold(hit[i]);
        for (int c = 0; c < columnCount(); c++) {
            it->setFont(c, f);
            it->setForeground(c, hit[i] ? QBrush(dark_ ? kHit : QColor(220, 50, 50)) : (dark_ && c > 0 ? QBrush(kMuted) : QBrush()));
        }
    }
}

void StructurePanel::SelectLink(int link) {
    blockSignals(true);
    if (link >= 0 && static_cast<size_t>(link) < linkItems_.size() && linkItems_[static_cast<size_t>(link)])
        setCurrentItem(linkItems_[static_cast<size_t>(link)]);
    else
        setCurrentItem(nullptr);
    blockSignals(false);
}

void StructurePanel::SetJointNotes(const simul::Map<simul::String, simul::Pair<QString, bool> >& notes) {
    for (size_t i = 0; i < linkItems_.size(); i++) {
        QTreeWidgetItem* it = linkItems_[i];
        if (!it) continue;
        const QString jn = it->data(1, kJointRole).toString();
        if (jn.isEmpty()) continue;
        simul::Map<simul::String, simul::Pair<QString, bool> >::const_iterator n = notes.find(simulqt::FromQ(jn));
        if (n == notes.end()) {
            it->setText(2, QString());
            continue;
        }
        it->setText(2, n->second.first);
        it->setForeground(2, n->second.second ? QBrush(dark_ ? kHit : QColor(220, 50, 50)) : (dark_ ? QBrush(kMuted) : QBrush()));
        QFont f = it->font(2);
        f.setBold(n->second.second);
        it->setFont(2, f);
    }
}
