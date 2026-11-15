#include "CoreQt/QtString.h"
#include "CoreQt/StructurePanel.h"

#include <QBrush>
#include <QFont>
#include <QHeaderView>
#include "CBase/Math.h"

namespace {

const int kLinkRole = Qt::UserRole;
const int kJointRole = Qt::UserRole + 1;  // 열 1 의 항목에 관절 이름을 넣어 둔다

QString Utf8(const char* s) { return QString::fromUtf8(s); }

QString JointKind(const amr::urdf::Joint& j) {
    const double deg = 180.0 / 3.14159265358979323846;
    switch (j.type) {
        case amr::urdf::kJointFixed: return Utf8("fixed");
        case amr::urdf::kJointRevolute:
            if (j.hasLimit)
                return QString("revolute [%1, %2] deg").arg(static_cast<double>(j.lower) * deg, 0, 'f', 1).arg(static_cast<double>(j.upper) * deg, 0, 'f', 1);
            return Utf8("revolute");
        case amr::urdf::kJointContinuous: return Utf8("continuous");
        case amr::urdf::kJointPrismatic:
            if (j.hasLimit) return QString("prismatic [%1, %2] m").arg(static_cast<double>(j.lower), 0, 'g', 4).arg(static_cast<double>(j.upper), 0, 'g', 4);
            return Utf8("prismatic");
        case amr::urdf::kJointFloating: return Utf8("floating");
        case amr::urdf::kJointPlanar: return Utf8("planar");
    }
    return QString();
}

}  // namespace

StructurePanel::StructurePanel(QWidget* parent) : QTreeWidget(parent) {
    setColumnCount(4);
    setHeaderLabels(QStringList() << Utf8("링크 / 노드") << Utf8("관절 / 메시") << Utf8("중력 하중") << Utf8("종류 / 비고"));
    setUniformRowHeights(true);
    setAlternatingRowColors(true);
    setSelectionMode(QAbstractItemView::SingleSelection);
    header()->setSectionResizeMode(QHeaderView::ResizeToContents);
    header()->setStretchLastSection(true);
    connect(this, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* cur, QTreeWidgetItem*) { onCurrentChanged(cur); });
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

void StructurePanel::SetRobot(const amr::urdf::Robot& robot) {
    Clear();
    linkItems_.assign(robot.links.size(), nullptr);
    if (robot.root < 0) return;

    amr::Array<amr::Array<int> > childJoints(robot.links.size());  // 링크 -> 그 링크가 부모인 관절들
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

void StructurePanel::addLink(QTreeWidgetItem* parent, const amr::urdf::Robot& robot, int link, int viaJoint,
                             const amr::Array<amr::Array<int> >& childJoints) {
    const amr::urdf::Link& l = robot.links[static_cast<size_t>(link)];
    QTreeWidgetItem* item = parent ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(this);
    item->setText(0, amrqt::ToQ(l.name));
    item->setData(0, kLinkRole, link);
    if (viaJoint >= 0) {
        const amr::urdf::Joint& j = robot.joints[static_cast<size_t>(viaJoint)];
        item->setText(1, amrqt::ToQ(j.name));
        item->setData(1, kJointRole, amrqt::ToQ(j.name));
        QString kind = JointKind(j);
        if (j.hasMimic) kind += Utf8("  (mimic ") + amrqt::ToQ(j.mimicJoint) + ")";
        item->setText(3, kind);
        item->setToolTip(1, QString("%1\n%2 -> %3\naxis (%4, %5, %6)")
                                .arg(amrqt::ToQ(j.name), amrqt::ToQ(j.parent), amrqt::ToQ(j.child))
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

    const amr::Array<int>& cj = childJoints[static_cast<size_t>(link)];
    for (size_t k = 0; k < cj.size(); k++) {
        const int child = robot.FindLink(robot.joints[static_cast<size_t>(cj[k])].child);
        if (child >= 0) addLink(item, robot, child, cj[k], childJoints);
    }
}

void StructurePanel::SetScene(const amr::Scene& scene) {
    Clear();
    blockSignals(true);
    for (size_t r = 0; r < scene.roots.size(); r++) addNode(nullptr, scene, scene.roots[r]);
    expandAll();
    blockSignals(false);
    setCurrentItem(nullptr);
}

void StructurePanel::addNode(QTreeWidgetItem* parent, const amr::Scene& scene, int node) {
    if (node < 0 || static_cast<size_t>(node) >= scene.nodes.size()) return;
    const amr::SceneNode& n = scene.nodes[static_cast<size_t>(node)];
    QTreeWidgetItem* item = parent ? new QTreeWidgetItem(parent) : new QTreeWidgetItem(this);
    item->setText(0, n.name.empty() ? Utf8("(이름 없음)") : amrqt::ToQ(n.name));
    item->setData(0, kLinkRole, -1);
    if (n.mesh >= 0 && static_cast<size_t>(n.mesh) < scene.meshes.size()) {
        const amr::Mesh& m = scene.meshes[static_cast<size_t>(n.mesh)];
        size_t tris = 0;
        for (size_t p = 0; p < m.primitives.size(); p++) tris += m.primitives[p].TriangleCount();
        item->setText(1, m.name.empty() ? Utf8("메시") : amrqt::ToQ(m.name));
        item->setText(3, QString("primitive %1, 삼각형 %2").arg(m.primitives.size()).arg(tris));
    }
    for (size_t c = 0; c < n.children.size(); c++) addNode(item, scene, n.children[c]);
}

void StructurePanel::SetCollidingLinks(const amr::Array<int>& links) {
    amr::Array<bool> hit(linkItems_.size(), false);
    for (size_t i = 0; i < links.size(); i++)
        if (links[i] >= 0 && static_cast<size_t>(links[i]) < hit.size()) hit[static_cast<size_t>(links[i])] = true;
    for (size_t i = 0; i < linkItems_.size(); i++) {
        QTreeWidgetItem* it = linkItems_[i];
        if (!it) continue;
        QFont f = it->font(0);
        f.setBold(hit[i]);
        for (int c = 0; c < columnCount(); c++) {
            it->setFont(c, f);
            it->setForeground(c, hit[i] ? QBrush(QColor(220, 50, 50)) : QBrush());
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

void StructurePanel::SetJointNotes(const amr::Map<amr::String, amr::Pair<QString, bool> >& notes) {
    for (size_t i = 0; i < linkItems_.size(); i++) {
        QTreeWidgetItem* it = linkItems_[i];
        if (!it) continue;
        const QString jn = it->data(1, kJointRole).toString();
        if (jn.isEmpty()) continue;
        amr::Map<amr::String, amr::Pair<QString, bool> >::const_iterator n = notes.find(amrqt::FromQ(jn));
        if (n == notes.end()) {
            it->setText(2, QString());
            continue;
        }
        it->setText(2, n->second.first);
        it->setForeground(2, n->second.second ? QBrush(QColor(220, 50, 50)) : QBrush());
        QFont f = it->font(2);
        f.setBold(n->second.second);
        it->setFont(2, f);
    }
}
