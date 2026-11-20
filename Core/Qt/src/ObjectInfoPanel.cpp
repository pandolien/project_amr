#include "CoreQt/ObjectInfoPanel.h"

#include <QHeaderView>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "3d/Units.h"
#include "CoreQt/QtString.h"

using simul::sim::JointInfo;
using simul::sim::ObjectInfo;

namespace {
const int kNameRole = Qt::UserRole + 1;   // 더블클릭 때 꺼낼 이름 (오브젝트 또는 관절)
}

ObjectInfoPanel::ObjectInfoPanel(simul::sim::Backend* backend, QWidget* parent) : QWidget(parent), backend_(backend) {
    tree_ = new QTreeWidget(this);
    tree_->setColumnCount(3);
    tree_->setHeaderLabels(QStringList() << QString::fromUtf8("이름") << QString::fromUtf8("값") << QString::fromUtf8("한계 / 종류"));
    tree_->setRootIsDecorated(true);
    tree_->setUniformRowHeights(true);
    tree_->header()->setStretchLastSection(true);
    tree_->setToolTip(QString::fromUtf8("더블클릭하면 편집기에 이름이 들어갑니다"));
    QVBoxLayout* lay = new QVBoxLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(tree_);
    timer_ = new QTimer(this);
    timer_->setInterval(100);
    QObject::connect(timer_, &QTimer::timeout, this, [this]() { Refresh(); });
    QObject::connect(tree_, &QTreeWidget::itemDoubleClicked, this, &ObjectInfoPanel::onDoubleClicked);
    timer_->start();
}

void ObjectInfoPanel::SetIntervalMs(int ms) { timer_->setInterval(ms < 20 ? 20 : ms); }

void ObjectInfoPanel::showEvent(QShowEvent* e) {
    QWidget::showEvent(e);
    Refresh();
}

QString ObjectInfoPanel::FormatValue(const JointInfo& j) {
    return QString::number(j.value, 'f', 4) + (j.kind == simul::sim::kJointPrismaticKind ? QStringLiteral(" m") : QStringLiteral(" rad"));
}

QString ObjectInfoPanel::FormatLimit(const JointInfo& j) {
    if (j.kind == simul::sim::kJointContinuousKind) return QString::fromUtf8("제한 없음 (연속)");
    if (!j.hasLimit) return QStringLiteral("-");
    return QStringLiteral("%1 ~ %2").arg(j.lower, 0, 'f', 3).arg(j.upper, 0, 'f', 3);
}

QString ObjectInfoPanel::FormatPose(const dhPose& p) {
    const simul::units::RpyDeg r = simul::units::RpyDegFromQuat(p.orientation);
    return QStringLiteral("(%1, %2, %3) m  rpy(%4, %5, %6)°")
        .arg(p.position.x, 0, 'f', 3).arg(p.position.y, 0, 'f', 3).arg(p.position.z, 0, 'f', 3)
        .arg(r.roll, 0, 'f', 1).arg(r.pitch, 0, 'f', 1).arg(r.yaw, 0, 'f', 1);
}

bool ObjectInfoPanel::structureChanged(const simul::Array<ObjectInfo>& list) const {
    if (tree_->topLevelItemCount() != static_cast<int>(list.size())) return true;
    for (size_t i = 0; i < list.size(); i++) {
        QTreeWidgetItem* it = tree_->topLevelItem(static_cast<int>(i));
        if (it->data(0, kNameRole).toString() != simulqt::ToQ(list[i].name)) return true;
        if (it->childCount() != static_cast<int>(list[i].joints.size())) return true;
        for (size_t k = 0; k < list[i].joints.size(); k++)
            if (it->child(static_cast<int>(k))->data(0, kNameRole).toString() != simulqt::ToQ(list[i].joints[k].name)) return true;
    }
    return false;
}

void ObjectInfoPanel::Refresh() {
    if (!backend_) { tree_->clear(); return; }
    simul::Array<ObjectInfo> list;
    const int n = backend_->ObjectCount();
    for (int i = 0; i < n; i++) {
        ObjectInfo info;
        if (backend_->ReadObject(i, &info)) list.push_back(info);
    }
    bool needResize = false;
    if (structureChanged(list)) {   // 오브젝트나 관절이 바뀐 때만 다시 만든다 (펼침 상태 유지)
        QStringList open;
        for (int i = 0; i < tree_->topLevelItemCount(); i++)
            if (tree_->topLevelItem(i)->isExpanded()) open << tree_->topLevelItem(i)->data(0, kNameRole).toString();
        const bool first = tree_->topLevelItemCount() == 0;
        tree_->clear();
        for (size_t i = 0; i < list.size(); i++) {
            QTreeWidgetItem* it = new QTreeWidgetItem(tree_);
            const QString name = simulqt::ToQ(list[i].name);
            it->setData(0, kNameRole, name);
            for (size_t k = 0; k < list[i].joints.size(); k++) {
                QTreeWidgetItem* c = new QTreeWidgetItem(it);
                c->setData(0, kNameRole, simulqt::ToQ(list[i].joints[k].name));
            }
            it->setExpanded(first || open.contains(name));
        }
        needResize = true;
    }
    for (size_t i = 0; i < list.size(); i++) {
        QTreeWidgetItem* it = tree_->topLevelItem(static_cast<int>(i));
        it->setText(0, simulqt::ToQ(list[i].name));
        it->setText(1, FormatPose(list[i].pose));
        it->setText(2, simulqt::ToQ(list[i].kind));
        for (size_t k = 0; k < list[i].joints.size(); k++) {
            QTreeWidgetItem* c = it->child(static_cast<int>(k));
            const JointInfo& j = list[i].joints[k];
            c->setText(0, simulqt::ToQ(j.name));
            c->setText(1, FormatValue(j));
            c->setText(2, FormatLimit(j));
        }
    }
    if (needResize) {   // 글자를 채운 뒤에 열 너비를 맞춘다
        tree_->resizeColumnToContents(0);
        tree_->resizeColumnToContents(1);
    }
}

int ObjectInfoPanel::ObjectRows() const { return tree_->topLevelItemCount(); }

QString ObjectInfoPanel::Text(int object, int joint, int column) const {
    QTreeWidgetItem* it = tree_->topLevelItem(object);
    if (!it) return QString();
    if (joint >= 0) it = it->child(joint);
    return it ? it->text(column) : QString();
}

void ObjectInfoPanel::onDoubleClicked(QTreeWidgetItem* item, int) {
    if (!item) return;
    const QString n = item->data(0, kNameRole).toString();
    if (!n.isEmpty()) emit nameActivated(QLatin1Char('"') + n + QLatin1Char('"'));
}
