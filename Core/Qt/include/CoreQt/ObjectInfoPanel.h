#ifndef CORECQT_OBJECTINFOPANEL_H
#define CORECQT_OBJECTINFOPANEL_H

#include <QWidget>

#include "CSimul/Engine.h"

class QTimer;
class QTreeWidget;
class QTreeWidgetItem;

// 장면의 오브젝트와 관절의 지금 값을 트리로 보여 준다. 값은 Backend 에서 주기적으로 읽는다 (Engine 이 없어도 된다).
//   오브젝트 행: 이름, 종류, 위치(m) 와 방향(roll pitch yaw, 도)
//   관절 행: 이름, 지금 값(rad 또는 m), 한계(최소 ~ 최대)
// 이름이나 관절 행을 더블클릭하면 nameActivated 가 온다 (편집기에 "이름" 을 넣는 데 쓴다).
class ObjectInfoPanel : public QWidget {
    Q_OBJECT
public:
    explicit ObjectInfoPanel(simul::sim::Backend* backend, QWidget* parent = nullptr);

    void SetBackend(simul::sim::Backend* backend) { backend_ = backend; Refresh(); }
    void SetIntervalMs(int ms);
    void Refresh();                       // 지금 바로 다시 읽는다
    int ObjectRows() const;               // 테스트용: 오브젝트 행 수
    QString Text(int object, int joint, int column) const;  // 테스트용: joint < 0 이면 오브젝트 행

    static QString FormatValue(const simul::sim::JointInfo& j);
    static QString FormatLimit(const simul::sim::JointInfo& j);
    static QString FormatPose(const dhPose& p);

signals:
    void nameActivated(const QString& quotedName);   // "elbow_pitch" 처럼 따옴표가 붙은 이름

protected:
    void showEvent(QShowEvent* e) override;

private slots:
    void onDoubleClicked(QTreeWidgetItem* item, int column);

private:
    bool structureChanged(const simul::Array<simul::sim::ObjectInfo>& list) const;
    simul::sim::Backend* backend_;
    QTreeWidget* tree_;
    QTimer* timer_;
};

#endif  // CORECQT_OBJECTINFOPANEL_H
