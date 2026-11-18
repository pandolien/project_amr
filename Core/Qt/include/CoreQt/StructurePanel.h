#ifndef CORECQT_STRUCTUREPANEL_H
#define CORECQT_STRUCTUREPANEL_H

#include <QString>
#include <QTreeWidget>
#include "CBase/Map.h"
#include "CBase/String.h"
#include "CBase/Util.h"
#include "CBase/Array.h"

#include "CMesh/Mesh.h"
#include "CURDF/Urdf.h"

// 오브젝트 구조를 트리로 보여준다.
//   URDF  : 루트 링크부터 관절을 따라 자식 링크가 이어지는 계층 (열: 링크 / 관절 / 부가 정보 / 종류, 비고)
//   정적 모델 : 장면의 노드 계층 (열: 노드 / 메시 / 비고)
// 링크를 선택하면 linkSelected(링크 인덱스)를 내보낸다 (ModelViewer::SetSelectedLink 에 연결).
class StructurePanel : public QTreeWidget {
    Q_OBJECT
public:
    explicit StructurePanel(QWidget* parent = nullptr);

    void SetRobot(const simul::urdf::Robot& robot);
    void SetScene(const simul::Scene& scene);
    void Clear();

    // 충돌 중인 링크를 빨갛게 표시한다 (링크 인덱스 목록)
    void SetCollidingLinks(const simul::Array<int>& links);
    // 어두운 3D 화면 위에 띄울 때 쓰는 모양: 헤더/줄무늬 없이, 관절 종류별 색 점, 보조 열은 흐리게
    void SetDarkTheme(bool on);
    // 관절별 부가 정보(예: 중력 하중)를 3번째 열에 보여준다. 관절 이름 -> (표시 문자열, 강조 여부).
    // 강조(true)이면 빨갛게 표시한다. 목록에 없는 관절은 비운다.
    void SetJointNotes(const simul::Map<simul::String, simul::Pair<QString, bool> >& notes);
    // 코드에서 링크를 선택한다 (linkSelected 는 내보내지 않는다). -1 이면 선택 해제.
    void SelectLink(int link);

signals:
    void linkSelected(int link);  // 링크가 아닌 항목이나 선택 해제는 -1
    void linkContextMenuRequested(int link, const QPoint& globalPos);  // 링크 항목에서 마우스 오른쪽 버튼 (링크가 아닌 곳은 -1)

private:
    void onCurrentChanged(QTreeWidgetItem* cur);
    void addLink(QTreeWidgetItem* parent, const simul::urdf::Robot& robot, int link, int viaJoint,
                 const simul::Array<simul::Array<int> >& childJoints);
    void addNode(QTreeWidgetItem* parent, const simul::Scene& scene, int node);

    bool dark_ = false;
    simul::Array<QTreeWidgetItem*> linkItems_;  // 링크 인덱스 -> 항목
};

#endif  // CORECQT_STRUCTUREPANEL_H
