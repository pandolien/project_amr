#ifndef CORECQT_SIMULVIEW_H
#define CORECQT_SIMULVIEW_H

#include <QFrame>
#include <QList>
#include <QMenu>
#include <QStringList>
#include <QTimer>
#include <QWidget>

#include "CoreQt/ModelViewer.h"
#include "CoreQt/FloatingPanel.h"
#include "CoreQt/SelectionOverlay.h"
#include "CoreQt/StructurePanel.h"

// 다른 Qt 프로그램(호스트)에 붙여 쓰는 3D 뷰 부품.
//   - ModelViewer(3D 화면 + 조작)와, 클릭한 물체 옆에 뜨는 선택 패널을 하나로 묶었다.
//   - 호스트가 Features 로 무엇을 켤지 고른다. 메뉴, 도크, 로그 같은 바깥 UI 는 전혀 만들지 않는다 (필요하면 호스트가 viewer() 의 시그널을 받아 만든다).
//   - 모든 기능은 viewer() 로 접근한다 (모델 추가/제거, 위치/자세, 관절 값, 카메라, 표시 옵션, 시그널 전부).
//
//   최소 사용:   SimulView* v = new SimulView(parent);  v->viewer()->AddFile("arm.urdf");
class SimulView : public QWidget {
    Q_OBJECT
public:
    enum Feature {
        kSelectionPanel = 1,   // 클릭한 물체 옆의 정보 + 관절 슬라이더 패널
        kMouseEdit = 2,        // 좌표계 화살표 이동, 회전 링, IK 끌기 (끄면 보기와 선택만)
        kDropFiles = 4,        // 파일을 끌어다 놓아 추가
        kMenu = 8,             // 3D 화면 위쪽에 뜨는 메뉴줄 (파일/보기/물리). 호스트의 메뉴를 넣으려면 SetMenus
        kTree = 16,            // 3D 화면 안에 뜨는 구조 트리 창 (링크 계층, 클릭하면 선택, 충돌 링크는 빨갛게). 비어 있으면 저절로 숨는다
        kAll = kSelectionPanel | kMouseEdit | kDropFiles | kMenu | kTree
    };

    explicit SimulView(QWidget* parent = nullptr, unsigned features = kAll);

    ModelViewer* viewer() const { return viewer_; }
    SelectionOverlay* selectionPanel() const { return overlay_; }

    // 3D 화면 안의 메뉴줄 (kMenu). 기본으로 파일/보기/물리 메뉴가 들어 있다.
    // 호스트가 자기 메뉴를 보이고 싶으면 SetMenus 로 통째로 바꾸고(소유권은 호스트), AddMenu 로 하나만 더할 수도 있다.
    void SetMenus(const QList<QMenu*>& menus);
    QMenu* AddMenu(const QString& title);   // 이 부품이 소유하는 메뉴를 하나 더한다
    void ResetMenus();                       // 기본 메뉴로 되돌린다

    // 3D 화면 안에 들어 있는 UI 를 프로그램에서 켜고 끄는 함수. EnableFeature 와 같은 일을 이름으로 부른다.
    void SetMenuVisible(bool on) { EnableFeature(kMenu, on); }
    bool MenuVisible() const { return HasFeature(kMenu); }
    void SetTreeVisible(bool on) { EnableFeature(kTree, on); }
    bool TreeVisible() const { return HasFeature(kTree); }
    void SetSelectionPanelVisible(bool on) { EnableFeature(kSelectionPanel, on); }
    bool SelectionPanelVisible() const { return HasFeature(kSelectionPanel); }
    // 3D 화면 안 UI(메뉴줄, 구조 트리, 선택 패널)의 크기 배율. 1.0 이 기본이고 0.6~2.5 로 제한한다.
    void SetUiScale(double scale);
    double UiScale() const { return uiScale_; }
    StructurePanel* tree() const { return tree_; }              // 구조 트리 (호스트가 SetJointNotes 같은 걸 직접 쓸 수 있다)
    FloatingPanel* treePanel() const { return treePanel_; }

    unsigned Features() const { return features_; }
    void SetFeatures(unsigned features);
    void EnableFeature(Feature f, bool on) { SetFeatures(on ? (features_ | f) : (features_ & ~static_cast<unsigned>(f))); }
    bool HasFeature(Feature f) const { return (features_ & f) != 0; }

signals:
    void uiScaleChanged(double scale);
    void featuresChanged(unsigned features);   // 켜진 기능이 바뀌었다 (X 로 창을 닫은 경우 포함): 호스트의 체크 표시를 맞출 때 쓴다
    // 구조 트리에서 링크를 오른쪽 클릭했다 (link -1 은 링크가 아닌 곳). 받는 쪽이 없으면 모델의 루트 링크에서 "제거" 메뉴만 보인다.
    void linkContextMenuRequested(int link, const QPoint& globalPos);
    // 선택 패널의 슬라이더로 관절 값을 바꿨다 (값은 이미 viewer() 에 반영되었다. 호스트의 다른 UI 를 맞출 때 쓴다)
    void jointEdited(const QString& jointName, double value);
    // 파일을 더하려 한다 (kDropFiles 로 놓았거나, 메뉴의 열기로 골랐다). 받는 쪽이 없으면 viewer()->AddFile 로 바로 추가한다.
    void filesDropped(const QStringList& paths);

protected:
    void dragEnterEvent(QDragEnterEvent* e) override;
    void dropEvent(QDropEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;

private slots:
    void buildMenuBar();
    void refreshTree();    // 로봇/장면이 바뀌었을 때 트리를 다시 채운다
    void refreshPanel();   // 선택이 바뀌었을 때: 내용을 새로 채운다
    void syncPanel();      // 같은 선택에서 자세/관절 값이 바뀌었을 때: 숫자와 슬라이더만 갱신
    void placePanel();     // 카메라가 움직여도 물체 옆을 따라다닌다

private:
    QString panelInfo(int link) const;
    QList<QMenu*> defaultMenus();
    void layoutMenuBar();
    void updateTreeVisibility();
    void applyMenus();

    ModelViewer* viewer_;
    SelectionOverlay* overlay_;
    StructurePanel* tree_;
    FloatingPanel* treePanel_;
    bool treePlaced_;
    double uiScale_;
    void applyMenuBarStyle();
    QFrame* menuBar_;
    QList<QMenu*> menus_;          // 지금 메뉴줄에 보이는 메뉴
    QList<QMenu*> ownedMenus_;     // 이 부품이 만든 것 (호스트가 SetMenus 로 바꾸면 쓰지 않는다)
    bool customMenus_;
    QTimer* placeTimer_;
    unsigned features_;
    int panelLink_;   // 패널이 지금 보여주는 링크 (-1 이면 없음)
};

#endif  // CORECQT_SIMULVIEW_H
