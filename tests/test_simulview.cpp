// SimulView(다른 Qt 프로그램에 붙이는 3D 뷰 부품)가 main.cpp 없이 혼자 동작하는지 본다.
// 창을 띄우지 않고 (QT_QPA_PLATFORM=offscreen) 위젯 API 와 시그널만 확인한다.
#include <QMouseEvent>
#include <QApplication>
#include <QMenu>
#include <QSlider>
#include <QToolButton>
#include <stdio.h>

#include "CBase/Math.h"
#include "CoreQt/QtString.h"
#include "CoreQt/SimulView.h"
#include "CURDF/UrdfWriter.h"
#include "TestUtil.h"

static int failures = 0;
#define CHECK(c, msg) do { if (!(c)) { fprintf(stderr, "FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); failures++; } } while (0)

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    const QString arm = QString(SIMUL_TEST_DATA_DIR) + "/Object/samples/arm.urdf";
    const QString room = QString(SIMUL_TEST_DATA_DIR) + "/Object/samples/lidar_room.urdf";

    SimulView view;   // 호스트 없이 혼자 만든다 (기본: 모든 기능)
    CHECK(view.HasFeature(SimulView::kSelectionPanel) && view.HasFeature(SimulView::kMouseEdit), "default features");
    CHECK(view.selectionPanel()->isHidden(), "panel hidden at start");

    ModelViewer* v = view.viewer();
    int robotSignals = 0;
    QObject::connect(v, &ModelViewer::robotChanged, [&]() { robotSignals++; });
    QString err;
    CHECK(v->AddFile(arm, &err), "add arm");
    CHECK(v->AddFile(room, &err), "add room");
    CHECK(v->ModelCount() == 2, "two models");
    CHECK(robotSignals >= 2, "robotChanged emitted");

    // 모델의 루트 링크를 고르면 패널에 그 모델의 관절만 나온다 (arm 은 4개)
    const int armRoot = v->Robot().FindLink(simul::String("base_link"));
    v->SetSelectedLink(armRoot);
    CHECK(!view.selectionPanel()->isHidden(), "panel shown after selection");
    CHECK(view.selectionPanel()->findChildren<QSlider*>().size() == 4, "root panel lists the model's 4 joints");

    // 중간 링크를 고르면 그 링크의 관절 하나
    v->SetSelectedLink(v->Robot().FindLink(simul::String("forearm")));
    CHECK(view.selectionPanel()->findChildren<QSlider*>().size() == 1, "forearm panel lists 1 joint");

    // 슬라이더를 움직이면 jointEdited 가 나오고 값은 viewer 에 바로 반영된다
    QString editedName;
    double editedValue = 0;
    QObject::connect(&view, &SimulView::jointEdited, [&](const QString& n, double x) { editedName = n; editedValue = x; });
    QSlider* s = view.selectionPanel()->findChildren<QSlider*>().first();
    s->setValue(s->maximum());
    CHECK(editedName == "elbow_pitch", "jointEdited name");
    simul::urdf::JointValues q = v->CurrentJointValues();
    CHECK(simul::Abs(q["elbow_pitch"] - static_cast<float>(editedValue)) < 1e-5f && editedValue > 0.5, "joint value applied to the viewer");

    // 기능을 끈다
    view.EnableFeature(SimulView::kSelectionPanel, false);
    CHECK(view.selectionPanel()->isHidden(), "panel hidden when feature is off");
    view.EnableFeature(SimulView::kMouseEdit, false);
    CHECK(!v->MouseEditEnabled(), "mouse edit off");
    view.SetFeatures(SimulView::kAll);
    CHECK(v->MouseEditEnabled() && !view.selectionPanel()->isHidden(), "features restored, panel back");

    // 선택을 풀면 패널이 사라진다
    v->SetSelectedLink(-1);
    CHECK(view.selectionPanel()->isHidden(), "panel hidden without selection");

    // 3D 화면 안의 메뉴줄: 기본 메뉴 3개 (파일/보기/물리), 호스트 메뉴로 바꾸기, 하나 더하기, 끄기
    {
        QFrame* bar = view.findChild<QFrame*>("simulMenuBar");
        CHECK(bar && !bar->isHidden(), "menu bar shown by default (kAll)");
        CHECK(bar->findChildren<QToolButton*>().size() == 3, "3 built-in menus");
        QMenu host("호스트");
        QAction* ran = host.addAction("실행");
        bool hit = false;
        QObject::connect(ran, &QAction::triggered, [&]() { hit = true; });
        view.SetMenus(QList<QMenu*>() << &host);
        CHECK(bar->findChildren<QToolButton*>().size() == 1, "host menu replaces the built-in ones");
        ran->trigger();
        CHECK(hit, "host action works through the in-view menu");
        view.ResetMenus();
        QMenu* extra = view.AddMenu("추가");
        extra->addAction("x");
        CHECK(bar->findChildren<QToolButton*>().size() == 4, "AddMenu appends one");
        view.EnableFeature(SimulView::kMenu, false);
        CHECK(bar->isHidden(), "menu bar hidden when feature is off");
        view.EnableFeature(SimulView::kMenu, true);
        // 기본 메뉴의 "모두 닫기" 가 장면을 비운다
        QList<QAction*> acts = view.findChildren<QAction*>();
        QAction* closeAll = nullptr;
        for (int i = 0; i < acts.size(); i++) if (acts[i]->text() == "모두 닫기") closeAll = acts[i];
        CHECK(closeAll != nullptr, "found 모두 닫기");
        if (closeAll) { CHECK(v->ModelCount() == 2, "still two models"); closeAll->trigger(); CHECK(v->ModelCount() == 0, "menu closes all"); }
    }

    // 구조 트리 창: 모델이 있으면 보이고, 함수로 끄고 켠다
    {
        CHECK(v->AddFile(arm, &err) && v->AddFile(room, &err), "re-add models");
        // 장면 -> URDF 하나로 저장 -> 다시 읽기
        {
            const QString out = QString::fromUtf8(tu::Tmp("simul_view_scene.urdf").c_str());
            QString serr;
            CHECK(v->SaveUrdf(out, &serr), "scene saved as URDF");
            simul::urdf::Robot back;
            simul::String perr;
            CHECK(simul::urdf::LoadUrdf(simulqt::FromQ(out), &back, &perr), "saved scene reloads");
            CHECK(back.links.size() == v->Robot().links.size() && back.joints.size() == v->Robot().joints.size(), "saved scene has the same structure");
        }
        view.show();
        view.resize(900, 600);
        QApplication::processEvents();
        view.SetTreeVisible(true);
        QApplication::processEvents();
        CHECK(view.tree()->topLevelItemCount() > 0, "tree filled");
        CHECK(view.treePanel()->isVisibleTo(view.viewer()), "tree panel visible");
        unsigned lastFeatures = 0;
        QObject::connect(&view, &SimulView::featuresChanged, [&](unsigned f) { lastFeatures = f; });
        view.SetTreeVisible(false);
        CHECK(!view.TreeVisible() && !view.treePanel()->isVisibleTo(view.viewer()), "tree off by function");
        CHECK(!(lastFeatures & SimulView::kTree), "featuresChanged on tree off");
        view.SetTreeVisible(true);
        CHECK(view.treePanel()->isVisibleTo(view.viewer()), "tree on again");
        // 오른쪽 아래 손잡이를 끌면 트리 창만 커지고, 호스트 창(메인 윈도우) 크기는 그대로다
        {
            QWidget* grip = nullptr;
            const QList<QWidget*> kids = view.treePanel()->findChildren<QWidget*>();
            for (int i = 0; i < kids.size(); i++) if (kids[i]->cursor().shape() == Qt::SizeFDiagCursor) grip = kids[i];
            CHECK(grip != nullptr, "tree grip exists");
            if (grip) {
                const QSize hostBefore = view.size(), panelBefore = view.treePanel()->size();
                const QPoint c = grip->rect().center();
                const QPoint g0 = grip->mapToGlobal(c);
                QMouseEvent press(QEvent::MouseButtonPress, c, g0, Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
                QApplication::sendEvent(grip, &press);
                QMouseEvent move(QEvent::MouseMove, c, g0 + QPoint(40, 30), Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
                QApplication::sendEvent(grip, &move);
                QMouseEvent rel(QEvent::MouseButtonRelease, c, g0 + QPoint(40, 30), Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
                QApplication::sendEvent(grip, &rel);
                CHECK(view.treePanel()->size() != panelBefore, "panel resized by grip");
                CHECK(view.size() == hostBefore, "host size unchanged");
            }
        }
        // UI 크기 배율: 선택 패널 폭과 트리 창이 함께 커지고 작아진다 (범위 0.6~2.5)
        {
            const int w0 = view.selectionPanel()->width(), tw0 = view.treePanel()->width();
            double got = 0;
            QObject::connect(&view, &SimulView::uiScaleChanged, [&](double s) { got = s; });
            view.SetUiScale(1.5);
            CHECK(view.UiScale() == 1.5 && got == 1.5, "ui scale set");
            CHECK(view.selectionPanel()->width() > w0, "selection panel grows");
            CHECK(view.treePanel()->width() > tw0, "tree panel grows");
            view.SetUiScale(10.0);
            CHECK(view.UiScale() == 2.5, "ui scale clamped high");
            view.SetUiScale(0.1);
            CHECK(view.UiScale() == 0.6, "ui scale clamped low");
            view.SetUiScale(1.0);
            CHECK(view.selectionPanel()->width() == w0, "back to default width");
        }
        // X 버튼으로 닫으면 기능도 꺼진다
        emit view.treePanel()->closeRequested();
        CHECK(!view.TreeVisible(), "close button turns feature off");
        view.SetTreeVisible(true);
        // 트리에서 링크를 고르면 뷰의 선택이 바뀐다
        emit view.tree()->linkSelected(armRoot);
        CHECK(v->SelectedLink() == armRoot, "tree selection reaches viewer");
        // 속성 창: 선택하면 보이고, 숫자를 고치면 모델 배치가 바뀐다. 함수로 끄고 켠다
        {
            view.SetPropertiesVisible(true);
            v->SetSelectedLink(armRoot);
            QApplication::processEvents();
            CHECK(view.propertiesPanel()->isVisibleTo(view.viewer()), "properties panel visible when a model is selected");
            QList<QDoubleSpinBox*> spins = view.propertiesPanel()->findChildren<QDoubleSpinBox*>();
            CHECK(spins.size() == 6, "properties has 6 fields");
            if (spins.size() == 6) {
                spins[0]->setValue(1.5);   // 위치 x
                spins[5]->setValue(30.0);  // 회전 yaw
                float pl[6];
                v->ModelPlacement6(v->ModelOfLink(armRoot), pl);
                CHECK(simul::Abs(pl[0] - 1.5f) < 1e-3f && simul::Abs(pl[5] - 30.0f) < 0.05f, "typing a value moves/rotates the model");
            }
            view.SetPropertiesVisible(false);
            CHECK(!view.propertiesPanel()->isVisibleTo(view.viewer()), "properties off by function");
            view.SetPropertiesVisible(true);
        }
        view.SetMenuVisible(false);
        view.SetSelectionPanelVisible(false);
        CHECK(!view.MenuVisible() && !view.SelectionPanelVisible(), "menu/panel off by function");
        view.SetMenuVisible(true);
        view.SetSelectionPanelVisible(true);
    }

    // 모두 닫기
    v->CloseAll();
    CHECK(view.tree()->topLevelItemCount() == 0 && !view.treePanel()->isVisibleTo(view.viewer()), "tree hides when empty");
    CHECK(v->ModelCount() == 0, "closed all");

    if (failures) { fprintf(stderr, "test_simulview: %d failed\n", failures); return 1; }
    printf("test_simulview: all passed\n");
    return 0;
}
