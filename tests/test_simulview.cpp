// SimulView(다른 Qt 프로그램에 붙이는 3D 뷰 부품)가 main.cpp 없이 혼자 동작하는지 본다.
// 창을 띄우지 않고 (QT_QPA_PLATFORM=offscreen) 위젯 API 와 시그널만 확인한다.
#include <QMouseEvent>
#include <QApplication>
#include <QFile>
#include <QMenu>
#include <QSlider>
#include <QToolButton>
#include <stdio.h>

#include "3d/Units.h"
#include "3d/dhPose.h"
#include "CBase/Math.h"
#include "CoreQt/SimulView.h"

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
        // 카메라 (m, radian, 쿼터니언)와 위치+쿼터니언 배치
        {
            v->SetCamera(dhVector(1, 2, 0.5), 4.0, 0.7, 0.3);
            CHECK(simul::Abs(static_cast<float>(v->CameraDistance()) - 4.0f) < 1e-5f && simul::Abs(static_cast<float>(v->CameraYaw()) - 0.7f) < 1e-5f && simul::Abs(static_cast<float>(v->CameraPitch()) - 0.3f) < 1e-5f, "camera getters return what was set (radian)");
            const dhPose cam = v->CameraPose();
            const dhVector fwd = cam * dhVector(1, 0, 0) - cam.position;   // 카메라 앞(x)이 가리키는 방향
            const dhVector toTarget(1.0f - cam.position.x, 2.0f - cam.position.y, 0.5f - cam.position.z);
            const float tl = simul::Sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y + toTarget.z * toTarget.z);
            CHECK(simul::Abs(tl - 4.0f) < 1e-3f, "camera eye is at the distance from the target");
            CHECK(simul::Abs(fwd.x - toTarget.x / tl) < 1e-3f && simul::Abs(fwd.y - toTarget.y / tl) < 1e-3f && simul::Abs(fwd.z - toTarget.z / tl) < 1e-3f, "camera x axis looks at the target");
            v->SetCamera(dhVector(0, 0, 0), 1.0, 0.0, 3.0);
            CHECK(v->CameraPitch() < 1.6, "pitch is limited below 90 degrees");
            v->FitView();
        }
        {
            const int before = v->ModelCount();
            const dhPose pose(dhVector(0.5f, -0.25f, 0.1f), simul::units::QuatFromRpyDeg(simul::units::RpyDeg(10, 20, 30)));
            simul::String qerr;
            CHECK(v->AddFile(arm, &qerr), "add a model for the quaternion placement test");
            const int box = before;
            v->SetModelPlacement(box, pose);
            const dhPose back = v->ModelPlacementPose(box);
            CHECK(simul::Abs(back.position.x - 0.5f) < 1e-4f && simul::Abs(back.position.y + 0.25f) < 1e-4f, "quaternion placement keeps the position (m)");
            const dhVector a = pose * dhVector(1, 0, 0), b = back * dhVector(1, 0, 0);
            CHECK(simul::Abs(a.x - b.x) < 1e-4f && simul::Abs(a.y - b.y) < 1e-4f && simul::Abs(a.z - b.z) < 1e-4f, "quaternion placement keeps the orientation");
            v->RemoveModel(box);
            CHECK(v->ModelCount() == before, "quaternion test model removed");
        }
        // 길이 단위 변환: mm 로 만든 파일(1000 단위 상자)을 0.001 배 하면 1 m 가 된다
        {
            const QString objPath = QString::fromUtf8(tu::Tmp("simul_mm_cube.obj").c_str());
            {
                QFile f(objPath);
                f.open(QIODevice::WriteOnly);
                f.write("v 0 0 0\nv 1000 0 0\nv 1000 1000 0\nv 0 1000 0\nv 0 0 1000\nv 1000 0 1000\nv 1000 1000 1000\nv 0 1000 1000\n"
                        "f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\nf 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n");
            }
            QString uerr;
            const int before = v->ModelCount();
            CHECK(v->AddFile(objPath, &uerr), "add a model that is 1000 units wide");
            const int mm = v->ModelCount() - 1;
            CHECK(v->ModelCount() == before + 1 && simul::Abs(static_cast<float>(v->ModelUnitScale(mm)) - 1.0f) < 1e-6f, "unit scale starts at 1");
            CHECK(v->Summary().contains(QString::fromUtf8("200 m")), "summary warns that the model is huge");
            CHECK(v->SetModelUnitScale(mm, 0.001) && simul::Abs(static_cast<float>(v->ModelUnitScale(mm)) - 0.001f) < 1e-9f, "unit scale set to 0.001");
            CHECK(v->SetModelUnitScale(mm, 0.001) && simul::Abs(static_cast<float>(v->ModelUnitScale(mm)) - 0.001f) < 1e-9f, "setting the same scale twice does not stack");
            CHECK(!v->SetModelUnitScale(mm, 0.0) && !v->SetModelUnitScale(99, 1.0), "bad scale or model index is rejected");
            v->RemoveModel(mm);
            QFile::remove(objPath);
        }
        // 크기 배율: 길이와 질량(s^3)이 함께 바뀐다
        {
            const int before = v->ModelCount();
            simul::String serr;
            CHECK(v->AddFile(arm, &serr), "add a model for the scale test");
            const int sm = before;
            auto totalMass = [&]() { float m = 0; for (size_t i = 0; i < v->Robot().links.size(); i++) if (v->ModelOfLink(static_cast<int>(i)) == sm) m += v->Robot().links[i].inertial.mass; return m; };
            const float m1 = totalMass();
            CHECK(simul::Abs(static_cast<float>(v->ModelScale(sm)) - 1.0f) < 1e-6f, "scale starts at 1");
            CHECK(v->SetModelScale(sm, 2.0) && simul::Abs(static_cast<float>(v->ModelScale(sm)) - 2.0f) < 1e-6f, "scale set to 2");
            CHECK(simul::Abs(totalMass() - m1 * 8.0f) < 1e-2f * (1.0f + m1), "mass grows by 2^3 after doubling");
            CHECK(!v->SetModelScale(sm, 0.0) && !v->SetModelScale(99, 2.0), "bad scale or model index rejected");
            v->RemoveModel(sm);
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
