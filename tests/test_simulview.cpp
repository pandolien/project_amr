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
#include "CoreQt/QtString.h"
#include "CoreQt/PropertiesPanel.h"
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
        CHECK(bar->findChildren<QToolButton*>().size() == 4, "4 built-in menus (file, view, add, physics)");
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
        CHECK(bar->findChildren<QToolButton*>().size() == 5, "AddMenu appends one");
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
        // 기본 도형 추가: 박스/원통/구가 모델로 올라가고, 위치와 이름이 지정되며, 잘못된 크기는 거절된다
        {
            const int before = v->ModelCount();
            const float pose[6] = {2.0f, 1.0f, 0.5f, 0, 0, 90.0f};
            QString aerr;
            const int box = v->AddBox("crate", 0.4, 0.6, 0.8, pose, QColor(200, 100, 50), 3.0, &aerr);
            CHECK(box == before && v->ModelCount() == before + 1, "box added as a model");
            float got[6];
            v->ModelPlacement6(box, got);
            CHECK(simul::Abs(got[0] - 2.0f) < 1e-3f && simul::Abs(got[1] - 1.0f) < 1e-3f && simul::Abs(got[5] - 90.0f) < 0.1f, "box placed at the given pose");
            CHECK(v->ModelName(box) == "crate", "box keeps its name");
            const int cyl = v->AddCylinder("pipe", 0.1, 1.2, nullptr, QColor(), 2.0, &aerr);
            const int sph = v->AddSphere("ball", 0.2, nullptr, QColor(), 1.0, &aerr);
            CHECK(cyl >= 0 && sph >= 0 && v->ModelCount() == before + 3, "cylinder and sphere added");
            const int dup = v->AddBox("crate", 1, 1, 1, nullptr, QColor(), 1.0, &aerr);
            CHECK(dup >= 0 && v->ModelName(dup) != "crate", "same name gets a unique one");
            CHECK(v->AddBox("bad", 0.0, 1, 1, nullptr, QColor(), 1.0, &aerr) < 0 && !aerr.isEmpty(), "zero-size box is rejected with a message");
            CHECK(v->ModelCount() == before + 4, "rejected shape leaves the scene unchanged");
            v->RemoveModel(dup);
            v->RemoveModel(sph);
            v->RemoveModel(cyl);
            v->RemoveModel(box);
        }
        // 입력 창으로 올리기: 종류별 입력 칸, z 자동 높이, 입력한 값 그대로 반영
        {
            const int before = v->ModelCount();
            const double at[3] = {1.5, -0.5, 0.0};
            ShapeDialog dlg(nullptr, ShapeDialog::kCylinder, at);
            CHECK(dlg.CurrentShape() == ShapeDialog::kCylinder && dlg.Name() == "cylinder", "dialog starts with the requested shape");
            CHECK(simul::Abs(static_cast<float>(dlg.Position(0)) - 1.5f) < 1e-6f && simul::Abs(static_cast<float>(dlg.Position(2)) - 0.25f) < 1e-6f, "dialog z rests on the floor");
            dlg.SetSize(0, 0.1);
            dlg.SetSize(1, 2.0);
            CHECK(simul::Abs(static_cast<float>(dlg.Position(2)) - 1.0f) < 1e-6f, "z follows the length while untouched");
            dlg.SetName("pole");
            dlg.SetMass(7.5);
            dlg.SetColor(QColor(10, 200, 30));
            QString derr;
            const int idx = dlg.Apply(v, &derr);
            CHECK(idx == before && v->ModelCount() == before + 1 && v->ModelName(idx) == "pole", "dialog adds the shape");
            float got[6];
            v->ModelPlacement6(idx, got);
            CHECK(simul::Abs(got[0] - 1.5f) < 1e-3f && simul::Abs(got[1] + 0.5f) < 1e-3f && simul::Abs(got[2] - 1.0f) < 1e-3f, "dialog position applied");
            dlg.SetShape(ShapeDialog::kSphere);
            dlg.SetSize(0, 0.0);
            CHECK(dlg.Size(0) > 0.0, "dialog fields cannot hold a zero size");
            const int ball = dlg.Apply(v, &derr);
            CHECK(ball == before + 1, "dialog adds a sphere too");
            v->RemoveModel(ball);
            v->RemoveModel(idx);
        }
        // 오른쪽 클릭 신호를 받는 호스트가 있으면 기본 메뉴 대신 호스트가 처리한다
        {
            int gotHits = 0;
            double gx = 0, gy = 0;
            QMetaObject::Connection c = QObject::connect(&view, &SimulView::viewContextMenuRequested, [&](const QPoint&, double x, double y) { gotHits++; gx = x; gy = y; });
            emit v->viewContextMenuRequested(QPoint(0, 0), 1.25, -2.0);
            CHECK(gotHits == 1 && simul::Abs(static_cast<float>(gx) - 1.25f) < 1e-6f && simul::Abs(static_cast<float>(gy) + 2.0f) < 1e-6f, "host receives the right-click position");
            QObject::disconnect(c);
        }
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
            const int box = v->AddBox("qbox", 0.2, 0.2, 0.2, nullptr, QColor(), 1.0, nullptr);
            v->SetModelPlacement(box, pose);
            const dhPose back = v->ModelPlacementPose(box);
            CHECK(simul::Abs(back.position.x - 0.5f) < 1e-4f && simul::Abs(back.position.y + 0.25f) < 1e-4f, "quaternion placement keeps the position (m)");
            const dhVector a = pose * dhVector(1, 0, 0), b = back * dhVector(1, 0, 0);
            CHECK(simul::Abs(a.x - b.x) < 1e-4f && simul::Abs(a.y - b.y) < 1e-4f && simul::Abs(a.z - b.z) < 1e-4f, "quaternion placement keeps the orientation");
            v->RemoveModel(box);
            CHECK(v->ModelCount() == before, "quaternion test model removed");
        }
        // 바닥 모델: 지정한 모델의 윗면(평면, 곡면)에 닿으면 멈춘다. 지정하지 않은 자리는 z=0 평면
        {
            auto writeUrdf = [](const char* name, const char* geometry) {
                const QString path = QString::fromUtf8(tu::Tmp(name).c_str());
                QFile f(path);
                f.open(QIODevice::WriteOnly);
                f.write(QByteArray("<robot name=\"floor_part\"><link name=\"floor_part_link\"><visual><geometry>") + geometry +
                        "</geometry></visual></link></robot>");
                f.close();
                return path;
            };
            const int before = v->ModelCount();
            QString ferr;
            const QString table = writeUrdf("simul_floor_table.urdf", "<box size=\"1 1 1\"/>");   // 윗면 z = 0.5 (중심이 원점)
            CHECK(v->AddFile(table, &ferr), "add the floor model");
            const int fl = before;
            CHECK(v->AddFile(arm, &ferr), "add a falling model");
            const int item = before + 1;
            v->SetModelPlacement(fl, dhPose(dhVector(0, 0, 0), simul::units::QuatFromRpyDeg(simul::units::RpyDeg(0, 0, 0))));
            v->SetModelPlacement(item, dhPose(dhVector(0, 0, 3), simul::units::QuatFromRpyDeg(simul::units::RpyDeg(0, 0, 0))));
            v->SetFloorModel(fl);
            CHECK(v->FloorModel() == fl, "floor model is set");
            double z = 0.0;
            CHECK(v->FloorHeightAt(0.2, -0.3, &z) && simul::Abs(static_cast<float>(z) - 0.5f) < 1e-4f, "floor top is 0.5 m");
            CHECK(!v->FloorHeightAt(2.0, 0.0, &z), "no floor surface outside the floor model");
            v->SimulateFall(20.0);
            const float restZ = v->ModelPlacementPose(item).position.z;   // 팔의 바닥면이 모델 원점이라 원점 높이 = 윗면 높이
            CHECK(simul::Abs(restZ - 0.5f) < 2e-3f, "the model rests on the floor model's top (0.5 m), not on z = 0");
            // 바닥 모델 바깥으로 옮기면 z=0 평면까지 내려간다
            v->SetModelPlacement(item, dhPose(dhVector(3, 0, 1), simul::units::QuatFromRpyDeg(simul::units::RpyDeg(0, 0, 0))));
            v->SimulateFall(20.0);
            CHECK(simul::Abs(v->ModelPlacementPose(item).position.z) < 2e-3f, "beside the floor model it falls to the z = 0 plane");
            v->SetFloorModel(-1);
            CHECK(v->FloorModel() < 0, "floor model cleared");
            v->RemoveModel(item);
            v->RemoveModel(fl);

            // 곡면: 반지름 1 m 구의 꼭대기 위로 떨어뜨린다. 지정한 모델이 아닌 곳은 영향이 없다.
            const QString ball = writeUrdf("simul_floor_ball.urdf", "<sphere radius=\"1\"/>");
            CHECK(v->AddFile(ball, &ferr) && v->AddFile(arm, &ferr), "add a curved floor model and a falling model");
            const int b = before, it = before + 1;
            v->SetModelPlacement(it, dhPose(dhVector(0, 0, 4), simul::units::QuatFromRpyDeg(simul::units::RpyDeg(0, 0, 0))));
            v->SetFloorModel(b);
            v->SimulateFall(20.0);
            const float zc = v->ModelPlacementPose(it).position.z;
            // 상자 바닥 모서리(0.25, 0.25)가 구 표면(높이 sqrt(1 - 0.125) = 0.935)에 닿는다. 구는 조각으로 근사하므로 약간 낮을 수 있다
            CHECK(zc > 0.88f && zc < 1.0f, "the model stops where its corners touch the curved top");
            v->SetFloorModel(-1);
            v->RemoveModel(it);
            v->RemoveModel(b);
            CHECK(v->ModelCount() == before, "floor test models removed");
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
        // 크기 배율: 기본 도형도 된다 (질량 s^3). 속성 위젯의 크기 칸이 엔진을 제어한다.
        {
            const int sbox = v->AddBox("scalebox", 0.4, 0.4, 0.4, nullptr, QColor(), 8.0, nullptr);
            CHECK(sbox >= 0 && simul::Abs(static_cast<float>(v->ModelScale(sbox)) - 1.0f) < 1e-6f, "scale starts at 1");
            CHECK(v->SetModelScale(sbox, 2.0) && simul::Abs(static_cast<float>(v->ModelScale(sbox)) - 2.0f) < 1e-6f, "scale set to 2");
            float massSum = 0;
            for (size_t i = 0; i < v->Robot().links.size(); i++)
                if (v->Robot().links[i].name == "scalebox" || v->Robot().links[i].name.find("scalebox") != simul::String::npos) massSum += v->Robot().links[i].inertial.mass;
            CHECK(simul::Abs(massSum - 64.0f) < 1e-2f, "mass is 8 kg x 2^3 after doubling");
            {
                PropertiesPanel ext(v);
                v->SetSelectedLink(v->Robot().FindLink("scalebox"));
                CHECK(ext.Model() == sbox && simul::Abs(static_cast<float>(ext.Field(6)->value()) - 2.0f) < 1e-4f, "panel shows the model scale");
                ext.Field(6)->setValue(0.5);
                CHECK(simul::Abs(static_cast<float>(v->ModelScale(sbox)) - 0.5f) < 1e-4f, "scale field controls the engine");
            }
            CHECK(!v->SetModelScale(sbox, 0.0) && !v->SetModelScale(99, 2.0), "bad scale or model index rejected");
            v->RemoveModel(sbox);
        }
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
        // 엔진 밖의 속성 위젯: 엔진 함수/신호만으로 선택을 따라가고, 숫자를 고치면 엔진을 제어한다
        {
            PropertiesPanel ext(v);   // 3D 화면(view) 안이 아니라 독립 위젯이다
            v->SetSelectedLink(armRoot);
            QApplication::processEvents();
            CHECK(ext.Model() == v->ModelOfLink(armRoot), "external panel follows the engine selection");
            ext.Field(0)->setValue(1.5);   // 위치 x
            ext.Field(5)->setValue(30.0);  // 회전 yaw
            float pl[6];
            v->ModelPlacement6(v->ModelOfLink(armRoot), pl);
            CHECK(simul::Abs(pl[0] - 1.5f) < 1e-3f && simul::Abs(pl[5] - 30.0f) < 0.05f, "external panel controls the engine");
            CHECK(!ext.isAncestorOf(view.viewer()) && view.viewer()->findChild<PropertiesPanel*>() == nullptr, "panel is not inside the 3D view");
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
