// SimulView(다른 Qt 프로그램에 붙이는 3D 뷰 부품)가 main.cpp 없이 혼자 동작하는지 본다.
// 창을 띄우지 않고 (QT_QPA_PLATFORM=offscreen) 위젯 API 와 시그널만 확인한다.
#include <QApplication>
#include <QSlider>
#include <stdio.h>

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

    // 모두 닫기
    v->CloseAll();
    CHECK(v->ModelCount() == 0, "closed all");

    if (failures) { fprintf(stderr, "test_simulview: %d failed\n", failures); return 1; }
    printf("test_simulview: all passed\n");
    return 0;
}
