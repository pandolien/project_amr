// 다른 Qt 프로그램이 3D 뷰를 붙이는 가장 작은 예. 메뉴, 도크, 로그는 호스트(이 프로그램)가 원하는 대로 만든다.
//   실행:  embed_host [파일.urdf ...]   (인자가 없으면 빈 화면, 파일을 끌어다 놓으면 추가)
#include <QApplication>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <QWidget>

#include "CoreQt/SimulView.h"

int main(int argc, char** argv) {
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);
    QApplication app(argc, argv);

    QWidget host;
    host.setWindowTitle("embed_host");
    QVBoxLayout* col = new QVBoxLayout(&host);

    // 호스트가 고른 기능만 켠다: 여기서는 선택 패널만 켜고, 마우스 편집과 파일 끌어놓기는 체크박스로 고른다
    SimulView* view = new SimulView(&host, SimulView::kSelectionPanel);
    col->addWidget(view, 1);

    QHBoxLayout* bar = new QHBoxLayout;
    QCheckBox* edit = new QCheckBox("마우스로 이동/회전");
    QCheckBox* panel = new QCheckBox("선택 패널");
    panel->setChecked(true);
    QLabel* status = new QLabel("모델 0개");
    QPushButton* clear = new QPushButton("모두 닫기");
    bar->addWidget(edit);
    bar->addWidget(panel);
    bar->addWidget(status, 1);
    bar->addWidget(clear);
    col->addLayout(bar);

    QObject::connect(edit, &QCheckBox::toggled, [view](bool on) { view->EnableFeature(SimulView::kMouseEdit, on); view->EnableFeature(SimulView::kDropFiles, on); });
    QObject::connect(panel, &QCheckBox::toggled, [view](bool on) { view->EnableFeature(SimulView::kSelectionPanel, on); });
    QObject::connect(clear, &QPushButton::clicked, [view]() { view->viewer()->CloseAll(); });
    // 뷰가 알려 주는 이벤트로 호스트의 UI 를 갱신한다
    auto refresh = [view, status]() { status->setText(QString("모델 %1개").arg(view->viewer()->ModelCount())); };
    QObject::connect(view->viewer(), &ModelViewer::robotChanged, refresh);
    QObject::connect(view->viewer(), &ModelViewer::modelDragFinished, [status](int model, bool rotated) { status->setText(QString("모델 %1 %2 끝").arg(model).arg(rotated ? "회전" : "이동")); });

    for (int i = 1; i < argc; i++) {
        QString err;
        if (!view->viewer()->AddFile(QString::fromLocal8Bit(argv[i]), &err)) status->setText(err);
    }
    refresh();
    host.resize(900, 650);
    host.show();
    return app.exec();
}
