// AMR + 로봇 팔 시스템 진입점.
//   amr_main [옵션] [파일]      파일: .urdf .glb .gltf .stl .obj .dae
//   Qt 로 빌드했으면 3D 뷰어 창을 띄우고, 아니면 파일 정보만 출력한다 (--info 와 같음).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "CMesh/MeshLoader.h"
#include "CURDF/Urdf.h"

namespace {

struct Options {
    std::string file;
    bool info;                 // 창 없이 정보만 출력
    std::string shot;          // 비어 있지 않으면 화면을 PNG 로 저장하고 종료
    std::vector<std::pair<std::string, double> > joints;           // --joint 이름=값 (revolute: 도, prismatic: m)
    std::vector<std::pair<std::string, std::string> > packages;    // --package 이름=폴더
    int width, height;
    Options() : info(false), width(1100), height(720) {}
};

void PrintUsage(const char* exe) {
    std::printf(
        "사용법: %s [옵션] [파일]\n"
        "  파일               .urdf .glb .gltf .stl .obj .dae (생략하면 빈 뷰어)\n"
        "  --info             창을 띄우지 않고 파일 정보만 출력\n"
        "  --joint 이름=값    관절 값 지정 (revolute/continuous: 도, prismatic: m), 여러 번 가능\n"
        "  --package 이름=폴더 package://이름/ 을 찾을 폴더, 여러 번 가능\n"
        "  --shot 파일.png    화면을 저장하고 종료 (Qt 빌드에서만)\n"
        "  --size 가로x세로   창 크기 (기본 1100x720)\n"
        "  -h, --help         도움말\n",
        exe);
}

bool SplitPair(const std::string& s, std::string* a, std::string* b) {
    const size_t eq = s.find('=');
    if (eq == std::string::npos || eq == 0) return false;
    *a = s.substr(0, eq);
    *b = s.substr(eq + 1);
    return true;
}

// 0: 계속, 1: 정상 종료(도움말), 2: 인자 오류
int ParseArgs(int argc, char** argv, Options* o) {
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") { PrintUsage(argv[0]); return 1; }
        if (a == "--info") { o->info = true; continue; }
        const bool hasNext = i + 1 < argc;
        if (a == "--shot" && hasNext) { o->shot = argv[++i]; continue; }
        if (a == "--joint" && hasNext) {
            std::string n, v;
            if (!SplitPair(argv[++i], &n, &v)) { std::fprintf(stderr, "--joint 은 이름=값 형식입니다\n"); return 2; }
            o->joints.push_back(std::make_pair(n, std::atof(v.c_str())));
            continue;
        }
        if (a == "--package" && hasNext) {
            std::string n, v;
            if (!SplitPair(argv[++i], &n, &v)) { std::fprintf(stderr, "--package 는 이름=폴더 형식입니다\n"); return 2; }
            o->packages.push_back(std::make_pair(n, v));
            continue;
        }
        if (a == "--size" && hasNext) {
            if (std::sscanf(argv[++i], "%dx%d", &o->width, &o->height) != 2 || o->width < 100 || o->height < 100) {
                std::fprintf(stderr, "--size 는 가로x세로 형식입니다 (예: 1280x800)\n");
                return 2;
            }
            continue;
        }
        if (!a.empty() && a[0] == '-') { std::fprintf(stderr, "알 수 없는 옵션: %s (--help)\n", a.c_str()); return 2; }
        o->file = a;
    }
    return 0;
}

std::string LowerExt(const std::string& p) {
    const size_t dot = p.find_last_of('.');
    if (dot == std::string::npos) return std::string();
    std::string e = p.substr(dot + 1);
    for (size_t i = 0; i < e.size(); i++) if (e[i] >= 'A' && e[i] <= 'Z') e[i] = static_cast<char>(e[i] - 'A' + 'a');
    return e;
}

// 창 없이 파일 정보를 출력한다. 성공하면 0.
int PrintInfo(const Options& o) {
    if (o.file.empty()) { PrintUsage("amr_main"); return 2; }
    std::string err;
    if (LowerExt(o.file) == "urdf") {
        amr::urdf::Robot robot;
        if (!amr::urdf::LoadUrdf(o.file, &robot, &err)) { std::fprintf(stderr, "URDF 읽기 실패: %s\n", err.c_str()); return 1; }
        std::printf("로봇 '%s': link %zu개, joint %zu개 (구동 %zu개), 루트 link '%s'\n", robot.name.c_str(), robot.links.size(),
                    robot.joints.size(), robot.ActuatedJoints().size(), robot.links[static_cast<size_t>(robot.root)].name.c_str());
        amr::urdf::LoadOptions opt;
        for (size_t i = 0; i < o.packages.size(); i++) opt.packagePaths.push_back(o.packages[i]);
        amr::urdf::RobotMeshes meshes;
        const bool ok = amr::urdf::LoadRobotMeshes(robot, opt, &meshes);
        size_t tris = 0;
        for (size_t i = 0; i < meshes.visuals.size(); i++) tris += meshes.visuals[i].scene.TriangleCount();
        std::printf("visual %zu개 읽음, 삼각형 %zu개\n", meshes.visuals.size(), tris);
        for (size_t i = 0; i < robot.warnings.size(); i++) std::printf("경고: %s\n", robot.warnings[i].c_str());
        for (size_t i = 0; i < meshes.warnings.size(); i++) std::printf("경고: %s\n", meshes.warnings[i].c_str());
        return ok ? 0 : 1;
    }
    amr::Scene scene;
    if (!amr::LoadMesh(o.file, &scene, &err)) { std::fprintf(stderr, "읽기 실패: %s\n", err.c_str()); return 1; }
    dhVector mn, mx;
    scene.ComputeBounds(&mn, &mx);
    std::printf("메시 %zu개, 노드 %zu개, 삼각형 %zu개, 재질 %zu개\n", scene.meshes.size(), scene.nodes.size(), scene.TriangleCount(),
                scene.materials.size());
    std::printf("경계 상자: (%g, %g, %g) ~ (%g, %g, %g)\n", static_cast<double>(mn.x), static_cast<double>(mn.y), static_cast<double>(mn.z),
                static_cast<double>(mx.x), static_cast<double>(mx.y), static_cast<double>(mx.z));
    for (size_t i = 0; i < scene.warnings.size(); i++) std::printf("경고: %s\n", scene.warnings[i].c_str());
    return 0;
}

}  // namespace

#ifdef AMR_HAS_QT

#include <QAction>
#include <QApplication>
#include <QDockWidget>
#include <QDragEnterEvent>
#include <QFileDialog>
#include <QMainWindow>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QSurfaceFormat>
#include <QStatusBar>
#include <QTimer>

#include "CoreQt/JointPanel.h"
#include "CoreQt/ModelViewer.h"

namespace {

class MainWindow : public QMainWindow {
public:
    explicit MainWindow(const Options& o) : QMainWindow(nullptr) {
        setWindowTitle("AMR RobotArm Viewer");
        resize(o.width, o.height);
        setAcceptDrops(true);

        viewer_ = new ModelViewer(this);
        for (size_t i = 0; i < o.packages.size(); i++)
            viewer_->LoadOptions().packagePaths.push_back(std::make_pair(o.packages[i].first, o.packages[i].second));
        setCentralWidget(viewer_);

        panel_ = new JointPanel;
        dock_ = new QDockWidget(QString::fromUtf8("관절"), this);
        dock_->setWidget(panel_);
        addDockWidget(Qt::RightDockWidgetArea, dock_);
        dock_->hide();
        QObject::connect(panel_, &JointPanel::valuesChanged, this, [this]() { viewer_->SetJointValues(panel_->Values()); });

        QMenu* file = menuBar()->addMenu(QString::fromUtf8("파일"));
        QAction* open = file->addAction(QString::fromUtf8("열기..."));
        open->setShortcut(QKeySequence::Open);
        QObject::connect(open, &QAction::triggered, this, [this]() {
            const QString p = QFileDialog::getOpenFileName(
                this, QString::fromUtf8("모델 또는 URDF 열기"), QString(),
                QString::fromUtf8("3D 모델 / URDF (*.urdf *.glb *.gltf *.stl *.obj *.dae);;모든 파일 (*)"));
            if (!p.isEmpty()) openFile(p);
        });
        QAction* quit = file->addAction(QString::fromUtf8("종료"));
        quit->setShortcut(QKeySequence::Quit);
        QObject::connect(quit, &QAction::triggered, this, &QWidget::close);
        QMenu* view = menuBar()->addMenu(QString::fromUtf8("보기"));
        QAction* fit = view->addAction(QString::fromUtf8("화면에 맞춤"));
        fit->setShortcut(Qt::Key_F);
        QObject::connect(fit, &QAction::triggered, viewer_, &ModelViewer::FitView);
        QAction* grid = view->addAction(QString::fromUtf8("격자"));
        grid->setCheckable(true);
        grid->setChecked(true);
        QObject::connect(grid, &QAction::toggled, viewer_, &ModelViewer::SetShowGrid);
        statusBar()->showMessage(QString::fromUtf8("파일 > 열기 (Ctrl+O) 또는 파일을 창에 끌어다 놓으세요. 왼쪽 드래그: 회전, 오른쪽 드래그: 이동, 휠: 확대"));
    }

    bool openFile(const QString& path) {
        QString err;
        if (!viewer_->LoadFile(path, &err)) {
            QMessageBox::warning(this, QString::fromUtf8("열 수 없음"), err);
            return false;
        }
        afterLoad(path);
        return true;
    }

    // 명령행에서 받은 관절 값을 적용한다 (revolute/continuous 는 도 -> rad)
    void applyJoints(const Options& o) {
        for (size_t i = 0; i < o.joints.size(); i++) {
            const int ji = viewer_->Robot().FindJoint(o.joints[i].first);
            if (ji < 0) { std::fprintf(stderr, "경고: 관절 '%s' 이 없습니다\n", o.joints[i].first.c_str()); continue; }
            const bool prismatic = viewer_->Robot().joints[static_cast<size_t>(ji)].type == amr::urdf::kJointPrismatic;
            panel_->SetValue(QString::fromStdString(o.joints[i].first), prismatic ? o.joints[i].second : o.joints[i].second * 3.14159265358979 / 180.0);
        }
    }

    ModelViewer* viewer() { return viewer_; }

protected:
    void dragEnterEvent(QDragEnterEvent* e) override {
        if (e->mimeData()->hasUrls()) e->acceptProposedAction();
    }
    void dropEvent(QDropEvent* e) override {
        const QList<QUrl> urls = e->mimeData()->urls();
        if (!urls.isEmpty()) openFile(urls[0].toLocalFile());
    }

private:
    void afterLoad(const QString& path) {
        if (viewer_->HasRobot()) {
            panel_->SetRobot(viewer_->Robot());
            dock_->setVisible(!panel_->IsEmpty());
        } else {
            dock_->hide();
        }
        QString msg = path + "  -  " + viewer_->Summary();
        if (!viewer_->Warnings().isEmpty()) msg += QString::fromUtf8("  (경고 %1건: %2)").arg(viewer_->Warnings().size()).arg(viewer_->Warnings()[0]);
        statusBar()->showMessage(msg);
        setWindowTitle(QString("AMR RobotArm Viewer - ") + path);
    }

    ModelViewer* viewer_;
    JointPanel* panel_;
    QDockWidget* dock_;
};

}  // namespace

int main(int argc, char** argv) {
    Options o;
    const int pr = ParseArgs(argc, argv, &o);
    if (pr == 1) return 0;
    if (pr == 2) return 2;
    if (o.info) return PrintInfo(o);

    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    MainWindow w(o);
    w.show();
    if (!o.file.empty()) {
        if (!w.openFile(QString::fromLocal8Bit(o.file.c_str()))) {
            if (!o.shot.empty()) return 1;
        } else {
            w.applyJoints(o);
        }
    }
    if (!o.shot.empty()) {
        const std::string shot = o.shot;
        ModelViewer* v = w.viewer();
        QTimer::singleShot(400, &app, [&app, v, shot]() {
            const QImage img = v->grabFramebuffer();
            const bool ok = !img.isNull() && img.save(QString::fromLocal8Bit(shot.c_str()));
            if (!ok) std::fprintf(stderr, "화면 저장 실패: %s\n", shot.c_str());
            app.exit(ok ? 0 : 1);
        });
    }
    return app.exec();
}

#else  // Qt 없이 빌드: 콘솔 모드

int main(int argc, char** argv) {
    Options o;
    const int pr = ParseArgs(argc, argv, &o);
    if (pr == 1) return 0;
    if (pr == 2) return 2;
    if (o.file.empty()) {
        PrintUsage(argv[0]);
        std::printf("\n(이 빌드는 Qt 없이 만들어져 3D 창이 없습니다. -DAMR_BUILD_QT=ON 으로 다시 빌드하세요.)\n");
        return 0;
    }
    return PrintInfo(o);
}

#endif
