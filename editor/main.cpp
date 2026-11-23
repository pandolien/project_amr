// Simul Editor: 프로젝트(main.py + URDF + 오브젝트별 python)를 만들고, 고치고, 돌려 보는 프로그램.
//   실행:  simul_editor [프로젝트 폴더]
//         simul_editor --new 폴더 모델.urdf [모델2.urdf ...]   프로젝트를 만들어 연다
//         simul_editor --upgrade 폴더                            예전 구조 프로젝트를 class 구조로 바꾸고 끝낸다
//         simul_editor 폴더 --run                                 열자마자 main.py 를 실행한다 (3D View 에서 보기. 확인용)
//         simul_editor ... --shot out.png                      창을 그려 PNG 로 저장하고 끝낸다 (확인용)
#include <QApplication>
#include <QSurfaceFormat>
#include <QTimer>

#include <cstdio>

#include "EditorWindow.h"
#include "ProjectTools.h"

int main(int argc, char** argv) {
    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);
    QApplication app(argc, argv);

    QString project, shot, newDir;
    bool run = false;
    QStringList urdfs;
    const QStringList args = app.arguments();
    for (int i = 1; i < args.size(); i++) {
        if (args[i] == QLatin1String("--shot") && i + 1 < args.size()) shot = args[++i];
        else if (args[i] == QLatin1String("--new") && i + 1 < args.size()) newDir = args[++i];
        else if (args[i] == QLatin1String("--run")) run = true;
        else if (!newDir.isEmpty() && args[i].endsWith(QLatin1String(".urdf"), Qt::CaseInsensitive)) urdfs << args[i];
        else if (project.isEmpty()) project = args[i];
    }
    for (int i = 1; i + 1 < args.size(); i++) {
        if (args[i] != QLatin1String("--upgrade")) continue;
        QString err;
        QStringList lines;
        const bool ok = projecttools::UpgradeProject(args[i + 1], &err, &lines);
        for (int k = 0; k < lines.size(); k++) std::printf("%s\n", lines[k].toLocal8Bit().constData());
        if (!ok) std::printf("%s\n", err.toLocal8Bit().constData());
        return ok ? 0 : 1;
    }
    EditorWindow w;
    w.show();
    if (!newDir.isEmpty()) { if (!w.NewProject(newDir, urdfs)) return 1; }
    else if (!project.isEmpty()) { if (!w.OpenProject(project)) return 1; }
    if (run) w.RunMain(EditorWindow::InEditor);
    if (!shot.isEmpty()) {
        QTimer::singleShot(run ? 4500 : 1500, &app, [&]() { w.grab().save(shot); app.quit(); });
    }
    return app.exec();
}
