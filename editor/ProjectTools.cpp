#include "ProjectTools.h"

#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QTextStream>
#include <QXmlStreamReader>

namespace projecttools {
namespace {

QString Utf8(const char* s) { return QString::fromUtf8(s); }

bool WriteText(const QString& path, const QString& text, QString* error) {
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) { if (error) *error = Utf8("쓸 수 없습니다: ") + path; return false; }
    f.write(text.toUtf8());
    return true;
}

bool CopyTree(const QString& src, const QString& dst, QString* error) {
    QDir().mkpath(dst);
    QDirIterator it(src, QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot, QDirIterator::NoIteratorFlags);
    while (it.hasNext()) {
        it.next();
        const QFileInfo fi = it.fileInfo();
        if (fi.fileName() == QLatin1String("__pycache__")) continue;
        const QString to = dst + QLatin1Char('/') + fi.fileName();
        if (fi.isDir()) { if (!CopyTree(fi.filePath(), to, error)) return false; }
        else {
            QFile::remove(to);
            if (!QFile::copy(fi.filePath(), to)) { if (error) *error = Utf8("복사하지 못했습니다: ") + fi.filePath(); return false; }
        }
    }
    return true;
}

// URDF 가 참조하는 mesh/텍스처만 (상대 경로를 지켜서) 복사
bool CopyModel(const QString& urdf, const QString& dstDir, QString* error) {
    QDir().mkpath(dstDir);
    const QFileInfo ui(urdf);
    QFile::remove(dstDir + QLatin1Char('/') + ui.fileName());
    if (!QFile::copy(urdf, dstDir + QLatin1Char('/') + ui.fileName())) { if (error) *error = Utf8("복사하지 못했습니다: ") + urdf; return false; }
    QFile f(urdf);
    if (!f.open(QIODevice::ReadOnly)) return true;
    const QString text = QString::fromUtf8(f.readAll());
    QRegularExpression re(QStringLiteral("filename\\s*=\\s*\"([^\"]+)\""));
    QRegularExpressionMatchIterator m = re.globalMatch(text);
    while (m.hasNext()) {
        QString rel = m.next().captured(1);
        rel.remove(QRegularExpression(QStringLiteral("^(package://[^/]*/|file://)")));
        if (QDir::isAbsolutePath(rel) || rel.split(QLatin1Char('/')).contains(QStringLiteral(".."))) continue;
        const QString sp = ui.absolutePath() + QLatin1Char('/') + rel;
        if (!QFileInfo(sp).isFile()) continue;
        const QString to = dstDir + QLatin1Char('/') + rel;
        QDir().mkpath(QFileInfo(to).absolutePath());
        QFile::remove(to);
        QFile::copy(sp, to);
        if (rel.endsWith(QStringLiteral(".obj"), Qt::CaseInsensitive)) {
            const QString mtl = sp.left(sp.size() - 4) + QStringLiteral(".mtl");
            if (QFileInfo(mtl).isFile()) { const QString mto = to.left(to.size() - 4) + QStringLiteral(".mtl"); QFile::remove(mto); QFile::copy(mtl, mto); }
        }
    }
    return true;
}

// simul_core / simul_gui 공유 라이브러리를 찾는다 (에디터와 같이 빌드된 것)
QStringList FindLibs(const QString& base) {
    const QString app = QCoreApplication::applicationDirPath();
    QStringList names;
#ifdef _WIN32
    names << base + QStringLiteral(".dll");
#else
    names << QStringLiteral("lib") + base + QStringLiteral(".so");
#endif
    QStringList dirs;
    dirs << app << app + QStringLiteral("/../lib") << app + QStringLiteral("/..") << app + QStringLiteral("/../../bin") << app + QStringLiteral("/../../lib");
    const QString env = QString::fromLocal8Bit(qgetenv("SIMUL_LIB"));
    if (!env.isEmpty()) dirs.prepend(env);
    QStringList found;
    for (int i = 0; i < dirs.size(); i++)
        for (int k = 0; k < names.size(); k++) {
            const QFileInfo fi(dirs[i] + QLatin1Char('/') + names[k]);
            if (fi.isFile()) { found << fi.canonicalFilePath(); return found; }
        }
    return found;
}

const char* kMain = R"PY(# -*- coding: utf-8 -*-
"""프로젝트 시작 파일.  python main.py [--no-window]
오브젝트는 objects/<이름>.py 의 update(info) 가 움직인다. DLL(simul_core)은 simul/ 폴더에 들어 있다."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                       # HERE/simul : DLL 을 부르는 python 파일
import simul  # noqa: E402

WINDOW = "--no-window" not in sys.argv         # 3D 창 (simul_gui, Qt 필요)
DURATION = None                                # 초. None 이면 창을 닫을 때까지 (창이 없으면 5 초)

sim = simul.Simulation()
%1
behaviors = simul.Behaviors(os.path.join(HERE, "objects"), sim)
if WINDOW:
    sim.show("%2")
sim.run(behaviors, dt=0.02, duration=DURATION if WINDOW else (DURATION or 5.0), realtime=WINDOW)
)PY";

const char* kStartup = R"PY(# -*- coding: utf-8 -*-
# Python CMD 가 시작할 때 읽는다: sim 과 오브젝트 이름을 바로 쓸 수 있다.  예) arm.set_joint("base_yaw", 1.0); sim.step(0.5)
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
sys.path.insert(0, HERE)
import simul

sim = simul.Simulation()
%1
for _name in sim.objects:
    if _name.isidentifier():
        globals()[_name] = sim.objects[_name]
print("sim, 오브젝트: " + ", ".join(sorted(sim.objects)) + "   (sim.show() 로 3D 창)")
)PY";

const char* kObject = R"PY(# -*- coding: utf-8 -*-
"""오브젝트 "%1" 의 python 코드. 매 스텝 update(info) 가 불리고, 이 오브젝트의 움직임을 돌려준다.

info.time, info.dt, info.objects["%1"] (.position .quaternion .joints[이름].value/.lower/.upper), info.collided("%1")
돌려주는 값: {"joints": {관절: 목표 rad 또는 (목표, 속도)}, "position": (x, y, z), "speed": m/s}   (없으면 None)
관절: %2
"""


def update(info):
    return None
)PY";

}  // namespace

bool IsProject(const QString& dir) { return QFileInfo(dir + QStringLiteral("/main.py")).isFile(); }

ModelInfo ReadModelInfo(const QString& urdfPath) {
    ModelInfo mi;
    QFile f(urdfPath);
    if (!f.open(QIODevice::ReadOnly)) return mi;
    QXmlStreamReader xml(&f);
        while (!xml.atEnd()) {
        xml.readNext();
        if (!xml.isStartElement()) continue;
        if (xml.name() == QLatin1String("joint")) {
            const QString type = xml.attributes().value(QStringLiteral("type")).toString();
            if (type != QLatin1String("fixed")) mi.joints << xml.attributes().value(QStringLiteral("name")).toString();
        }
    }
    mi.object = QFileInfo(urdfPath).completeBaseName();   // simul_core 는 오브젝트를 파일 이름(확장자 제외)으로 부른다
    return mi;
}

QStringList ProjectModels(const QString& dir) {
    QStringList out;
    QDirIterator it(dir + QStringLiteral("/models"), QStringList() << QStringLiteral("*.urdf"), QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) out << it.next();
    out.sort();
    return out;
}

bool CreateProject(const QString& dirIn, const QStringList& urdfs, QString* error, QStringList* log) {
    const QString dir = QDir::cleanPath(QFileInfo(dirIn).absoluteFilePath());
    if (urdfs.isEmpty()) { if (error) *error = Utf8("URDF 를 하나 이상 고르세요"); return false; }
    const QStringList cores = FindLibs(QStringLiteral("simul_core"));
    if (cores.isEmpty()) { if (error) *error = Utf8("simul_core 라이브러리를 찾지 못했습니다 (에디터와 같이 빌드되어야 합니다. 환경 변수 SIMUL_LIB 로 폴더를 알려 줄 수도 있습니다)"); return false; }
    const QString pkgSrc = QString::fromUtf8(SIMUL_SOURCE_DIR) + QStringLiteral("/python/simul");
    if (!QFileInfo(pkgSrc + QStringLiteral("/__init__.py")).isFile()) { if (error) *error = Utf8("python/simul 을 찾지 못했습니다: ") + pkgSrc; return false; }
    if (!QDir().mkpath(dir + QStringLiteral("/objects")) || !QDir().mkpath(dir + QStringLiteral("/models"))) { if (error) *error = Utf8("폴더를 만들 수 없습니다: ") + dir; return false; }

    const QString pkg = dir + QStringLiteral("/simul");
    if (!CopyTree(pkgSrc, pkg, error)) return false;
    QFile::copy(cores[0], pkg + QLatin1Char('/') + QFileInfo(cores[0]).fileName());
    if (log) *log << Utf8("DLL 복사: ") + QFileInfo(cores[0]).fileName();
    const QStringList guis = FindLibs(QStringLiteral("simul_gui"));
    if (!guis.isEmpty()) { QFile::copy(guis[0], pkg + QLatin1Char('/') + QFileInfo(guis[0]).fileName()); if (log) *log << Utf8("DLL 복사: ") + QFileInfo(guis[0]).fileName(); }
    else if (log) *log << Utf8("[알림] simul_gui(3D 창)가 없어 python 은 창 없이 돕니다");

    QString addLines;
    for (int i = 0; i < urdfs.size(); i++) {
        const QFileInfo ui(urdfs[i]);
        const QString sub = ui.completeBaseName();
        if (!CopyModel(ui.absoluteFilePath(), dir + QStringLiteral("/models/") + sub, error)) return false;
        const ModelInfo mi = ReadModelInfo(ui.absoluteFilePath());
        const QString py = dir + QStringLiteral("/objects/") + mi.object + QStringLiteral(".py");
        if (!QFileInfo(py).exists()) {
            QString joints = mi.joints.isEmpty() ? Utf8("(없음)") : mi.joints.join(QStringLiteral(", "));
            if (!WriteText(py, QString::fromUtf8(kObject).arg(mi.object, joints), error)) return false;
        }
        addLines += QStringLiteral("sim.add_model(os.path.join(HERE, \"models\", \"%1\", \"%2\"))\n").arg(sub, ui.fileName());
        if (log) *log << Utf8("오브젝트 ") + mi.object + Utf8(": 관절 ") + mi.joints.join(QStringLiteral(", "));
    }
    addLines.chop(1);
    if (!WriteText(dir + QStringLiteral("/main.py"), QString::fromUtf8(kMain).arg(addLines, QFileInfo(dir).fileName()), error)) return false;
    if (!WriteText(dir + QStringLiteral("/startup.py"), QString::fromUtf8(kStartup).arg(addLines), error)) return false;
    if (log) *log << Utf8("만들었습니다: ") + dir;
    return true;
}

void FindPython(QString* program, QStringList* args) {
    args->clear();
    program->clear();
    const QByteArray envPy = qgetenv("SIMUL_PYTHON");
    if (!envPy.isEmpty()) { *program = QString::fromLocal8Bit(envPy); return; }
    const char* names[] = {"python3", "python"};
    for (int i = 0; i < 2; i++) {
        const QString p = QStandardPaths::findExecutable(QString::fromLatin1(names[i]));
        if (!p.isEmpty()) { *program = p; return; }
    }
    const QString py = QStandardPaths::findExecutable(QStringLiteral("py"));   // Windows 의 python 실행기
    if (!py.isEmpty()) { *program = py; *args << QStringLiteral("-3"); }
}

}  // namespace projecttools
