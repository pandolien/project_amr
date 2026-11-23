#include "ProjectTools.h"

#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QMap>
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

// URDF 를 OBJECT/<이름>.urdf 로 쓰고, 그것이 참조하는 mesh/텍스처를 OBJECT/OBJ/ 로 모은다 (URDF 안의 경로는 OBJ/<파일> 로 고쳐 쓴다)
bool CopyModel(const QString& urdf, const QString& objectDir, QMap<QString, QString>* used, QString* error) {
    const QFileInfo ui(urdf);
    const QString objDir = objectDir + QStringLiteral("/OBJ");
    QDir().mkpath(objDir);
    QFile f(urdf);
    if (!f.open(QIODevice::ReadOnly)) { if (error) *error = Utf8("읽을 수 없습니다: ") + urdf; return false; }
    QString text = QString::fromUtf8(f.readAll());
    f.close();

    auto putFile = [&](const QString& src) -> QString {   // OBJ/ 안의 이름을 돌려준다
        const QString key = QFileInfo(src).canonicalFilePath();
        if (used->contains(key)) return (*used)[key];
        QString base = QFileInfo(src).fileName();
        QString name = base;
        for (int n = 2; QFileInfo(objDir + QLatin1Char('/') + name).exists(); n++) {
            const int dot = base.lastIndexOf(QLatin1Char('.'));
            name = (dot > 0 ? base.left(dot) : base) + QLatin1Char('_') + QString::number(n) + (dot > 0 ? base.mid(dot) : QString());
        }
        QFile::copy(src, objDir + QLatin1Char('/') + name);
        used->insert(key, name);
        // .obj 의 .mtl, .mtl 의 텍스처는 옆에 같이 둔다
        if (name.endsWith(QStringLiteral(".obj"), Qt::CaseInsensitive)) {
            const QString mtl = src.left(src.size() - 4) + QStringLiteral(".mtl");
            if (QFileInfo(mtl).isFile()) {
                const QString mname = name.left(name.size() - 4) + QStringLiteral(".mtl");
                QFile::remove(objDir + QLatin1Char('/') + mname);
                QFile::copy(mtl, objDir + QLatin1Char('/') + mname);
                QFile mf(mtl);
                if (mf.open(QIODevice::ReadOnly)) {
                    const QStringList lines = QString::fromUtf8(mf.readAll()).split(QLatin1Char('\n'));
                    for (int i = 0; i < lines.size(); i++) {
                        const QString l = lines[i].trimmed();
                        if (l.startsWith(QStringLiteral("map_"))) {
                            const QString tex = l.section(QLatin1Char(' '), -1);
                            const QString tp = QFileInfo(mtl).absolutePath() + QLatin1Char('/') + tex;
                            if (QFileInfo(tp).isFile()) {
                                const QString to = objDir + QLatin1Char('/') + QFileInfo(tex).fileName();
                                QDir().mkpath(QFileInfo(to).absolutePath());
                                QFile::remove(to);
                                QFile::copy(tp, to);
                            }
                        }
                    }
                }
            }
        }
        return name;
    };

    QRegularExpression re(QStringLiteral("filename\\s*=\\s*\"([^\"]+)\""));
    QString out;
    int last = 0;
    QRegularExpressionMatchIterator m = re.globalMatch(text);
    while (m.hasNext()) {
        const QRegularExpressionMatch mm = m.next();
        QString rel = mm.captured(1);
        QString clean = rel;
        clean.remove(QRegularExpression(QStringLiteral("^(package://[^/]*/|file://)")));
        const QString sp = QDir::isAbsolutePath(clean) ? clean : ui.absolutePath() + QLatin1Char('/') + clean;
        if (clean.split(QLatin1Char('/')).contains(QStringLiteral("..")) && !QFileInfo(sp).isFile()) continue;
        if (!QFileInfo(sp).isFile()) continue;
        out += text.mid(last, mm.capturedStart(1) - last);
        out += QStringLiteral("OBJ/") + putFile(sp);
        last = mm.capturedEnd(1);
    }
    out += text.mid(last);
    return WriteText(objectDir + QLatin1Char('/') + ui.fileName(), out, error);
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
오브젝트는 Code/<오브젝트 이름>.py 의 update(info) 가 움직인다. DLL(simul_core)은 Simul/ 폴더에 들어 있다."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "Simul"))     # Simul/Simul.py : DLL 을 부르는 python 파일
import Simul  # noqa: E402

WINDOW = "--no-window" not in sys.argv              # 3D 창 (simul_gui, Qt 필요)
DURATION = None                                     # 초. None 이면 창을 닫을 때까지 (창이 없으면 5 초)

sim = Simul.Simulation()
%1
behaviors = Simul.Behaviors(os.path.join(HERE, "Code"), sim)
if WINDOW:
    sim.show("%2")
sim.run(behaviors, dt=0.02, duration=DURATION if WINDOW else (DURATION or 5.0), realtime=WINDOW)
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
    QDirIterator it(dir + QStringLiteral("/OBJECT"), QStringList() << QStringLiteral("*.urdf"), QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) out << it.next();
    out.sort();
    return out;
}

bool CreateProject(const QString& dirIn, const QStringList& urdfs, QString* error, QStringList* log) {
    const QString dir = QDir::cleanPath(QFileInfo(dirIn).absoluteFilePath());
    if (urdfs.isEmpty()) { if (error) *error = Utf8("URDF 를 하나 이상 고르세요"); return false; }
    const QStringList cores = FindLibs(QStringLiteral("simul_core"));
    if (cores.isEmpty()) { if (error) *error = Utf8("simul_core 라이브러리를 찾지 못했습니다 (에디터와 같이 빌드되어야 합니다. 환경 변수 SIMUL_LIB 로 폴더를 알려 줄 수도 있습니다)"); return false; }
    const QString pySrc = QString::fromUtf8(SIMUL_SOURCE_DIR) + QStringLiteral("/python/Simul/Simul.py");
    if (!QFileInfo(pySrc).isFile()) { if (error) *error = Utf8("python/Simul/Simul.py 를 찾지 못했습니다: ") + pySrc; return false; }
    const QString simulDir = dir + QStringLiteral("/Simul");
    const QString objectDir = dir + QStringLiteral("/OBJECT");
    if (!QDir().mkpath(simulDir) || !QDir().mkpath(objectDir + QStringLiteral("/OBJ")) || !QDir().mkpath(dir + QStringLiteral("/Code"))) { if (error) *error = Utf8("폴더를 만들 수 없습니다: ") + dir; return false; }

    QFile::remove(simulDir + QStringLiteral("/Simul.py"));
    QFile::copy(pySrc, simulDir + QStringLiteral("/Simul.py"));
    QFile::remove(simulDir + QLatin1Char('/') + QFileInfo(cores[0]).fileName());
    QFile::copy(cores[0], simulDir + QLatin1Char('/') + QFileInfo(cores[0]).fileName());
    if (log) *log << Utf8("DLL 복사: ") + QFileInfo(cores[0]).fileName();
    const QStringList guis = FindLibs(QStringLiteral("simul_gui"));
    if (!guis.isEmpty()) {
        QFile::remove(simulDir + QLatin1Char('/') + QFileInfo(guis[0]).fileName());
        QFile::copy(guis[0], simulDir + QLatin1Char('/') + QFileInfo(guis[0]).fileName());
        if (log) *log << Utf8("DLL 복사: ") + QFileInfo(guis[0]).fileName();
    } else if (log) *log << Utf8("[알림] simul_gui(3D 창)가 없어 python 은 창 없이 돕니다");

    QString addLines;
    QMap<QString, QString> usedMeshes;
    for (int i = 0; i < urdfs.size(); i++) {
        const QFileInfo ui(urdfs[i]);
        if (!CopyModel(ui.absoluteFilePath(), objectDir, &usedMeshes, error)) return false;
        const ModelInfo mi = ReadModelInfo(ui.absoluteFilePath());
        const QString py = dir + QStringLiteral("/Code/") + mi.object + QStringLiteral(".py");
        if (!QFileInfo(py).exists()) {
            const QString joints = mi.joints.isEmpty() ? Utf8("(없음)") : mi.joints.join(QStringLiteral(", "));
            if (!WriteText(py, QString::fromUtf8(kObject).arg(mi.object, joints), error)) return false;
        }
        addLines += QStringLiteral("sim.add_model(os.path.join(HERE, \"OBJECT\", \"%1\"))\n").arg(ui.fileName());
        if (log) *log << Utf8("오브젝트 ") + mi.object + Utf8(": 관절 ") + mi.joints.join(QStringLiteral(", "));
    }
    addLines.chop(1);
    if (!WriteText(dir + QStringLiteral("/main.py"), QString::fromUtf8(kMain).arg(addLines, QFileInfo(dir).fileName()), error)) return false;
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
