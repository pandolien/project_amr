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

// URDF 의 mesh 경로(상대, 절대, package://, file://)를 실제 파일로 찾는다.
// package:// 나 ../ 는 URDF 위쪽 폴더들에서도 찾고, 그래도 없으면 위쪽 폴더 밑에서 같은 끝 경로를 가진 파일을 찾는다.
QString ResolveMesh(const QString& urdfDir, const QString& ref, const QStringList& extraDirs) {
    QString clean = ref.trimmed().replace(QLatin1Char('\\'), QLatin1Char('/'));
    QString pkgRest;
    QRegularExpressionMatch pm = QRegularExpression(QStringLiteral("^package://[^/]*/(.*)$")).match(clean);
    if (pm.hasMatch()) { pkgRest = pm.captured(1); clean = pkgRest; }
    clean.remove(QRegularExpression(QStringLiteral("^file://")));
    if (QDir::isAbsolutePath(clean) && QFileInfo(clean).isFile()) return QFileInfo(clean).canonicalFilePath();
    // 1) URDF 폴더와 그 위쪽 폴더들에서 같은 상대 경로
    QDir d(urdfDir);
    for (int up = 0; up < 8; up++) {
        const QString p = d.absoluteFilePath(clean);
        if (QFileInfo(p).isFile()) return QFileInfo(p).canonicalFilePath();
        if (!d.cdUp()) break;
    }
    // 2) ../ 를 뗀 끝 경로를 위쪽 폴더 밑에서 찾는다 (긴 경로부터)
    QStringList parts = clean.split(QLatin1Char('/'));
    parts.removeAll(QString());
    while (!parts.isEmpty() && (parts.first() == QStringLiteral("..") || parts.first() == QStringLiteral("."))) parts.removeFirst();
    if (parts.isEmpty()) return QString();
    const QString fileName = parts.last();
    QStringList roots = extraDirs;                      // 사용자가 알려 준 mesh 폴더가 먼저
    QDir top(urdfDir);
    for (int up = 0; up < 4 && top.absolutePath().count(QLatin1Char('/')) > 2 && top.cdUp(); up++) {}   // 위로 4 단계 (루트/홈 바로 밑까지만)
    roots << top.absolutePath();
    const QString lowName = fileName.toLower();
    QString best;
    int bestScore = -1, seen = 0;
    for (int r = 0; r < roots.size(); r++) {
        QDirIterator it(roots[r], QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext() && seen < 60000) {
            const QString p = it.next();
            seen++;
            const QStringList pp = p.split(QLatin1Char('/'));
            if (pp.last().toLower() != lowName) continue;
            int score = 0;
            for (int a = parts.size() - 1, b = pp.size() - 1; a >= 0 && b >= 0 && parts[a].toLower() == pp[b].toLower(); a--, b--) score++;
            if (score > bestScore) { bestScore = score; best = p; }
        }
        if (!best.isEmpty()) break;                     // 앞선 폴더에서 찾았으면 거기서 끝
    }
    return best.isEmpty() ? QString() : QFileInfo(best).canonicalFilePath();
}

// URDF 를 OBJECT/<이름>.urdf 로 쓰고, 그것이 참조하는 mesh/텍스처를 OBJECT/OBJ/ 로 모은다 (URDF 안의 경로는 OBJ/<파일> 로 고쳐 쓴다)
bool CopyModel(const QString& urdf, const QString& objectDir, QMap<QString, QString>* used, QString* error, QStringList* log, const QStringList& extraDirs, int* missingTotal) {
    const QFileInfo ui(urdf);
    const QString objDir = objectDir + QStringLiteral("/OBJ");
    QDir().mkpath(objDir);
    QFile f(urdf);
    if (!f.open(QIODevice::ReadOnly)) { if (error) *error = Utf8("읽을 수 없습니다: ") + urdf; return false; }
    QString text = QString::fromUtf8(f.readAll());
    f.close();

    // src 를 OBJ/<rel> 로 복사한다 (rel 은 하위 폴더 포함 가능). 이미 같은 이름이 다른 파일이면 번호를 붙인다.
    auto copyInto = [&](const QString& src, const QString& rel, bool uniq) -> QString {
        const QString key = QFileInfo(src).canonicalFilePath();
        if (uniq && used->contains(key)) return (*used)[key];
        QString name = rel;
        if (uniq) {
            const QString base = QFileInfo(rel).fileName();
            name = base;
            for (int n = 2; QFileInfo(objDir + QLatin1Char('/') + name).exists(); n++) {
                const int dot = base.lastIndexOf(QLatin1Char('.'));
                name = (dot > 0 ? base.left(dot) : base) + QLatin1Char('_') + QString::number(n) + (dot > 0 ? base.mid(dot) : QString());
            }
        }
        const QString to = objDir + QLatin1Char('/') + name;
        QDir().mkpath(QFileInfo(to).absolutePath());
        QFile::remove(to);
        if (!QFile::copy(src, to)) { if (log) *log << Utf8("[경고] 복사 실패: ") + src; return name; }
        if (uniq) used->insert(key, name);
        return name;
    };
    // 참조 파일(.mtl, 텍스처)을 원래 상대 경로 그대로 mesh 옆에 둔다
    auto copyRef = [&](const QString& fromDir, const QString& meshName, const QString& ref) {
        const QString r = ref.trimmed().replace(QLatin1Char('\\'), QLatin1Char('/'));
        if (r.isEmpty() || r.startsWith(QStringLiteral("data:"))) return;
        QString src = QDir::isAbsolutePath(r) ? r : fromDir + QLatin1Char('/') + r;
        if (!QFileInfo(src).isFile()) {
            const QString alt = fromDir + QLatin1Char('/') + QFileInfo(r).fileName();
            if (QFileInfo(alt).isFile()) src = alt;
            else { if (log) *log << Utf8("[경고] 참조 파일을 찾지 못했습니다: ") + r + QStringLiteral("  (") + meshName + QLatin1Char(')'); return; }
        }
        const QString rel = QDir::isAbsolutePath(r) ? QFileInfo(r).fileName() : QDir::cleanPath(QFileInfo(meshName).path() + QLatin1Char('/') + r);
        if (rel.startsWith(QStringLiteral(".."))) { copyInto(src, QFileInfo(r).fileName(), false); return; }
        copyInto(src, rel, false);
    };

    auto putFile = [&](const QString& src) -> QString {   // OBJ/ 안의 이름을 돌려준다
        const bool had = used->contains(QFileInfo(src).canonicalFilePath());
        const QString name = copyInto(src, QFileInfo(src).fileName(), true);
        if (had) return name;
        const QString srcDir = QFileInfo(src).absolutePath();
        const QString lower = name.toLower();
        QFile sf(src);
        if (lower.endsWith(QStringLiteral(".obj")) && sf.open(QIODevice::ReadOnly)) {
            const QStringList lines = QString::fromUtf8(sf.readAll()).split(QLatin1Char('\n'));
            bool anyMtl = false;
            for (int i = 0; i < lines.size(); i++) {
                const QString l = lines[i].trimmed();
                if (l.startsWith(QStringLiteral("mtllib "))) {
                    anyMtl = true;
                    const QString mref = l.mid(7).trimmed();
                    copyRef(srcDir, name, mref);
                    // mtl 안의 텍스처
                    QString msrc = QDir::isAbsolutePath(mref) ? mref : srcDir + QLatin1Char('/') + mref;
                    QFile mf(msrc);
                    if (mf.open(QIODevice::ReadOnly)) {
                        const QStringList ml = QString::fromUtf8(mf.readAll()).split(QLatin1Char('\n'));
                        for (int k = 0; k < ml.size(); k++) {
                            const QString t = ml[k].trimmed();
                            if (t.startsWith(QStringLiteral("map_")) || t.startsWith(QStringLiteral("bump ")) || t.startsWith(QStringLiteral("disp ")) || t.startsWith(QStringLiteral("decal "))) {
                                const QString tex = t.section(QLatin1Char(' '), -1);
                                copyRef(QFileInfo(msrc).absolutePath(), QFileInfo(mref).isRelative() ? QFileInfo(name).path() + QLatin1Char('/') + mref : name, tex);
                            }
                        }
                    }
                }
            }
            if (!anyMtl) {   // mtllib 줄이 없어도 같은 이름의 .mtl 이 옆에 있으면 같이 둔다
                const QString mtl = src.left(src.size() - 4) + QStringLiteral(".mtl");
                if (QFileInfo(mtl).isFile()) copyInto(mtl, name.left(name.size() - 4) + QStringLiteral(".mtl"), false);
            }
        } else if (lower.endsWith(QStringLiteral(".dae"))) {   // collada 의 텍스처 (<init_from>파일</init_from>)
            if (sf.open(QIODevice::ReadOnly)) {
                const QString dae = QString::fromUtf8(sf.readAll());
                QRegularExpressionMatchIterator im = QRegularExpression(QStringLiteral("<init_from>\\s*([^<\\s]+\\.(?:png|jpg|jpeg|bmp|tga|tif|tiff))\\s*</init_from>"), QRegularExpression::CaseInsensitiveOption).globalMatch(dae);
                while (im.hasNext()) {
                    QString tex = im.next().captured(1);
                    tex.remove(QRegularExpression(QStringLiteral("^file://")));
                    copyRef(srcDir, name, tex);
                }
            }
        }
        return name;
    };

    QRegularExpression re(QStringLiteral("filename\\s*=\\s*\"([^\"]+)\""));
    QString out;
    int last = 0, copied = 0, missing = 0;
    QRegularExpressionMatchIterator m = re.globalMatch(text);
    while (m.hasNext()) {
        const QRegularExpressionMatch mm = m.next();
        const QString ref = mm.captured(1);
        const QString sp = ResolveMesh(ui.absolutePath(), ref, extraDirs);
        if (sp.isEmpty()) {
            missing++;
            if (log) *log << Utf8("[경고] mesh 를 찾지 못했습니다: ") + ref + QStringLiteral("  (") + ui.fileName() + QLatin1Char(')');
            continue;
        }
        out += text.mid(last, mm.capturedStart(1) - last);
        out += QStringLiteral("OBJ/") + putFile(sp);
        last = mm.capturedEnd(1);
        copied++;
    }
    out += text.mid(last);
    if (missingTotal) *missingTotal += missing;
    if (log) *log << ui.fileName() + Utf8(": mesh ") + QString::number(copied) + Utf8("개 복사") + (missing ? Utf8(", 못 찾은 것 ") + QString::number(missing) + Utf8("개") : QString());
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
DLL(simul_core)은 Simul/ 폴더에 있고, 오브젝트는 OBJECT/*.urdf, 오브젝트별 제어 class 는 Code/<오브젝트 이름>.py 에 있다."""
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "Simul"))     # Simul/Simul.py : DLL 을 부르는 python 파일
import Simul  # noqa: E402


class Main(object):
    """프로젝트 전체: 시뮬레이션을 만들고, OBJECT/ 의 오브젝트를 올리고, Code/ 의 오브젝트 제어 class 들을 연결해 돌린다."""

    TITLE = "%1"
    DT = 0.02                                       # 한 스텝 (s)
    DURATION = None                                 # 돌 시간 (s). None 이면 창을 닫을 때까지 (창이 없으면 5 초)

    def __init__(self, window=True, editor=False):
        self.window = window                        # 3D 창 (simul_gui, Qt 필요)
        self.editor = editor                        # 에디터가 3D View 로 보여 준다 (창 없이, 닫을 때까지 실시간으로 돈다)
        self.sim = Simul.Simulation()               # simul_core DLL 위의 시뮬레이션
        self.load_objects()
        self.controllers = Simul.Behaviors(os.path.join(HERE, "Code"), self.sim)   # Code/<오브젝트>.py 의 Controller 들

    def load_objects(self):
        for f in sorted(glob.glob(os.path.join(HERE, "OBJECT", "*.urdf"))):       # OBJECT/ 의 URDF 를 모두 올린다 (추가하려면 파일만 넣으면 된다)
            self.sim.add_model(f)

    def run(self):
        if self.window:
            self.sim.show(self.TITLE)
        try:
            live = self.window or self.editor
            self.sim.run(self.controllers, dt=self.DT, duration=self.DURATION if live else (self.DURATION or 5.0), realtime=live)
        finally:
            self.controllers.stop()


if __name__ == "__main__":
    editor = "--editor" in sys.argv
    Main(window="--no-window" not in sys.argv and not editor, editor=editor).run()
)PY";

const char* kRunBat = R"BAT(@echo off
rem Project run script: creates the venv (first time), installs requirements.txt, runs main.py.
rem   run.bat                 3D window
rem   run.bat --no-window     no window (5 seconds)
setlocal
cd /d "%~dp0"
set "SIMUL_LIB=%~dp0Simul"
if not exist "venv\Scripts\python.exe" (
    echo [venv] creating venv ...
    where py >nul 2>nul
    if not errorlevel 1 (py -3 -m venv venv) else (python -m venv venv)
    if errorlevel 1 (
        echo [venv] failed. Install Python 3 and try again.
        exit /b 1
    )
)
if exist requirements.txt "venv\Scripts\python.exe" -m pip install -q --disable-pip-version-check -r requirements.txt
"venv\Scripts\python.exe" main.py %*
exit /b %errorlevel%
)BAT";

const char* kRunSh = R"SH(#!/usr/bin/env bash
# Project run script: creates the venv (first time), installs requirements.txt, runs main.py.
#   ./run.sh                 3D window
#   ./run.sh --no-window     no window (5 seconds)
cd "$(dirname "$0")" || exit 1
export SIMUL_LIB="$PWD/Simul"
if [ ! -x venv/bin/python ]; then
    echo "[venv] creating venv ..."
    python3 -m venv venv || { echo "[venv] failed. Install python3-venv (Ubuntu: sudo apt install python3-venv) and try again."; exit 1; }
fi
[ -f requirements.txt ] && venv/bin/python -m pip install -q --disable-pip-version-check -r requirements.txt
exec venv/bin/python main.py "$@"
)SH";

const char* kRequirements = "# python packages this project needs (pip install -r). Simul itself needs none.\n";

bool WriteRunFiles(const QString& dir, QString* error) {
    if (!QFileInfo(dir + QStringLiteral("/run.bat")).isFile() && !WriteText(dir + QStringLiteral("/run.bat"), QString::fromUtf8(kRunBat).replace(QStringLiteral("\n"), QStringLiteral("\r\n")), error)) return false;
    if (!QFileInfo(dir + QStringLiteral("/run.sh")).isFile()) {
        if (!WriteText(dir + QStringLiteral("/run.sh"), QString::fromUtf8(kRunSh), error)) return false;
        QFile::setPermissions(dir + QStringLiteral("/run.sh"), QFile::permissions(dir + QStringLiteral("/run.sh")) | QFileDevice::ExeOwner | QFileDevice::ExeGroup | QFileDevice::ExeOther | QFileDevice::ReadOwner | QFileDevice::WriteOwner);
    }
    if (!QFileInfo(dir + QStringLiteral("/requirements.txt")).isFile() && !WriteText(dir + QStringLiteral("/requirements.txt"), QString::fromUtf8(kRequirements), error)) return false;
    return true;
}

const char* kTemplate = R"PY(# -*- coding: utf-8 -*-
"""오브젝트 제어 class 의 견본. 이 파일을 복사해 <오브젝트 이름>.py 로 바꾸면 그 오브젝트의 코드가 된다 (오브젝트 이름 = OBJECT/ 의 URDF 파일 이름).
class 이름은 자유이고, Simul.Controller 를 이어받으면 main.py 가 오브젝트마다 하나씩 만들어 연결한다."""
import Simul


class Object(Simul.Controller):
    """self.sim: 시뮬레이션, self.obj: 이 오브젝트, self.name: 오브젝트 이름"""

    def start(self):
        """시작할 때 한 번"""

    def update(self, info):
        """매 스텝. info.time, info.dt, info.objects["이름"] (.position .quaternion .joints[관절].value/.lower/.upper), info.collided("이름")
        돌려주는 값: {"joints": {관절: 목표 rad 또는 (목표, 속도)}, "position": (x, y, z), "speed": m/s}   (없으면 None)"""
        return None

    def stop(self):
        """끝날 때 한 번"""
)PY";

const char* kObject = R"PY(# -*- coding: utf-8 -*-
"""오브젝트 "%1" 의 제어 class. main.py 가 시작할 때 만들어 연결한다.
관절: %2"""
import Simul


class %3(Simul.Controller):
    """self.sim: 시뮬레이션, self.obj: 이 오브젝트("%1"), self.name: 오브젝트 이름"""

    def start(self):
        """시작할 때 한 번"""

    def update(self, info):
        """매 스텝. info.time, info.dt, info.objects["%1"] (.position .quaternion .joints[이름].value/.lower/.upper), info.collided("%1")
        돌려주는 값: {"joints": {관절: 목표 rad 또는 (목표, 속도)}, "position": (x, y, z), "speed": m/s}   (없으면 None)"""
        return None

    def stop(self):
        """끝날 때 한 번"""
)PY";

// 오브젝트 이름 -> python class 이름 (arm_1 -> Arm1)
QString ClassName(const QString& object) {
    QString out;
    bool up = true;
    for (int i = 0; i < object.size(); i++) {
        const QChar c = object[i];
        if (c.isLetterOrNumber() && c.unicode() < 128) { out += up ? c.toUpper() : c; up = false; }
        else up = true;
    }
    if (out.isEmpty() || out[0].isDigit()) out.prepend(QStringLiteral("Obj"));
    return out;
}

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

bool AddModels(const QString& dirIn, const QStringList& urdfs, QString* error, QStringList* log, const QStringList& meshDirs, int* missing) {
    const QString dir = QDir::cleanPath(QFileInfo(dirIn).absoluteFilePath());
    const QString objectDir = dir + QStringLiteral("/OBJECT");
    QDir().mkpath(objectDir + QStringLiteral("/OBJ"));
    QDir().mkpath(dir + QStringLiteral("/Code"));
    if (missing) *missing = 0;
    QMap<QString, QString> usedMeshes;
    for (int i = 0; i < urdfs.size(); i++) {
        const QFileInfo ui(urdfs[i]);
        if (!CopyModel(ui.absoluteFilePath(), objectDir, &usedMeshes, error, log, meshDirs, missing)) return false;
        const ModelInfo mi = ReadModelInfo(ui.absoluteFilePath());
        const QString py = dir + QStringLiteral("/Code/") + mi.object + QStringLiteral(".py");
        if (!QFileInfo(py).exists()) {
            const QString joints = mi.joints.isEmpty() ? Utf8("(없음)") : mi.joints.join(QStringLiteral(", "));
            if (!WriteText(py, QString::fromUtf8(kObject).arg(mi.object, joints, ClassName(mi.object)), error)) return false;
        }
        if (log) *log << Utf8("오브젝트 ") + mi.object + Utf8(": 관절 ") + mi.joints.join(QStringLiteral(", "));
    }
    return true;
}

bool CreateProject(const QString& dirIn, const QStringList& urdfs, QString* error, QStringList* log) {
    const QString dir = QDir::cleanPath(QFileInfo(dirIn).absoluteFilePath());
    const QStringList cores = FindLibs(QStringLiteral("simul_core"));
    if (cores.isEmpty()) { if (error) *error = Utf8("simul_core 라이브러리를 찾지 못했습니다 (에디터와 같이 빌드되어야 합니다. 환경 변수 SIMUL_LIB 로 폴더를 알려 줄 수도 있습니다)"); return false; }
    const QString pySrc = QString::fromUtf8(SIMUL_SOURCE_DIR) + QStringLiteral("/python/Simul/Simul.py");
    if (!QFileInfo(pySrc).isFile()) { if (error) *error = Utf8("python/Simul/Simul.py 를 찾지 못했습니다: ") + pySrc; return false; }
    if (QFileInfo(dir + QStringLiteral("/main.py")).exists()) { if (error) *error = Utf8("이미 프로젝트입니다: ") + dir; return false; }
    const QString simulDir = dir + QStringLiteral("/Simul");
    if (!QDir().mkpath(simulDir) || !QDir().mkpath(dir + QStringLiteral("/OBJECT/OBJ")) || !QDir().mkpath(dir + QStringLiteral("/Code"))) { if (error) *error = Utf8("폴더를 만들 수 없습니다: ") + dir; return false; }

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

    if (!WriteText(dir + QStringLiteral("/Code/object.py"), QString::fromUtf8(kTemplate), error)) return false;
    if (!WriteText(dir + QStringLiteral("/main.py"), QString::fromUtf8(kMain).arg(QFileInfo(dir).fileName()), error)) return false;
    if (!WriteRunFiles(dir, error)) return false;
    if (log) *log << Utf8("run.bat / run.sh / requirements.txt 생성 (실행하면 venv 를 만들어 main.py 를 돌립니다)");
    if (!AddModels(dir, urdfs, error, log)) return false;
    if (log) *log << Utf8("만들었습니다: ") + dir;
    return true;
}

namespace {
bool HasCoreLib(const QString& dir) {
    const QStringList names = QDir(dir + QStringLiteral("/Simul")).entryList(QStringList() << QStringLiteral("*simul_core*"), QDir::Files);
    return !names.isEmpty();
}
}  // namespace

bool MainIsOld(const QString& dir) {
    QFile f(dir + QStringLiteral("/main.py"));
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QString text = QString::fromUtf8(f.readAll());
    return !text.contains(QStringLiteral("class Main")) || !text.contains(QStringLiteral("--editor"));
}

// main.py 가 예전 구조이거나, Simul/ 에 Simul.py 나 DLL 이 없는 프로젝트 (예제 프로젝트를 처음 열 때 등)
bool NeedsUpgrade(const QString& dir) {
    QFile f(dir + QStringLiteral("/main.py"));
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QString text = QString::fromUtf8(f.readAll());
    if (!text.contains(QStringLiteral("class Main")) || !text.contains(QStringLiteral("--editor"))) return true;
    QFile sp(dir + QStringLiteral("/Simul/Simul.py"));
    if (!sp.open(QIODevice::ReadOnly) || !QString::fromUtf8(sp.readAll()).contains(QStringLiteral("SIMUL_FEED"))) return true;   // 없거나 에디터 연동 이전 버전
    return !HasCoreLib(dir) || !QFileInfo(dir + QStringLiteral("/run.sh")).isFile();
}

// 예전 구조(함수형)의 프로젝트를 class 구조로 바꾼다. 바꾸는 파일은 .bak 으로 남긴다.
// 직접 고친 Code/<오브젝트>.py 는 건드리지 않는다 (함수형 update(info) 도 그대로 돈다).
bool UpgradeProject(const QString& dirIn, QString* error, QStringList* log) {
    const QString dir = QDir::cleanPath(QFileInfo(dirIn).absoluteFilePath());
    auto backup = [&](const QString& path) {
        QFile::remove(path + QStringLiteral(".bak"));
        QFile::copy(path, path + QStringLiteral(".bak"));
    };
    const QString pySrc = QString::fromUtf8(SIMUL_SOURCE_DIR) + QStringLiteral("/python/Simul/Simul.py");
    const QString pyDst = dir + QStringLiteral("/Simul/Simul.py");
    QDir().mkpath(dir + QStringLiteral("/Simul"));
    if (QFileInfo(pySrc).isFile()) {
        QFile::remove(pyDst);
        QFile::copy(pySrc, pyDst);
        if (log) *log << Utf8("Simul/Simul.py 갱신");
    }
    const char* libs[] = {"simul_core", "simul_gui"};
    for (int i = 0; i < 2; i++) {
        const QStringList found = FindLibs(QString::fromLatin1(libs[i]));
        if (found.isEmpty()) { if (log && i == 0) *log << Utf8("[경고] simul_core 라이브러리를 찾지 못했습니다 (SIMUL_LIB 환경 변수로 알려 주세요)"); continue; }
        const QString to = dir + QStringLiteral("/Simul/") + QFileInfo(found[0]).fileName();
        QFile::remove(to);
        QFile::copy(found[0], to);
        if (log) *log << Utf8("DLL 복사: ") + QFileInfo(found[0]).fileName();
    }
    if (!QFileInfo(dir + QStringLiteral("/run.sh")).isFile() || !QFileInfo(dir + QStringLiteral("/run.bat")).isFile()) {
        if (!WriteRunFiles(dir, error)) return false;
        if (log) *log << Utf8("run.bat / run.sh / requirements.txt 생성");
    }
    if (MainIsOld(dir)) {
        backup(dir + QStringLiteral("/main.py"));
        if (!WriteText(dir + QStringLiteral("/main.py"), QString::fromUtf8(kMain).arg(QFileInfo(dir).fileName()), error)) return false;
        if (log) *log << Utf8("main.py 를 Main class 로 바꿈 (예전 것은 main.py.bak)");
    }
    const QString codeDir = dir + QStringLiteral("/Code");
    {
        QFile of(codeDir + QStringLiteral("/object.py"));
        const bool has = of.open(QIODevice::ReadOnly);
        const bool ok = has && QString::fromUtf8(of.readAll()).contains(QStringLiteral("Simul.Controller"));
        of.close();
        if (!ok) {
            if (has) backup(codeDir + QStringLiteral("/object.py"));
            QDir().mkpath(codeDir);
            if (!WriteText(codeDir + QStringLiteral("/object.py"), QString::fromUtf8(kTemplate), error)) return false;
        }
    }
    const QStringList urdfs = ProjectModels(dir);
    for (int i = 0; i < urdfs.size(); i++) {
        const ModelInfo mi = ReadModelInfo(urdfs[i]);
        const QString py = codeDir + QLatin1Char('/') + mi.object + QStringLiteral(".py");
        QFile pf(py);
        QString cur;
        const bool exists = pf.open(QIODevice::ReadOnly);
        if (exists) { cur = QString::fromUtf8(pf.readAll()); pf.close(); }
        if (exists && cur.contains(QStringLiteral("Simul.Controller"))) continue;
        // 틀 그대로(update 가 None 만 돌려줌)이면 class 틀로 바꾼다. 직접 쓴 코드가 있으면 그대로 둔다.
        QString body = cur;
        body.remove(QRegularExpression(QStringLiteral("\"\"\"[\\s\\S]*?\"\"\"")));
        body.remove(QRegularExpression(QStringLiteral("#[^\n]*")));
        body.replace(QRegularExpression(QStringLiteral("\\s+")), QStringLiteral(" "));
        if (!exists || body.trimmed() == QStringLiteral("def update(info): return None")) {
            if (exists) backup(py);
            const QString joints = mi.joints.isEmpty() ? Utf8("(없음)") : mi.joints.join(QStringLiteral(", "));
            if (!WriteText(py, QString::fromUtf8(kObject).arg(mi.object, joints, ClassName(mi.object)), error)) return false;
            if (log) *log << QStringLiteral("Code/") + mi.object + Utf8(".py 를 class 틀로 바꿈");
        } else if (log) *log << QStringLiteral("Code/") + mi.object + Utf8(".py 는 직접 쓴 코드라 그대로 둠 (함수형 update(info) 도 돌아갑니다. class 로 옮기려면 Code/object.py 견본을 보세요)");
    }
    return true;
}

QString VenvPython(const QString& projectDir) {
#ifdef _WIN32
    const QString p = projectDir + QStringLiteral("/venv/Scripts/python.exe");
#else
    const QString p = projectDir + QStringLiteral("/venv/bin/python");
#endif
    return QFileInfo(p).isFile() ? p : QString();
}

void FindPython(QString* program, QStringList* args, const QString& projectDir) {
    if (!projectDir.isEmpty()) {
        const QString v = VenvPython(projectDir);
        if (!v.isEmpty() && qgetenv("SIMUL_PYTHON").isEmpty()) { args->clear(); *program = v; return; }   // 프로젝트의 venv 가 있으면 그것
    }
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
