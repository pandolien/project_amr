#include "ProjectPack.h"

#ifdef SIMUL_HAS_QT

#include <QByteArray>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>

#include "CProject/Project.h"

namespace projectpack {
using simul::Array;

namespace {

using namespace simul::project;

// exe 의 끝에 패키지가 붙어 있는가. 있으면 패키지의 시작 위치와 크기, 체크섬을 돌려준다.
bool ReadFooter(QFile& f, qint64* payloadStart, unsigned long long* payloadSize, unsigned long long* checksum) {
    const qint64 size = f.size();
    if (size < static_cast<qint64>(kFooterSize)) return false;
    unsigned char foot[kFooterSize];
    if (!f.seek(size - static_cast<qint64>(kFooterSize)) || f.read(reinterpret_cast<char*>(foot), kFooterSize) != static_cast<qint64>(kFooterSize)) return false;
    unsigned long long ps = 0, cs = 0;
    if (!ParseFooter(foot, &ps, &cs)) return false;
    if (ps > static_cast<unsigned long long>(size - static_cast<qint64>(kFooterSize))) return false;
    *payloadSize = ps;
    *checksum = cs;
    *payloadStart = size - static_cast<qint64>(kFooterSize) - static_cast<qint64>(ps);
    return true;
}

QString HexOf(unsigned long long v) { return QString::number(v, 16); }

}  // namespace

bool PackProject(const QString& projectFile, const QString& dataDir, const QString& exeIn, const QString& exeOut, QString* error, int* fileCount) {
    const QFileInfo pf(projectFile);
    if (!pf.isFile()) {
        *error = QStringLiteral("프로젝트 파일이 없습니다: ") + projectFile;
        return false;
    }
    // 프로젝트 파일이 올바른지 먼저 확인한다
    {
        Project p;
        simul::String err;
        if (!LoadProject(simul::String(pf.absoluteFilePath().toUtf8().constData()), &p, &err)) {
            *error = QString::fromUtf8(err.c_str());
            return false;
        }
    }
    const QDir base = pf.absoluteDir();
    const QString outAbs = QFileInfo(exeOut).absoluteFilePath();
    Array<PackedFile> files;
    QDirIterator it(base.absolutePath(), QDir::Files | QDir::NoSymLinks | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        const QString abs = it.next();
        const QString rel = base.relativeFilePath(abs);
        if (rel.startsWith(QStringLiteral(".git/")) || rel.contains(QStringLiteral("/.git/")) || rel.contains(QStringLiteral("__pycache__/"))) continue;
        if (QFileInfo(abs).absoluteFilePath() == outAbs) continue;
        if (rel == QStringLiteral("project.simul") && pf.fileName() != QStringLiteral("project.simul")) continue;   // 다른 이름이면 아래에서 project.simul 로 넣는다
        QFile f(abs);
        if (!f.open(QIODevice::ReadOnly)) {
            *error = QStringLiteral("파일을 읽을 수 없습니다: ") + abs;
            return false;
        }
        PackedFile pk;
        pk.path = simul::String(rel.toUtf8().constData());
        if (!SafeRelativePath(pk.path)) {
            *error = QStringLiteral("패키지에 넣을 수 없는 경로입니다: ") + rel;
            return false;
        }
        const QByteArray bytes = f.readAll();
        pk.data.assign(reinterpret_cast<const unsigned char*>(bytes.constData()), reinterpret_cast<const unsigned char*>(bytes.constData()) + bytes.size());
        files.push_back(pk);
    }
    {   // 프로젝트 파일은 항상 project.simul 로
        bool have = false;
        for (size_t i = 0; i < files.size(); i++) have = have || files[i].path == "project.simul";
        if (!have) {
            QFile f(pf.absoluteFilePath());
            f.open(QIODevice::ReadOnly);
            const QByteArray bytes = f.readAll();
            PackedFile pk;
            pk.path = "project.simul";
            pk.data.assign(reinterpret_cast<const unsigned char*>(bytes.constData()), reinterpret_cast<const unsigned char*>(bytes.constData()) + bytes.size());
            files.push_back(pk);
        }
    }
    {   // 기본 Main.py: 받는 PC 에는 개발용 Data 폴더가 없다
        QFile f(QDir(dataDir).filePath(QStringLiteral("Python/Main.py")));
        if (f.open(QIODevice::ReadOnly)) {
            const QByteArray bytes = f.readAll();
            PackedFile pk;
            pk.path = "__simul/Main.py";
            pk.data.assign(reinterpret_cast<const unsigned char*>(bytes.constData()), reinterpret_cast<const unsigned char*>(bytes.constData()) + bytes.size());
            files.push_back(pk);
        } else {
            *error = QStringLiteral("기본 Main.py 를 찾을 수 없습니다: ") + f.fileName();
            return false;
        }
    }
    {   // 좌표계 화살표 (없어도 선으로 대신 그려지지만 받는 PC 에서도 같은 모양으로)
        const QString names[2] = {QStringLiteral("axes.obj"), QStringLiteral("axes.mtl")};
        for (int k = 0; k < 2; k++) {
            QFile f(QDir(dataDir).filePath(QStringLiteral("Object/gizmo/") + names[k]));
            if (!f.open(QIODevice::ReadOnly)) continue;
            const QByteArray bytes = f.readAll();
            PackedFile pk;
            pk.path = simul::String((QStringLiteral("__simul/Data/Object/gizmo/") + names[k]).toUtf8().constData());
            pk.data.assign(reinterpret_cast<const unsigned char*>(bytes.constData()), reinterpret_cast<const unsigned char*>(bytes.constData()) + bytes.size());
            files.push_back(pk);
        }
    }
    Array<unsigned char> raw;
    SerializePackage(files, &raw);
    const QByteArray packed = qCompress(reinterpret_cast<const uchar*>(raw.data()), static_cast<int>(raw.size()), 6);

    QFile exe(exeIn);
    if (!exe.open(QIODevice::ReadOnly)) {
        *error = QStringLiteral("실행 파일을 읽을 수 없습니다: ") + exeIn;
        return false;
    }
    qint64 start = 0;
    unsigned long long ps = 0, cs = 0;
    qint64 keep = exe.size();
    if (ReadFooter(exe, &start, &ps, &cs)) keep = start;   // 이미 패키지가 붙은 실행 파일이면 그 앞부분만 쓴다
    exe.seek(0);
    const QByteArray head = exe.read(keep);
    exe.close();
    unsigned char foot[kFooterSize];
    MakeFooter(static_cast<size_t>(packed.size()), Checksum(reinterpret_cast<const unsigned char*>(packed.constData()), static_cast<size_t>(packed.size())), foot);
    QFile out(outAbs);
    if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        *error = QStringLiteral("출력 파일을 만들 수 없습니다: ") + outAbs;
        return false;
    }
    out.write(head);
    out.write(packed);
    out.write(reinterpret_cast<const char*>(foot), static_cast<qint64>(kFooterSize));
    out.close();
    out.setPermissions(QFile::permissions(exeIn));   // 실행 권한
    if (fileCount) *fileCount = static_cast<int>(files.size());
    return true;
}

Found ExtractEmbedded(const QString& exePath, QString* dir, QString* error) {
    QFile f(exePath);
    if (!f.open(QIODevice::ReadOnly)) return kNone;
    qint64 start = 0;
    unsigned long long ps = 0, cs = 0;
    if (!ReadFooter(f, &start, &ps, &cs)) return kNone;
    const QString target = QDir::temp().filePath(QStringLiteral("simul_pkg_") + HexOf(cs));
    const QString marker = QDir(target).filePath(QStringLiteral(".complete"));
    if (QFileInfo(marker).isFile()) {   // 같은 패키지를 이미 풀어 두었다
        *dir = target;
        return kExtracted;
    }
    f.seek(start);
    const QByteArray packed = f.read(static_cast<qint64>(ps));
    if (static_cast<unsigned long long>(packed.size()) != ps || Checksum(reinterpret_cast<const unsigned char*>(packed.constData()), static_cast<size_t>(packed.size())) != cs) {
        *error = QStringLiteral("패키지가 손상되었습니다 (체크섬이 맞지 않음)");
        return kFailed;
    }
    const QByteArray raw = qUncompress(packed);
    Array<PackedFile> files;
    simul::String err;
    if (raw.isEmpty() || !ParsePackage(reinterpret_cast<const unsigned char*>(raw.constData()), static_cast<size_t>(raw.size()), &files, &err)) {
        *error = QStringLiteral("패키지를 읽을 수 없습니다: ") + QString::fromUtf8(err.c_str());
        return kFailed;
    }
    QDir(target).removeRecursively();   // 중간에 끊긴 이전 시도가 남아 있으면 지운다
    for (size_t i = 0; i < files.size(); i++) {
        const QString path = QDir(target).filePath(QString::fromUtf8(files[i].path.c_str()));
        QDir().mkpath(QFileInfo(path).absolutePath());
        QFile o(path);
        if (!o.open(QIODevice::WriteOnly | QIODevice::Truncate) || o.write(reinterpret_cast<const char*>(files[i].data.data()), static_cast<qint64>(files[i].data.size())) != static_cast<qint64>(files[i].data.size())) {
            *error = QStringLiteral("풀 수 없습니다: ") + path;
            return kFailed;
        }
    }
    QFile m(marker);
    m.open(QIODevice::WriteOnly);
    m.write("ok");
    *dir = target;
    return kExtracted;
}

}  // namespace projectpack

#endif  // SIMUL_HAS_QT
