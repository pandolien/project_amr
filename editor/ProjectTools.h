#ifndef SIMUL_EDITOR_PROJECTTOOLS_H
#define SIMUL_EDITOR_PROJECTTOOLS_H

#include <QString>
#include <QStringList>

// 프로젝트 폴더 만들기와 python 찾기 (Qt 만 쓴다).
//   <폴더>/main.py                 시작 파일 (python main.py)
//   <폴더>/Simul/Simul.py          DLL 을 부르는 python 파일 + 같은 폴더에 simul_core / simul_gui DLL(so)
//   <폴더>/OBJECT/<이름>.urdf      URDF (mesh 는 OBJECT/OBJ/ 에 모은다)
//   <폴더>/Code/<오브젝트>.py      그 오브젝트의 python 코드: def update(info)
namespace projecttools {

struct ModelInfo {
    QString object;        // python 에서 쓰는 오브젝트 이름 (= URDF 파일 이름, 확장자 제외)
    QStringList joints;    // fixed 가 아닌 관절 이름
};

bool IsProject(const QString& dir);
ModelInfo ReadModelInfo(const QString& urdfPath);
// urdfs 를 OBJECT/ 로 복사(참조하는 mesh 는 OBJECT/OBJ/)하고 위 구조를 만든다. 실패하면 false 와 사유.
bool CreateProject(const QString& dir, const QStringList& urdfs, QString* error, QStringList* log);
// 이 프로젝트가 쓸 python 실행 파일(과 앞에 붙일 인자). 못 찾으면 program 이 빈 문자열.
void FindPython(QString* program, QStringList* args);
// python 에 줄 환경 (SIMUL_LIB = 프로젝트의 simul 폴더)
QStringList ProjectModels(const QString& dir);   // OBJECT/ 아래의 .urdf 전체 경로

}  // namespace projecttools

#endif
