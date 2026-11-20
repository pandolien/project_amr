// 프로젝트를 실행 파일 하나로 묶고(PackProject), 묶인 실행 파일이 시작할 때 자기 안의 프로젝트를 푸는(ExtractEmbedded) 부분.
//   묶는 방식: 실행 파일 뒤에 [압축한 패키지][꼬리표 24바이트]를 붙인다. 운영체제는 뒤에 붙은 데이터를 무시하므로 그대로 실행된다.
//   Qt 에 의존하므로 SIMUL_HAS_QT 일 때만 컴파일된다.
#ifndef PROJECTPACK_H
#define PROJECTPACK_H

#ifdef SIMUL_HAS_QT

#include <QString>

namespace projectpack {

// projectFile(project.simul) 이 있는 폴더 전체 + 기본 Main.py(dataDir/Python/Main.py) 를 exeIn 뒤에 붙여 exeOut 으로 쓴다.
// exeIn 에 이미 패키지가 붙어 있으면 떼어낸다. 프로젝트 폴더 안의 exeOut 과 .git 은 넣지 않는다.
bool PackProject(const QString& projectFile, const QString& dataDir, const QString& exeIn, const QString& exeOut, QString* error, int* fileCount = nullptr);

enum Found { kNone, kExtracted, kFailed };
// exePath 뒤에 붙은 패키지가 있으면 임시 폴더(같은 패키지는 한 번만)에 풀어서 그 폴더를 돌려준다. 없으면 kNone, 있는데 풀지 못하면 kFailed + error.
Found ExtractEmbedded(const QString& exePath, QString* dir, QString* error);

}  // namespace projectpack

#endif  // SIMUL_HAS_QT
#endif  // PROJECTPACK_H
