// 프로젝트: 장면(모델), 등록할 python 파일과 권한, python 설정을 한 파일(project.simul, XML)로 묶는다.
//   그리고 프로젝트 폴더를 하나의 "패키지"로 묶고 푸는 직렬화 (실행 파일 뒤에 붙여서 하나의 프로그램으로 만든다).
// Qt, Python 에 의존하지 않는다. 경로는 모두 project.simul 이 있는 폴더 기준 상대 경로 ('/' 구분).
//
//   <simul_project version="1" name="demo">
//     <model file="models/arm.urdf" x="0" y="0" z="0" roll="0" pitch="0" yaw="1.5708"/>   <!-- 위치 m, 자세 rad (생략하면 0) -->
//     <script file="scripts/pick.py" objects="arm,box"/>      <!-- 이 파일의 클래스가 움직일 수 있는 오브젝트 (비우면 파일 안의 objects) -->
//     <python hz="60" autostart="true" editor="false" main=""/>   <!-- 틱 수, 열자마자 시작, 편집기 도크, 다른 Main.py -->
//   </simul_project>
#ifndef CORE_CPROJECT_PROJECT_H
#define CORE_CPROJECT_PROJECT_H

#include <stddef.h>

#include "CBase/Array.h"
#include "CBase/String.h"

namespace simul {
namespace project {

struct ModelEntry {
    String file;
    double position[3];   // m
    double rpy[3];        // rad
    ModelEntry() { position[0] = position[1] = position[2] = 0; rpy[0] = rpy[1] = rpy[2] = 0; }
    bool Placed() const { for (int i = 0; i < 3; i++) if (position[i] != 0 || rpy[i] != 0) return true; return false; }
};

struct ScriptEntry {
    String file;
    String objects;   // 쉼표로 나눈 이름. 비면 파일 안의 objects. "*" 는 전부
};

struct Project {
    String name;
    Array<ModelEntry> models;
    Array<ScriptEntry> scripts;
    double hz;            // 초당 틱 수 (1 ~ 1000)
    bool autostart;       // 열자마자 시뮬레이션을 시작한다
    bool showEditor;      // Python 편집기 도크를 연다
    String mainPy;        // 비면 기본 Data/Python/Main.py
    Project() : hz(60.0), autostart(true), showEditor(false) {}
};

bool ParseProject(const String& xml, Project* out, String* error);
String WriteProjectText(const Project& p);
bool LoadProject(const String& path, Project* out, String* error);
bool SaveProject(const String& path, const Project& p, String* error);

// ---- 패키지 ----
struct PackedFile {
    String path;                    // 상대 경로 ('/'). ".." 나 절대 경로는 허용하지 않는다
    Array<unsigned char> data;
};

// 상대 경로로 쓸 수 있는가 (비어 있지 않고, 절대 경로나 드라이브 문자, "..", 역슬래시가 없다)
bool SafeRelativePath(const String& path);

// 파일들을 하나의 바이트열로 직렬화 / 복원한다 (경로가 안전하지 않으면 거부)
void SerializePackage(const Array<PackedFile>& files, Array<unsigned char>* out);
bool ParsePackage(const unsigned char* data, size_t size, Array<PackedFile>* out, String* error);

// 실행 파일 뒤에 붙이는 꼬리표 (24 바이트): 패키지 크기(8), 체크섬(8), 매직 "SIMULPK1"(8). 리틀 엔디언.
const size_t kFooterSize = 24;
void MakeFooter(size_t payloadSize, unsigned long long checksum, unsigned char out[kFooterSize]);
bool ParseFooter(const unsigned char in[kFooterSize], unsigned long long* payloadSize, unsigned long long* checksum);
unsigned long long Checksum(const unsigned char* data, size_t size);   // FNV-1a 64

}  // namespace project
}  // namespace simul

#endif  // CORE_CPROJECT_PROJECT_H
