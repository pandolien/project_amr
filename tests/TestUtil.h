// 테스트용 이식성 도우미: 임시 폴더 경로, 폴더 만들기/지우기 (Linux, Windows 공통)
#ifndef TESTS_TESTUTIL_H
#define TESTS_TESTUTIL_H

#include <stdlib.h>

#include "CBase/String.h"

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace tu {

const double kPi = 3.14159265358979323846;

// 임시 폴더 아래 경로. 구분자는 항상 '/' (Windows API 도 허용한다)
inline amr::String Tmp(const amr::String& name) {
    const char* keys[] = {"TMPDIR", "TEMP", "TMP"};
    amr::String base;
    for (size_t i = 0; i < 3 && base.empty(); i++) {
        const char* v = getenv(keys[i]);
        if (v && *v) base = v;
    }
    if (base.empty()) base = "/tmp";
    for (size_t i = 0; i < base.size(); i++) if (base[i] == '\\') base[i] = '/';
    if (base[base.size() - 1] != '/') base += "/";
    return base + name;
}

inline void MkDir(const amr::String& path) {  // 이미 있으면 무시
#ifdef _WIN32
    _mkdir(path.c_str());
#else
    mkdir(path.c_str(), 0755);
#endif
}

inline void RmDir(const amr::String& path) {  // 비어 있을 때만 지워진다
#ifdef _WIN32
    _rmdir(path.c_str());
#else
    rmdir(path.c_str());
#endif
}

}  // namespace tu

#endif  // TESTS_TESTUTIL_H
