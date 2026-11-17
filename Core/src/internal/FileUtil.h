#ifndef CORE_CMESH_FILEUTIL_H
#define CORE_CMESH_FILEUTIL_H

// CMesh 내부용 도우미 (외부에 공개하지 않는다)
#include <stdint.h>

#include "CBase/File.h"
#include "CBase/String.h"
#include "CBase/Array.h"

namespace simul {
namespace detail {

inline void SetError(simul::String* error, const simul::String& msg) {
    if (error) *error = msg;
}

// 파일 전체를 읽는다. 실패하면 false.
inline bool ReadFileBytes(const simul::String& path, simul::Array<char>* out, simul::String* error) {
    simul::Array<unsigned char> bytes;
    simul::String why;
    if (!simul::ReadAllBytes(path, &bytes, &why)) {
        SetError(error, "파일을 열 수 없습니다: " + path);
        out->clear();
        return false;
    }
    out->assign(reinterpret_cast<const char*>(bytes.begin()), reinterpret_cast<const char*>(bytes.end()));
    return true;
}

// 경로에서 폴더 부분을 돌려준다 ("a/b/c.obj" -> "a/b", 폴더가 없으면 "")
inline simul::String DirName(const simul::String& path) {
    const size_t pos = path.find_last_of("/\\");
    if (pos == simul::String::npos) return simul::String();
    return path.substr(0, pos);
}

inline simul::String JoinPath(const simul::String& dir, const simul::String& name) {
    if (dir.empty()) return name;
    if (!name.empty() && (name[0] == '/' || name[0] == '\\')) return name;
    const char last = dir[dir.size() - 1];
    if (last == '/' || last == '\\') return dir + name;
    return dir + "/" + name;
}

// 확장자를 소문자로 ("model.GLB" -> "glb"), 없으면 ""
inline simul::String LowerExtension(const simul::String& path) {
    const size_t slash = path.find_last_of("/\\");
    const size_t dot = path.find_last_of('.');
    if (dot == simul::String::npos || (slash != simul::String::npos && dot < slash)) return simul::String();
    simul::String ext = path.substr(dot + 1);
    for (size_t i = 0; i < ext.size(); i++) {
        if (ext[i] >= 'A' && ext[i] <= 'Z') ext[i] = static_cast<char>(ext[i] - 'A' + 'a');
    }
    return ext;
}

}  // namespace detail
}  // namespace simul

#endif  // CORE_CMESH_FILEUTIL_H
