#ifndef CORE_CMESH_FILEUTIL_H
#define CORE_CMESH_FILEUTIL_H

// CMesh 내부용 도우미 (외부에 공개하지 않는다)
#include <stdint.h>

#include <fstream>
#include <string>
#include <vector>

namespace amr {
namespace detail {

inline void SetError(std::string* error, const std::string& msg) {
    if (error) *error = msg;
}

// 파일 전체를 읽는다. 실패하면 false.
inline bool ReadFileBytes(const std::string& path, std::vector<char>* out, std::string* error) {
    std::ifstream f(path.c_str(), std::ios::in | std::ios::binary);
    if (!f) {
        SetError(error, "파일을 열 수 없습니다: " + path);
        return false;
    }
    f.seekg(0, std::ios::end);
    const std::streamoff size = f.tellg();
    if (size < 0) {
        SetError(error, "파일 크기를 알 수 없습니다: " + path);
        return false;
    }
    f.seekg(0, std::ios::beg);
    out->assign(static_cast<size_t>(size), 0);
    if (size > 0 && !f.read(&(*out)[0], size)) {
        SetError(error, "파일을 읽는 중 오류가 발생했습니다: " + path);
        return false;
    }
    return true;
}

// 경로에서 폴더 부분을 돌려준다 ("a/b/c.obj" -> "a/b", 폴더가 없으면 "")
inline std::string DirName(const std::string& path) {
    const size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return std::string();
    return path.substr(0, pos);
}

inline std::string JoinPath(const std::string& dir, const std::string& name) {
    if (dir.empty()) return name;
    if (!name.empty() && (name[0] == '/' || name[0] == '\\')) return name;
    const char last = dir[dir.size() - 1];
    if (last == '/' || last == '\\') return dir + name;
    return dir + "/" + name;
}

// 확장자를 소문자로 ("model.GLB" -> "glb"), 없으면 ""
inline std::string LowerExtension(const std::string& path) {
    const size_t slash = path.find_last_of("/\\");
    const size_t dot = path.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return std::string();
    std::string ext = path.substr(dot + 1);
    for (size_t i = 0; i < ext.size(); i++) {
        if (ext[i] >= 'A' && ext[i] <= 'Z') ext[i] = static_cast<char>(ext[i] - 'A' + 'a');
    }
    return ext;
}

}  // namespace detail
}  // namespace amr

#endif  // CORE_CMESH_FILEUTIL_H
