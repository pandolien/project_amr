#ifndef CORE_CBASE_FILE_H
#define CORE_CBASE_FILE_H

#include <stddef.h>

#include "CBase/Array.h"
#include "CBase/String.h"

// 파일 읽기/쓰기 (ifstream/ofstream 대체, C 의 FILE* 사용). 경로는 바이트 문자열 그대로 운영체제에 넘긴다.
namespace simul {

// 파일 전체를 읽는다. 실패하면 false 와 사유(err, null 가능).
bool ReadAllBytes(const String& path, Array<unsigned char>* out, String* err);
// 파일 전체를 쓴다 (있으면 덮어쓴다).
bool WriteFileBytes(const String& path, const void* data, size_t size);
inline bool WriteFileText(const String& path, const String& text) { return WriteFileBytes(path, text.c_str(), text.size()); }
bool RemoveFile(const String& path);  // 지웠으면 true

}  // namespace simul

#endif  // CORE_CBASE_FILE_H
