#include "CBase/File.h"

#include <stdio.h>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#endif

namespace simul {

bool ReadAllBytes(const String& path, Array<unsigned char>* out, String* err) {
    out->clear();
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) {
        if (err) *err = "cannot open: " + path;
        return false;
    }
    unsigned char buf[65536];
    for (;;) {
        const size_t n = fread(buf, 1, sizeof buf, f);
        if (n == 0) break;
        out->insert(out->end(), buf, buf + n);
    }
    const bool bad = ferror(f) != 0;
    fclose(f);
    if (bad) {
        if (err) *err = "read error: " + path;
        out->clear();
        return false;
    }
    return true;
}

bool WriteFileBytes(const String& path, const void* data, size_t size) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return false;
    const bool ok = size == 0 || fwrite(data, 1, size, f) == size;
    return fclose(f) == 0 && ok;
}

bool RemoveFile(const String& path) { return remove(path.c_str()) == 0; }

bool DirectoryExists(const String& path) {
    if (path.empty()) return false;
    struct stat st;
    if (stat(path.c_str(), &st) != 0) return false;
    return (st.st_mode & S_IFDIR) != 0;
}

bool MakeDirectories(const String& pathIn) {
    String path = pathIn;
    for (size_t i = 0; i < path.size(); i++)
        if (path[i] == '\\') path[i] = '/';
    while (path.size() > 1 && path[path.size() - 1] == '/') path.erase(path.size() - 1);
    if (path.empty()) return false;
    for (size_t i = 1; i <= path.size(); i++) {
        if (i != path.size() && path[i] != '/') continue;
        const String part = path.substr(0, i);
        if (part.empty() || (part.size() == 2 && part[1] == ':') || DirectoryExists(part)) continue;
#ifdef _WIN32
        if (_mkdir(part.c_str()) != 0 && !DirectoryExists(part)) return false;
#else
        if (mkdir(part.c_str(), 0755) != 0 && !DirectoryExists(part)) return false;
#endif
    }
    return DirectoryExists(path);
}

}  // namespace simul
