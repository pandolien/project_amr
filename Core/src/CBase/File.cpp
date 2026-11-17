#include "CBase/File.h"

#include <stdio.h>

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

}  // namespace simul
