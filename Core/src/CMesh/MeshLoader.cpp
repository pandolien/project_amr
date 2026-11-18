#include "CMesh/MeshLoader.h"

#include <sys/stat.h>
#include <sys/types.h>

#include "CBase/Array.h"
#include "FileUtil.h"

namespace simul {

bool LoadMesh(const simul::String& path, Scene* out, simul::String* error) {
    const simul::String ext = detail::LowerExtension(path);
    if (ext == "glb" || ext == "gltf") return LoadGlb(path, out, error);
    if (ext == "stl") return LoadStl(path, out, error);
    if (ext == "obj") return LoadObj(path, out, error);
    if (ext == "dae") return LoadDae(path, out, error);
    if (out) out->Clear();
    detail::SetError(error, "지원하지 않는 형식입니다 (.glb .gltf .stl .obj .dae): " + path);
    return false;
}

namespace {

struct CacheEntry {
    simul::String path;
    long long size, mtime;
    Scene scene;
    size_t bytes;
    CacheEntry() : size(0), mtime(0), bytes(0) {}
};

simul::Array<CacheEntry>& Cache() {
    static simul::Array<CacheEntry> c;
    return c;
}

bool FileStamp(const simul::String& path, long long* size, long long* mtime) {
#ifdef _WIN32
    struct _stat64 st;
    if (_stat64(path.c_str(), &st) != 0) return false;
#else
    struct stat st;
    if (stat(path.c_str(), &st) != 0) return false;
#endif
    *size = static_cast<long long>(st.st_size);
    *mtime = static_cast<long long>(st.st_mtime);
    return true;
}

size_t SceneBytes(const Scene& s) {
    size_t b = 0;
    for (size_t m = 0; m < s.meshes.size(); m++)
        for (size_t p = 0; p < s.meshes[m].primitives.size(); p++) {
            const Primitive& pr = s.meshes[m].primitives[p];
            b += (pr.positions.size() + pr.normals.size() + pr.uvs.size() + pr.colors.size()) * sizeof(float) + pr.indices.size() * sizeof(uint32_t);
        }
    for (size_t i = 0; i < s.materials.size(); i++) b += s.materials[i].baseColorImage.size();
    return b;
}

const size_t kCacheBudget = static_cast<size_t>(1) << 30;

}  // namespace

bool LoadMeshCached(const simul::String& path, Scene* out, simul::String* error) {
    Scene local;   // out 이 NULL 이면 캐시에만 채운다 (읽을 수 있는 파일인지 미리 보는 용도)
    if (!out) out = &local;
    long long size = 0, mtime = 0;
    const bool stamped = FileStamp(path, &size, &mtime);
    simul::Array<CacheEntry>& cache = Cache();
    if (stamped) {
        for (size_t i = 0; i < cache.size(); i++) {
            if (cache[i].path == path && cache[i].size == size && cache[i].mtime == mtime) {
                *out = cache[i].scene;
                return true;
            }
        }
    }
    if (!LoadMesh(path, out, error)) return false;
    if (!stamped) return true;
    for (size_t i = 0; i < cache.size(); i++) {   // 바뀐 파일의 옛 항목은 버린다
        if (cache[i].path == path) {
            cache.erase(cache.begin() + static_cast<ptrdiff_t>(i));
            break;
        }
    }
    const size_t bytes = SceneBytes(*out);
    if (bytes > kCacheBudget) return true;   // 너무 커서 두지 않는다
    size_t total = bytes;
    for (size_t i = 0; i < cache.size(); i++) total += cache[i].bytes;
    while (total > kCacheBudget && !cache.empty()) {
        total -= cache[0].bytes;
        cache.erase(cache.begin());
    }
    CacheEntry e;
    e.path = path;
    e.size = size;
    e.mtime = mtime;
    e.scene = *out;
    e.bytes = bytes;
    cache.push_back(e);
    return true;
}

void ClearMeshCache() { Cache().clear(); }

}  // namespace simul
