#include "CMesh/MeshLoader.h"

#include "FileUtil.h"

namespace amr {

bool LoadMesh(const std::string& path, Scene* out, std::string* error) {
    const std::string ext = detail::LowerExtension(path);
    if (ext == "glb" || ext == "gltf") return LoadGlb(path, out, error);
    if (ext == "stl") return LoadStl(path, out, error);
    if (ext == "obj") return LoadObj(path, out, error);
    if (ext == "dae") return LoadDae(path, out, error);
    if (out) out->Clear();
    detail::SetError(error, "지원하지 않는 형식입니다 (.glb .gltf .stl .obj .dae): " + path);
    return false;
}

}  // namespace amr
