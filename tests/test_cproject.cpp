// CProject 테스트: project.simul 읽기/쓰기, 경로 검사, 패키지 직렬화와 꼬리표
#include <math.h>
#include <stdio.h>

#include "CProject/Project.h"

using namespace simul;
using namespace simul::project;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

void testProject() {
    Project p;
    String err;
    const char* xml =
        "<simul_project version=\"1\" name=\"demo &amp; co\">\n"
        "  <model file=\"models/arm.urdf\" x=\"1\" y=\"2\" z=\"0.5\" yaw=\"1.5708\"/>\n"
        "  <model file=\"models/box.urdf\"/>\n"
        "  <script file=\"scripts/pick.py\" objects=\"arm,box\"/>\n"
        "  <script file=\"scripts/other.py\"/>\n"
        "  <python hz=\"30\" autostart=\"false\" editor=\"true\" main=\"scripts/Main.py\"/>\n"
        "  <future thing=\"1\"/>\n"
        "</simul_project>\n";
    check(ParseProject(xml, &p, &err), "parse");
    check(p.name == "demo & co" && p.models.size() == 2 && p.scripts.size() == 2, "counts and entity decoding");
    check(p.models[0].file == "models/arm.urdf" && fabs(p.models[0].position[1] - 2.0) < 1e-12 && fabs(p.models[0].rpy[2] - 1.5708) < 1e-12 && p.models[0].Placed(), "placement");
    check(!p.models[1].Placed(), "no placement attributes");
    check(p.scripts[0].objects == "arm,box" && p.scripts[1].objects.empty(), "script grants");
    check(fabs(p.hz - 30.0) < 1e-12 && !p.autostart && p.showEditor && p.mainPy == "scripts/Main.py", "python settings");
    // 쓰고 다시 읽으면 같다
    Project q;
    check(ParseProject(WriteProjectText(p), &q, &err), "re-parse written text");
    check(q.name == p.name && q.models.size() == 2 && q.scripts.size() == 2 && fabs(q.models[0].rpy[2] - 1.5708) < 1e-9 && q.mainPy == p.mainPy && q.hz == p.hz && q.autostart == p.autostart, "round trip");
    // 기본값
    check(ParseProject("<simul_project/>", &q, &err) && q.hz == 60.0 && q.autostart && !q.showEditor, "defaults");
    // 오류
    check(!ParseProject("<robot/>", &q, &err) && !err.empty(), "wrong root");
    check(!ParseProject("<simul_project><model file=\"../x.urdf\"/></simul_project>", &q, &err), "path escaping the folder is rejected");
    check(!ParseProject("<simul_project><model file=\"/etc/x.urdf\"/></simul_project>", &q, &err), "absolute path rejected");
    check(!ParseProject("<simul_project><model file=\"C:/x.urdf\"/></simul_project>", &q, &err), "drive path rejected");
    check(!ParseProject("<simul_project><model file=\"a.urdf\" x=\"abc\"/></simul_project>", &q, &err), "non numeric attribute");
    check(!ParseProject("<simul_project><python hz=\"0\"/></simul_project>", &q, &err), "hz range");
    check(!ParseProject("<simul_project><python autostart=\"yes\"/></simul_project>", &q, &err), "bool attribute");
    check(!ParseProject("<simul_project><model", &q, &err), "broken xml");
}

void testPaths() {
    check(SafeRelativePath("a/b/c.py") && SafeRelativePath("x.urdf"), "normal paths");
    check(!SafeRelativePath("") && !SafeRelativePath("a//b") && !SafeRelativePath("a/../b") && !SafeRelativePath("..") && !SafeRelativePath("a\\b") && !SafeRelativePath("/a"), "bad paths");
}

void testPackage() {
    Array<PackedFile> in;
    PackedFile a, b;
    a.path = "project.simul";
    a.data.push_back('x');
    b.path = "models/m.stl";
    for (int i = 0; i < 1000; i++) b.data.push_back(static_cast<unsigned char>(i * 7));
    in.push_back(a);
    in.push_back(b);
    Array<unsigned char> blob;
    SerializePackage(in, &blob);
    Array<PackedFile> out;
    String err;
    check(ParsePackage(blob.data(), blob.size(), &out, &err) && out.size() == 2 && out[0].path == "project.simul" && out[1].data.size() == 1000 && out[1].data[999] == static_cast<unsigned char>(999 * 7), "package round trip");
    check(!ParsePackage(blob.data(), blob.size() - 1, &out, &err), "truncated package rejected");
    check(!ParsePackage(blob.data(), 5, &out, &err), "tiny package rejected");
    Array<unsigned char> bad = blob;
    bad[0] = 'X';
    check(!ParsePackage(bad.data(), bad.size(), &out, &err), "wrong magic rejected");
    // 경로가 안전하지 않은 패키지는 풀지 않는다
    Array<PackedFile> evil;
    PackedFile e;
    e.path = "a/../../x";
    evil.push_back(e);
    SerializePackage(evil, &blob);
    check(!ParsePackage(blob.data(), blob.size(), &out, &err), "unsafe path in a package rejected");
    // 꼬리표
    unsigned char foot[kFooterSize];
    MakeFooter(123456, 0xdeadbeefcafeULL, foot);
    unsigned long long sz = 0, cs = 0;
    check(ParseFooter(foot, &sz, &cs) && sz == 123456 && cs == 0xdeadbeefcafeULL, "footer round trip");
    foot[20] = 'Z';
    check(!ParseFooter(foot, &sz, &cs), "footer without the magic");
    const unsigned char d1[3] = {1, 2, 3}, d2[3] = {1, 2, 4};
    check(Checksum(d1, 3) != Checksum(d2, 3) && Checksum(d1, 3) == Checksum(d1, 3), "checksum");
}

}  // namespace

int main() {
    testProject();
    testPaths();
    testPackage();
    if (g_fail) {
        printf("test_cproject: %d failure(s)\n", g_fail);
        return 1;
    }
    printf("test_cproject: all passed\n");
    return 0;
}
