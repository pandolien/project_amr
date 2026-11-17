// CBase (std 를 쓰지 않는 기본 도구) 시험. 이 파일도 C++ 표준 라이브러리 헤더를 쓰지 않는다.
#include <stdio.h>
#include <stdlib.h>

#include "CBase/Array.h"
#include "CBase/File.h"
#include "CBase/HashMap.h"
#include "CBase/Map.h"
#include "CBase/Math.h"
#include "CBase/String.h"
#include "CBase/Text.h"
#include "CBase/Util.h"

using namespace simul;

namespace {

int g_fail = 0;
void check(bool ok, const char* what) {
    if (!ok) {
        printf("FAIL: %s\n", what);
        ++g_fail;
    }
}

struct Rng {
    unsigned s;
    explicit Rng(unsigned seed) : s(seed) {}
    unsigned next() {
        s = s * 1664525u + 1013904223u;
        return s >> 8;
    }
};

// 소멸자를 세는 타입: 자원이 새거나 두 번 지워지지 않는지 본다
int g_live = 0;
struct Counted {
    int v;
    Counted() : v(0) { g_live++; }
    Counted(int x) : v(x) { g_live++; }
    Counted(const Counted& o) : v(o.v) { g_live++; }
    Counted& operator=(const Counted& o) {
        v = o.v;
        return *this;
    }
    ~Counted() { g_live--; }
    bool operator<(const Counted& o) const { return v < o.v; }
    bool operator==(const Counted& o) const { return v == o.v; }
};

void testArray() {
    Array<int> a;
    check(a.empty() && a.size() == 0, "new array is empty");
    for (int i = 0; i < 1000; i++) a.push_back(i);
    check(a.size() == 1000 && a[0] == 0 && a[999] == 999 && a.back() == 999 && a.front() == 0, "push_back and index");
    a.push_back(a[3]);  // 자기 원소를 넣어도 (재할당이 일어나도) 안전
    Array<int> grow;
    grow.push_back(7);
    for (int i = 0; i < 40; i++) grow.push_back(grow[0]);
    bool allSeven = true;
    for (size_t i = 0; i < grow.size(); i++) allSeven = allSeven && grow[i] == 7;
    check(a.back() == 3 && allSeven, "push_back of an own element survives reallocation");
    a.pop_back();
    Array<int> b(a);
    check(b == a && b.size() == 1000, "copy equals original");
    b[5] = -1;
    check(!(b == a) && a[5] == 5, "copy is independent");
    Array<int> c(Move(b));
    check(c.size() == 1000 && b.size() == 0 && c[5] == -1, "move leaves the source empty");
    c.erase(c.begin() + 10, c.begin() + 20);
    check(c.size() == 990 && c[9] == 9 && c[10] == 20, "erase range");
    c.erase(c.begin());
    check(c.size() == 989 && c[0] == 1, "erase one");
    c.insert(c.begin() + 2, 77);
    check(c.size() == 990 && c[2] == 77 && c[1] == 2 && c[3] == 3, "insert one");
    const int extra[3] = {10, 11, 12};
    c.insert(c.end(), extra, extra + 3);
    check(c.size() == 993 && c[990] == 10 && c[992] == 12, "insert a pointer range at the end");
    c.insert(c.begin(), c.begin() + 5, c.begin() + 8);  // 자기 자신의 일부를 앞에 넣는다
    check(c.size() == 996 && c[0] == -1 && c[1] == 6 && c[2] == 7 && c[3] == 1 && c[4] == 2, "insert a range from itself");
    Array<int> r(5, 9);
    check(r.size() == 5 && r[4] == 9, "Array(n, value) is count and value, not a range");
    Array<int> z(4);
    check(z.size() == 4 && z[0] == 0 && z[3] == 0, "Array(n) value-initializes");
    z.resize(6);
    check(z.size() == 6 && z[5] == 0, "resize grows with zeros");
    z.resize(2);
    z.resize(5);
    check(z[3] == 0 && z[4] == 0, "grown again after shrinking: new cells are zero");
    z.assign(3, 8);
    check(z.size() == 3 && z[0] == 8, "assign(n, v)");
    z.assign(extra, extra + 3);
    check(z.size() == 3 && z[2] == 12, "assign(range)");
    z.assign(z.begin() + 1, z.end());  // 자기 범위
    check(z.size() == 2 && z[0] == 11, "assign from its own range");
    const int* ep = extra;
    Array<float> fa(ep, ep + 3);
    check(fa.size() == 3 && fa[1] == 11.0f, "range constructor converts the element type");
    z.clear();
    check(z.empty(), "clear");
    Array<bool> flags(5, false);
    flags[2] = true;
    check(!flags[1] && flags[2], "Array<bool> works as a plain array");
    // 반복자 범위 for
    int sum = 0;
    for (int v : r) sum += v;
    check(sum == 45, "range-based for");
    Array<Array<int> > nested;
    nested.push_back(r);
    nested.push_back(r);
    nested[1][0] = 1;
    check(nested.size() == 2 && nested[0][0] == 9 && nested[1][0] == 1, "nested arrays copy deeply");
    // 자원: 소멸자가 정확히 한 번씩 불린다
    {
        Array<Counted> cc;
        for (int i = 0; i < 100; i++) cc.push_back(Counted(i));
        cc.erase(cc.begin() + 3, cc.begin() + 10);
        cc.pop_back();
        cc.resize(20);
        cc.clear();
    }
    check(g_live == 0, "no element is leaked or destroyed twice");
}

void testUniquePtrAndArray() {
    {
        Array<UniquePtr<Counted> > items;
        for (int i = 0; i < 50; i++) items.push_back(UniquePtr<Counted>(new Counted(i)));
        check(items.size() == 50 && items[7]->v == 7, "array of owning pointers");
        UniquePtr<Counted> taken(Move(items[3]));
        check(items[3].get() == nullptr && taken->v == 3, "move out of an element");
        items.erase(items.begin(), items.begin() + 10);
        check(items.size() == 40 && items[0]->v == 10, "erase moves owners down");
        items.clear();
        check(g_live == 1, "clear destroys the owned objects (only the moved-out one is left)");
    }
    check(g_live == 0, "all gone");
    UniquePtr<int> p(new int(5));
    check(*p == 5 && static_cast<bool>(p), "UniquePtr dereference");
    p.reset(new int(6));
    check(*p == 6, "reset");
    int* raw = p.release();
    check(!p && *raw == 6, "release");
    delete raw;
}

void testSortAndUnique() {
    Rng rng(1);
    const int sizes[8] = {0, 1, 2, 5, 12, 13, 100, 1000};
    for (int si = 0; si < 8; si++) {
        const int n = sizes[si];
        Array<int> a;
        for (int i = 0; i < n; i++) a.push_back(static_cast<int>(rng.next() % 50));
        Array<int> s(a);
        Sort(s.begin(), s.end());
        bool ok = true;
        for (size_t i = 1; i < s.size(); i++) ok = ok && !(s[i] < s[i - 1]);
        // 같은 원소들인가: 개수를 센다
        int cnt[50] = {0}, cnt2[50] = {0};
        for (size_t i = 0; i < a.size(); i++) cnt[a[i]]++;
        for (size_t i = 0; i < s.size(); i++) cnt2[s[i]]++;
        for (int k = 0; k < 50; k++) ok = ok && cnt[k] == cnt2[k];
        check(ok, "Sort orders and keeps the same elements");
    }
    // 안정성: 키가 같으면 원래 순서를 지킨다
    struct KV {
        int k, order;
        KV() : k(0), order(0) {}
        KV(int kk, int o) : k(kk), order(o) {}
    };
    struct ByKey {
        bool operator()(const KV& a, const KV& b) const { return a.k < b.k; }
    };
    Array<KV> v;
    for (int i = 0; i < 300; i++) v.push_back(KV(static_cast<int>(rng.next() % 5), i));
    Sort(v.begin(), v.end(), ByKey());
    bool stable = true;
    for (size_t i = 1; i < v.size(); i++) stable = stable && (v[i - 1].k < v[i].k || (v[i - 1].k == v[i].k && v[i - 1].order < v[i].order));
    check(stable, "Sort is stable");
    Array<int> u;
    const int vals[8] = {1, 1, 2, 3, 3, 3, 4, 4};
    u.assign(vals, vals + 8);
    u.erase(Unique(u.begin(), u.end()), u.end());
    check(u.size() == 4 && u[0] == 1 && u[1] == 2 && u[2] == 3 && u[3] == 4, "Unique + erase");
    Array<int> none;
    none.erase(Unique(none.begin(), none.end()), none.end());
    check(none.empty(), "Unique on an empty array");
    Array<Pair<int, int> > ps;
    ps.push_back(MakePair(2, 1));
    ps.push_back(MakePair(1, 9));
    ps.push_back(MakePair(1, 3));
    Sort(ps.begin(), ps.end());
    check(ps[0] == MakePair(1, 3) && ps[1] == MakePair(1, 9) && ps[2] == MakePair(2, 1), "Pair orders by first then second");
    int x = 1, y = 2;
    Swap(x, y);
    check(x == 2 && y == 1 && Min(3, 4) == 3 && Max(3, 4) == 4 && Min(2.5f, 2.5f) == 2.5f, "Swap, Min, Max");
}

void testString() {
    String e;
    check(e.empty() && e.size() == 0 && e.c_str()[0] == '\0', "empty string");
    String a("hello");
    String b = a + " world";
    check(b == "hello world" && b.size() == 11 && b.c_str()[11] == '\0', "concatenate keeps the terminator");
    check(String("a") + String("b") == "ab" && "x" + String("y") == "xy" && String("x") + 'y' == "xy", "operator+ forms");
    b += '!';
    b += "?";
    b += String("#");
    check(b == "hello world!?#", "+= forms");
    check(b.find("world") == 6 && b.find('w') == 6 && b.find("xyz") == String::npos && b.find('o', 5) == 7, "find");
    check(b.rfind('o') == 7 && b.rfind("l") == 9 && b.rfind('z') == String::npos, "rfind");
    check(b.substr(6, 5) == "world" && b.substr(6).size() == 8 && b.substr(100).empty(), "substr");
    check(String("a/b\\c.obj").find_last_of("/\\") == 3 && String("abc").find_last_of("/\\") == String::npos, "find_last_of");
    check(String("  x").find_first_not_of(" ") == 2, "find_first_not_of");
    String c("abcdef");
    c.erase(1, 2);
    check(c == "adef", "erase");
    c.insert(1, "XY");
    check(c == "aXYdef", "insert");
    c.replace(1, 2, "-");
    check(c == "a-def", "replace");
    c.insert(0, c);  // 자기 자신을 넣는다
    check(c == "a-defa-def", "insert itself");
    check(String("a") < String("b") && String("a") < String("ab") && !(String("b") < String("a")) && String("abc").compare("abd") < 0, "ordering");
    // 부호 있는 char 때문에 순서가 뒤집히지 않는다 (바이트 값 순서)
    String hi;
    hi.push_back(static_cast<char>(0xC3));
    check(String("z") < hi, "bytes above 127 sort after ASCII");
    String d(a);
    d[0] = 'J';
    check(a == "hello" && d == "Jello", "copy is independent");
    String m(Move(d));
    check(m == "Jello" && d.empty(), "move leaves the source empty");
    String big(1000, 'x');
    big += big;
    check(big.size() == 2000 && big.c_str()[2000] == '\0', "grows");
    String self("abc");
    self = self.c_str() + 1;  // 자기 안을 가리키는 포인터를 대입
    check(self == "bc", "assign from a pointer into itself");
    String app("ab");
    app.append(app.c_str(), 2);  // 자기 안에서 덧붙인다
    app.append(app);
    check(app == "ababab" + String("ab"), "append from itself");
    check(String((const char*)nullptr).empty(), "null pointer is the empty string");
    check(String("abc", 2) == "ab", "pointer and length");
    String r;
    r.resize(3, 'q');
    check(r == "qqq", "resize grows with a fill character");
    r.resize(1);
    check(r == "q" && r.c_str()[1] == '\0', "resize shrinks");
    check(ToString(42) == "42" && ToString(-7) == "-7" && ToString(static_cast<size_t>(5)) == "5" && ToString(1.5) == "1.5" && ToString(123456789012LL) == "123456789012", "ToString");
    check(Format("%d-%s-%.2f", 3, "x", 1.0) == "3-x-1.00", "Format");
    String longf = Format("%0500d", 7);
    check(longf.size() == 500 && longf[499] == '7', "Format with a result longer than the first buffer");
    // 한글(UTF-8)이 그대로 들어간다
    String ko("관절");
    check(ko.size() == 6 && ko + "!" == "관절!", "UTF-8 bytes are kept");
    Array<String> names;
    names.push_back("b");
    names.push_back("a");
    names.push_back("c");
    Sort(names.begin(), names.end());
    check(names[0] == "a" && names[2] == "c", "Array<String> sorts");
}

void testMapSet() {
    Map<String, int> m;
    m["b"] = 2;
    m["a"] = 1;
    m["c"] = 3;
    check(m.size() == 3 && m["a"] == 1 && m.at("c") == 3, "Map set and get");
    check(m.count("a") == 1 && m.count("z") == 0 && m.find("z") == m.end() && m.find("b")->second == 2, "Map count and find");
    String order;
    for (Map<String, int>::const_iterator it = m.begin(); it != m.end(); ++it) order += it->first;
    check(order == "abc", "Map iterates in key order");
    check(m["zz"] == 0 && m.size() == 4, "operator[] creates a zero value");
    Pair<Map<String, int>::iterator, bool> r = m.insert(MakePair(String("a"), 99));
    check(!r.second && r.first->second == 1, "insert does not overwrite");
    r = m.insert(MakePair(String("d"), 4));
    check(r.second && r.first->second == 4 && m.size() == 5, "insert a new key");
    check(m.erase("a") == 1 && m.erase("a") == 0 && m.size() == 4 && m.find("a") == m.end(), "erase");
    const Map<String, int>& cm = m;
    check(cm.at("b") == 2 && cm.find("b") != cm.end(), "const access");
    // 많이 넣어도 정렬을 유지한다 (무작위 키 5000개를 단순한 배열 방식과 대조)
    Map<int, int> big;
    Rng rng(3);
    Array<int> keys;
    for (int i = 0; i < 5000; i++) {
        const int k = static_cast<int>(rng.next() % 3000);
        big[k] += 1;
        keys.push_back(k);
    }
    Sort(keys.begin(), keys.end());
    bool ok = true;
    size_t distinct = 0, idx = 0;
    for (Map<int, int>::const_iterator it = big.begin(); it != big.end(); ++it) {
        int cnt = 0;
        while (idx < keys.size() && keys[idx] == it->first) {
            cnt++;
            idx++;
        }
        ok = ok && cnt == it->second;
        distinct++;
    }
    check(ok && idx == keys.size() && distinct == big.size(), "Map matches a sorted-array reference");
    Set<int> s;
    check(s.insert(5).second && s.insert(3).second && !s.insert(5).second && s.size() == 2, "Set insert");
    check(s.count(3) == 1 && s.count(4) == 0 && *s.begin() == 3, "Set count and order");
    Set<Pair<int, int> > sp;
    sp.insert(MakePair(1, 2));
    check(sp.count(MakePair(1, 2)) == 1 && sp.count(MakePair(2, 1)) == 0, "Set of pairs");
    check(s.erase(3) == 1 && s.size() == 1, "Set erase");
}

struct Key3 {
    int v[3];
    Key3() { v[0] = v[1] = v[2] = 0; }
    Key3(int a, int b, int c) {
        v[0] = a;
        v[1] = b;
        v[2] = c;
    }
    bool operator==(const Key3& o) const { return v[0] == o.v[0] && v[1] == o.v[1] && v[2] == o.v[2]; }
    size_t Hash() const { return (static_cast<size_t>(v[0]) * 73856093u) ^ (static_cast<size_t>(v[1]) * 19349663u) ^ (static_cast<size_t>(v[2]) * 83492791u); }
};

void testHashMap() {
    HashMap<Key3, unsigned> h;
    check(h.Find(Key3(1, 2, 3)) == nullptr && h.size() == 0, "empty map finds nothing");
    bool ins = false;
    unsigned* p = h.Insert(Key3(1, 2, 3), 10, &ins);
    check(ins && *p == 10 && h.size() == 1, "insert");
    p = h.Insert(Key3(1, 2, 3), 99, &ins);
    check(!ins && *p == 10 && h.size() == 1, "insert keeps the existing value");
    // 20만 개를 넣고 전부 찾는다 (메시 정점 합치기 규모). 해시가 겹치는 키가 많아도 맞아야 한다.
    for (int i = 0; i < 200000; i++) h.Insert(Key3(i % 700, i / 700, -i), static_cast<unsigned>(i));
    bool ok = h.size() == 200001;
    for (int i = 0; i < 200000 && ok; i += 7) {
        unsigned* f = h.Find(Key3(i % 700, i / 700, -i));
        ok = f && *f == static_cast<unsigned>(i);
    }
    check(ok, "200k keys all found with their values");
    check(h.Find(Key3(5, 5, 5)) == nullptr, "an absent key is not found");
    h.clear();
    check(h.size() == 0 && h.Find(Key3(1, 2, 3)) == nullptr, "clear");
}

void testMathAndText() {
    check(Abs(-2.5f) == 2.5f && Abs(-2.5) == 2.5 && Sqrt(9.0f) == 3.0f && Floor(-1.5f) == -2.0f && Lround(2.5) == 3 && Lround(-2.5) == -3 && Lround(2.4) == 2, "math wrappers");
    check(IsFinite(1.0) && IsFinite(0.0f) && !IsFinite(NanFloat()) && !IsFinite(INFINITY) && !IsFinite(-INFINITY) && IsNan(NanFloat()) && !IsNan(1.0f), "IsFinite / IsNan");
    check(Abs(Sin(0.5) * Sin(0.5) + Cos(0.5) * Cos(0.5) - 1.0) < 1e-15 && Abs(Atan2(1.0, 1.0) - 0.78539816339744828) < 1e-15, "trig");
    double d;
    const char* end;
    check(ParseDouble("3.14", &d, &end) && d == 3.14 && *end == '\0', "ParseDouble 3.14 is exact");
    check(ParseDouble("  -0.5e2xyz", &d, &end) && d == -50.0 && String(end) == "xyz", "sign, exponent, stops at junk");
    check(ParseDouble("1e-3", &d, &end) && d == 1e-3, "1e-3");
    check(ParseDouble(".5", &d, &end) && d == 0.5 && ParseDouble("7.", &d, &end) && d == 7.0, ".5 and 7.");
    check(ParseDouble("0.1", &d, &end) && d == 0.1 && ParseDouble("123456.789", &d, &end) && d == 123456.789, "common decimals are correctly rounded");
    check(ParseDouble("1.7976931348623157e308", &d, &end) && d == 1.7976931348623157e308, "DBL_MAX");
    check(ParseDouble("0.000000000000000000001234", &d, &end) && Abs(d / 1.234e-21 - 1.0) < 1e-14, "tiny values");
    check(ParseDouble("12345678901234567890123", &d, &end) && Abs(d / 1.2345678901234568e22 - 1.0) < 1e-15, "more than 19 digits");
    check(ParseDouble("-inf", &d, &end) && d < 0 && !IsFinite(d) && ParseDouble("NaN", &d, &end) && IsNan(d), "inf and nan");
    check(!ParseDouble("abc", &d, &end) && end != nullptr && String(end) == "abc" && !ParseDouble("", &d, &end) && !ParseDouble("-", &d, &end) && !ParseDouble("e5", &d, &end), "not numbers");
    check(ParseDouble("1e", &d, &end) && d == 1.0 && String(end) == "e", "an incomplete exponent is left alone");
    // 무작위 값: printf 로 만든 문자열을 다시 읽으면 float 로는 항상 같아야 한다
    Rng rng(9);
    bool rt = true;
    for (int i = 0; i < 20000; i++) {
        const float f = static_cast<float>(static_cast<int>(rng.next() % 2000001) - 1000000) / static_cast<float>(1 + rng.next() % 1000) * 1e-2f;
        const String s = Format("%.9g", static_cast<double>(f));
        double back;
        const char* e2;
        rt = rt && ParseDouble(s.c_str(), &back, &e2) && static_cast<float>(back) == f;
    }
    check(rt, "float values survive print and parse");
    long li;
    check(ParseInt("-42abc", &li, &end) && li == -42 && String(end) == "abc" && !ParseInt("x", &li, &end), "ParseInt");
    check(ToDouble(" 2.5 ", &d) && d == 2.5 && !ToDouble("2.5x", &d) && !ToDouble("", &d), "ToDouble needs the whole string");
    check(ToInt("12", &li) && li == 12 && !ToInt("1.5", &li), "ToInt");
    check(Trim("  a b \n") == "a b" && Trim("   ").empty() && ToLower("AbC.GLB") == "abc.glb" && StartsWith("solid x", "solid") && !StartsWith("so", "solid"), "Trim, ToLower, StartsWith");
    Tokens t("  v 1.5  -2 x\t3 ");
    String tok;
    float f;
    int n;
    check(t.Next(&tok) && tok == "v" && t.NextFloat(&f) && f == 1.5f && t.NextInt(&n) && n == -2, "Tokens");
    check(!t.NextFloat(&f) && t.Next(&tok) && tok == "x", "a failed NextFloat does not consume the token");
    check(t.Rest() == "3" && t.AtEnd() && !t.Next(&tok), "Rest and end");
    Tokens t2("newmtl  my material  \r");
    t2.Next(&tok);
    check(t2.Rest() == "my material", "Rest keeps inner spaces and trims both ends");
    Lines ls("a\nb\r\n\nlast");
    String line;
    check(ls.Next(&line) && line == "a" && ls.Next(&line) && line == "b" && ls.Next(&line) && line.empty() && ls.Next(&line) && line == "last" && !ls.Next(&line), "Lines handles \\n, \\r\\n, blank lines and a missing final newline");
    Lines ls2("x\n");
    check(ls2.Next(&line) && line == "x" && !ls2.Next(&line), "a trailing newline does not make an extra empty line");
}

void testFile() {
    const String path = "simul_cbase_test.bin";
    unsigned char data[300];
    for (int i = 0; i < 300; i++) data[i] = static_cast<unsigned char>(i);
    check(WriteFileBytes(path, data, 300), "write a file");
    Array<unsigned char> back;
    String err;
    check(ReadAllBytes(path, &back, &err) && back.size() == 300 && back[0] == 0 && back[255] == 255 && back[299] == 43, "read it back");
    Array<unsigned char> big;
    Array<unsigned char> chunk(200000, 7);
    check(WriteFileBytes(path, chunk.data(), chunk.size()) && ReadAllBytes(path, &big, &err) && big.size() == 200000 && big[199999] == 7, "a file bigger than the read buffer");
    check(WriteFileBytes(path, nullptr, 0) && ReadAllBytes(path, &back, &err) && back.empty(), "an empty file");
    check(RemoveFile(path) && !RemoveFile(path), "remove");
    check(!ReadAllBytes(path, &back, &err) && !err.empty() && back.empty(), "missing file reports an error");
    check(!WriteFileBytes("no_such_dir_xyz/a.bin", data, 3), "writing into a missing folder fails");
}

}  // namespace

int main() {
    testArray();
    testUniquePtrAndArray();
    testSortAndUnique();
    testString();
    testMapSet();
    testHashMap();
    testMathAndText();
    testFile();
    if (g_fail == 0) printf("test_cbase: all passed\n");
    return g_fail == 0 ? 0 : 1;
}
