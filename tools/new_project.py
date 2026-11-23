#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""새 프로젝트 폴더를 만든다.   python3 tools/new_project.py <폴더> <모델.urdf> [<모델2.urdf> ...] [--lib 빌드/lib]

    <폴더>/main.py                  <- 이것을 실행한다 (DLL 을 불러 시뮬레이션을 돌린다)
    <폴더>/simul/                   <- DLL 과 연결하는 python 파일 (python/simul 복사본) + simul_core / simul_gui DLL
    <폴더>/models/<이름>/...        <- URDF 와 그 mesh (원본 폴더째 복사)
    <폴더>/objects/<이름>.py        <- 그 오브젝트의 python 코드 (update(info))
    <폴더>/requirements.txt         <- PyQt5==5.12.3 (3D 창용 Qt 5.12 를 venv 에 설치)
    <폴더>/setup_env.sh / .bat      <- .venv 를 만들고 설치  /  run.sh / .bat: .venv 로 main.py 실행
"""
import os
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "python"))

MAIN = '''# -*- coding: utf-8 -*-
"""프로젝트 시작 파일.  python main.py   (simul_core / simul_gui DLL 은 simul/ 폴더에 들어 있다. 다른 것을 쓰려면 환경 변수 SIMUL_LIB)"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                                 # HERE/simul : DLL 과 연결하는 python 파일
import simul  # noqa: E402

WINDOW = "--no-window" not in sys.argv
DURATION = None                                          # 초. None 이면 창을 닫을 때까지

sim = simul.Simulation()
{models}
behaviors = simul.Behaviors(os.path.join(HERE, "objects"), sim)   # objects/<이름>.py 의 update(info)
if WINDOW:
    sim.show("{name}")
sim.run(behaviors, dt=0.02, duration=DURATION if WINDOW else (DURATION or 5.0), realtime=WINDOW)
'''

SETUP_SH = """#!/bin/sh
# 3D 창용 Qt 5.12(PyQt5)를 이 프로젝트의 .venv 에 설치한다 (창 없이만 쓰면 필요 없다)
cd "$(dirname "$0")" && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
"""
RUN_SH = """#!/bin/sh
cd "$(dirname "$0")"
if [ -x .venv/bin/python ]; then exec .venv/bin/python main.py "$@"; else exec python3 main.py "$@"; fi
"""
SETUP_BAT = "@echo off\r\ncd /d %~dp0\r\npy -3 -m venv .venv && .venv\\Scripts\\pip install -r requirements.txt\r\n"
RUN_BAT = "@echo off\r\ncd /d %~dp0\r\nif exist .venv\\Scripts\\python.exe (.venv\\Scripts\\python main.py %*) else (py -3 main.py %*)\r\n"

OBJ = '''# -*- coding: utf-8 -*-
"""오브젝트 "{obj}" 의 python 코드. 매 스텝 update(info) 가 불리고, 이 오브젝트의 움직임을 돌려준다.

info.time, info.dt, info.objects["{obj}"] (.position .quaternion .joints[이름].value/.lower/.upper), info.collided("{obj}")
돌려주는 값: {{"joints": {{관절: 목표 rad 또는 (목표, 속도)}}, "position": (x, y, z), "speed": m/s}}  (없으면 None)
관절: {joints}
"""


def update(info):
    return None
'''


def copy_model(urdf, dst):
    """URDF 와 그것이 참조하는 mesh/텍스처 파일만 (상대 경로를 지켜서) 복사한다."""
    import re
    src = os.path.dirname(urdf)
    os.makedirs(dst)
    shutil.copy2(urdf, dst)
    with open(urdf, encoding="utf-8", errors="replace") as f:
        text = f.read()
    for ref in set(re.findall(r'filename\s*=\s*"([^"]+)"', text)):
        rel = re.sub(r"^(package://[^/]*/|file://)", "", ref)
        if os.path.isabs(rel) or ".." in rel.replace("\\", "/").split("/"):
            continue
        sp = os.path.join(src, rel)
        if os.path.isfile(sp):
            os.makedirs(os.path.dirname(os.path.join(dst, rel)), exist_ok=True)
            shutil.copy2(sp, os.path.join(dst, rel))
            if rel.lower().endswith(".obj"):                      # .mtl 이 옆에 있으면 같이
                mtl = os.path.splitext(sp)[0] + ".mtl"
                if os.path.isfile(mtl):
                    shutil.copy2(mtl, os.path.join(dst, os.path.dirname(rel)))


def create_project(dest, urdfs, lib, log=print):
    """프로젝트 폴더를 만든다. 성공하면 0, DLL 을 못 찾으면 1."""
    dest = os.path.abspath(dest)
    import simul
    from simul import _lib
    found = {}
    for base in ("simul_core", "simul_gui"):
        for d in (lib, os.path.join(lib, "..", "bin"), os.path.join(lib, "..", "lib")):
            for n in _lib._names(base):
                if os.path.isfile(os.path.join(d, n)):
                    found.setdefault(base, os.path.join(d, n))
    if "simul_core" not in found:
        log("simul_core DLL 을 찾지 못했습니다. 빌드한 폴더를 --lib 로 알려 주세요 (찾은 위치: %s)" % lib)
        return 1
    os.environ["SIMUL_LIB"] = os.path.dirname(found["simul_core"])
    os.makedirs(os.path.join(dest, "objects"), exist_ok=True)
    pkg = os.path.join(dest, "simul")
    if os.path.isdir(pkg):
        shutil.rmtree(pkg)
    shutil.copytree(os.path.join(ROOT, "python", "simul"), pkg, ignore=shutil.ignore_patterns("__pycache__"))
    for base, path in sorted(found.items()):
        shutil.copy2(path, pkg)
        log("DLL 복사: " + os.path.basename(path))
    if "simul_gui" not in found:
        log("[알림] simul_gui(3D 창)가 없어 창 없이 돌립니다 (Qt 로 빌드하면 생깁니다).")
    sim = simul.Simulation()
    lines = []
    for u in urdfs:
        u = os.path.abspath(u)
        src = os.path.dirname(u)
        sub = os.path.splitext(os.path.basename(u))[0]
        dst = os.path.join(dest, "models", sub)
        if not os.path.isdir(dst):
            copy_model(u, dst)
        obj = sim.add_model(os.path.join(dst, os.path.basename(u)))
        joints = ", ".join(sorted(j for j in obj.joints)) or "(없음)"
        pyf = os.path.join(dest, "objects", obj.name + ".py")
        if not os.path.exists(pyf):
            with open(pyf, "w", encoding="utf-8") as f:
                f.write(OBJ.format(obj=obj.name, joints=joints))
        lines.append('sim.add_model(os.path.join(HERE, "models", %r, %r))' % (sub, os.path.basename(u)))
        log("오브젝트 %s: 관절 %s" % (obj.name, joints))
    with open(os.path.join(dest, "main.py"), "w", encoding="utf-8") as f:
        f.write(MAIN.format(home=ROOT, lib=lib, models="\n".join(lines), name=os.path.basename(dest)))
    with open(os.path.join(dest, "requirements.txt"), "w") as f:
        f.write("PyQt5==5.12.3\n")
    for fn, txt, mode in (("setup_env.sh", SETUP_SH, 0o755), ("run.sh", RUN_SH, 0o755), ("setup_env.bat", SETUP_BAT, 0o644), ("run.bat", RUN_BAT, 0o644)):
        with open(os.path.join(dest, fn), "w", newline="\n" if fn.endswith(".sh") else None) as f:
            f.write(txt)
        os.chmod(os.path.join(dest, fn), mode)
    log("만들었습니다: " + dest)
    return 0


def main():
    args = sys.argv[1:]
    lib = os.path.join(ROOT, "build", "lib")
    if "--lib" in args:
        i = args.index("--lib")
        lib = os.path.abspath(args[i + 1])
        del args[i:i + 2]
    if len(args) < 2:
        print(__doc__)
        return 2
    return create_project(args[0], [os.path.abspath(u) for u in args[1:]], lib)


if __name__ == "__main__":
    sys.exit(main())
