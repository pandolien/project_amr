#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""새 프로젝트 폴더를 만든다.   python3 tools/new_project.py <폴더> <모델.urdf> [<모델2.urdf> ...] [--lib 빌드/lib]

    <폴더>/main.py                  <- 이것을 실행한다 (DLL 을 불러 시뮬레이션을 돌린다)
    <폴더>/models/<이름>/...        <- URDF 와 그 mesh (원본 폴더째 복사)
    <폴더>/objects/<이름>.py        <- 그 오브젝트의 python 코드 (update(info))
"""
import os
import shutil
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(ROOT, "python"))

MAIN = '''# -*- coding: utf-8 -*-
"""프로젝트 시작 파일.  python main.py  (SIMUL_LIB 에 simul_core/simul_gui 가 있는 폴더, SIMUL_HOME 에 저장소 폴더)"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HOME = os.environ.get("SIMUL_HOME", {home!r})            # simul 패키지(python/simul)가 있는 저장소 폴더
sys.path.insert(0, os.path.join(HOME, "python"))
os.environ.setdefault("SIMUL_LIB", {lib!r})              # 빌드한 DLL 이 있는 폴더 (이미 환경 변수가 있으면 그대로)
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

OBJ = '''# -*- coding: utf-8 -*-
"""오브젝트 "{obj}" 의 python 코드. 매 스텝 update(info) 가 불리고, 이 오브젝트의 움직임을 돌려준다.

info.time, info.dt, info.objects["{obj}"] (.position .quaternion .joints[이름].value/.lower/.upper), info.collided("{obj}")
돌려주는 값: {{"joints": {{관절: 목표 rad 또는 (목표, 속도)}}, "position": (x, y, z), "speed": m/s}}  (없으면 None)
관절: {joints}
"""


def update(info):
    return None
'''


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
    dest, urdfs = os.path.abspath(args[0]), args[1:]
    os.environ["SIMUL_LIB"] = lib
    import simul
    os.makedirs(os.path.join(dest, "objects"), exist_ok=True)
    sim = simul.Simulation()
    lines = []
    for u in urdfs:
        u = os.path.abspath(u)
        src = os.path.dirname(u)
        sub = os.path.splitext(os.path.basename(u))[0]
        dst = os.path.join(dest, "models", sub)
        if not os.path.isdir(dst):
            shutil.copytree(src, dst)
        obj = sim.add_model(os.path.join(dst, os.path.basename(u)))
        joints = ", ".join(sorted(j for j in obj.joints)) or "(없음)"
        pyf = os.path.join(dest, "objects", obj.name + ".py")
        if not os.path.exists(pyf):
            with open(pyf, "w", encoding="utf-8") as f:
                f.write(OBJ.format(obj=obj.name, joints=joints))
        lines.append('sim.add_model(os.path.join(HERE, "models", %r, %r))' % (sub, os.path.basename(u)))
        print("오브젝트 %s: 관절 %s" % (obj.name, joints))
    with open(os.path.join(dest, "main.py"), "w", encoding="utf-8") as f:
        f.write(MAIN.format(home=ROOT, lib=lib, models="\n".join(lines), name=os.path.basename(dest)))
    print("만들었습니다: " + dest)
    return 0


if __name__ == "__main__":
    sys.exit(main())
