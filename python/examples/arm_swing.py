# -*- coding: utf-8 -*-
"""arm 을 흔드는 예제. 창을 띄우고 update(info) 가 움직임을 돌려준다.

실행 (저장소 루트에서):
    SIMUL_LIB=build/lib python python/examples/arm_swing.py          # 빌드한 simul_core / simul_gui 가 있는 폴더
    SIMUL_LIB=build/lib python python/examples/arm_swing.py --no-window
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import simul  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))

sim = simul.Simulation()
sim.add_model(os.path.join(ROOT, "Data", "Object", "samples", "arm.urdf"))
if "--no-window" not in sys.argv:
    sim.show("arm_swing")


def update(info):
    """Main(시뮬레이션) 이 정보를 넘기고, 이 함수가 오브젝트 움직임 정보를 돌려준다."""
    if info.collided("arm"):
        print("[t=%.2f] 충돌: %s" % (info.time, info.collided("arm")[0]))
    return {"arm": {"joints": {"base_yaw": 0.8 * math.sin(info.time),
                               "shoulder_pitch": 0.5 + 0.4 * math.sin(2 * info.time)}}}


sim.run(update, dt=0.02, duration=10.0, realtime=True)
print("끝: t=%.2f" % sim.time)
