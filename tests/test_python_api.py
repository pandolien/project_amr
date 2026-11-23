# -*- coding: utf-8 -*-
"""python 래퍼(python/Simul/Simul.py)와 simul_core 를 함께 시험한다. 환경 변수 SIMUL_LIB(라이브러리 경로), SIMUL_TEST_DATA_DIR 가 필요하다."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "python", "Simul"))
import Simul as simul  # noqa: E402

DATA = os.environ["SIMUL_TEST_DATA_DIR"]
ARM = os.path.join(DATA, "Object", "samples", "arm.urdf")
MARKER = os.path.join(DATA, "Object", "markers", "aruco_4x4_id0.urdf")
failed = []


def check(cond, msg):
    if not cond:
        failed.append(msg)
        print("FAIL:", msg)


def raises(exc, fn):
    try:
        fn()
    except exc:
        return True
    except Exception:
        return False
    return False


sim = simul.Simulation()
arm = sim.add_model(ARM, position=(0.0, 0.0, 0.0))
check(arm.name == "arm" and arm.kind == "robot" and list(sim.objects) == ["arm"], "model is loaded as object 'arm'")
check(set(arm.joints) == {"base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_roll"}, "joints are listed by name")
check(abs(arm.joints["shoulder_pitch"].upper - 1.57) < 1e-3 and arm.joints["shoulder_pitch"].has_limit, "joint limits are readable")

arm.move_joint("shoulder_pitch", 0.5, 1.0)
check(arm.joints["shoulder_pitch"].value == 0.0, "a command waits for the next step")
for _ in range(100):
    sim.step(0.01)
check(abs(arm.joints["shoulder_pitch"].value - 0.5) < 1e-5 and not arm.moving, "joint reaches its target at the given speed")
check([e.type for e in sim.events()] == ["added", "joint_reached"], "events: added then joint_reached")
check(raises(simul.SimulError, lambda: arm.move_joint("nope", 0.1)), "unknown joint is a SimulError")
check(raises(simul.SimulError, lambda: arm.move_joint("shoulder_pitch", 9.0)), "target beyond the limit is a SimulError")
check(raises(ValueError, lambda: sim.step(0.0)), "dt must be positive")
check(raises(KeyError, lambda: sim["nobody"]), "unknown object is a KeyError")

# update 방식: 정보 묶음을 받고 움직임을 돌려준다
seen = {}


def update(info):
    seen["objects"] = sorted(info.objects)
    seen["dt"] = info.dt
    seen["limit"] = info.objects["arm"].joints["shoulder_pitch"].upper
    return {"arm": {"joints": {"elbow_pitch": 0.8}}}


sim.run(update, dt=0.02, duration=2.0)
check(seen["objects"] == ["arm"] and abs(seen["dt"] - 0.02) < 1e-12 and seen["limit"] > 1.0, "update gets objects, dt and joint limits")
check(abs(arm.joints["elbow_pitch"].value - 0.8) < 1e-5, "returned joint target is applied")
check(raises(TypeError, lambda: sim.apply(5)), "wrong return type is a TypeError")
check(raises(simul.SimulError, lambda: sim.apply({"arm": {"speeed": 1}})), "unknown movement key is a SimulError")

# 충돌
sim.add_model(MARKER, position=(0.3, 0.0, 0.1))
hits = sim.collisions()
check(any(c.involves("arm") and c.involves("aruco_4x4_id0") and not c.same for c in hits), "collision between two objects is reported")
info = sim.info()
check(len(info.collided("arm")) >= 1 and info.collided("nobody") == [], "info.collided filters by object")
sim.remove_model("aruco_4x4_id0")
check(sim.collisions() == [], "no collision after the marker is removed")

# 두 번째 시뮬레이션은 독립이다
sim2 = simul.Simulation()
check(len(sim2.objects) == 0, "a second simulation starts empty")
sim2.close()
sim.close()
if failed:
    print("test_python_api: %d failure(s)" % len(failed))
    sys.exit(1)
print("test_python_api: all passed")
