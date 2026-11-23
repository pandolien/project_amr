# -*- coding: utf-8 -*-
"""Project Simul 의 python 인터페이스 (simul_core 공유 라이브러리 위의 얇은 래퍼).

    import simul
    sim = simul.Simulation()
    arm = sim.add_model("arm.urdf", position=(0, 0, 0))
    arm.move_joint("joint_1", 1.57, speed=0.5)         # 명령은 다음 step 시작에 적용된다
    for _ in range(300):
        sim.step(0.01)                                  # 0.01 s 진행
    print(arm.joints["joint_1"].value, sim.time)

update 방식으로 돌리려면 (정보를 받고 움직임을 돌려주는 함수):

    def update(info):                                   # info.time, info.dt, info.objects, info.collisions
        return {"arm": {"joints": {"joint_1": 1.57}}}
    sim.run(update, dt=0.01, duration=5.0)

단위: 길이 m, 각도 rad, 시간 s, 자세 쿼터니언 (x, y, z, w).
"""
import ctypes
import math
import time as _time

from . import _lib

__all__ = ["Simulation", "SimulError", "Object", "Joint", "Contact", "Event", "Info", "ObjectState", "JointState"]

_EVENT_NAMES = {0: "added", 1: "removed", 2: "arrived", 3: "joint_reached", 4: "command_failed"}
_KIND_NAMES = {1: "revolute", 2: "prismatic", 3: "continuous"}


from .behaviors import Behaviors  # noqa: E402,F401

import atexit as _atexit
import weakref as _weakref

_live = _weakref.WeakSet()
_exit_registered = False


def _close_all():
    """프로세스가 끝나기 전에 창과 시뮬레이션을 먼저 닫는다 (PyQt5 가 QApplication 을 지운 뒤에는 창을 닫을 수 없다)."""
    for sim in list(_live):
        try:
            sim.close()
        except Exception:
            pass


class SimulError(Exception):
    """명령이나 모델 읽기가 거부되었다 (메시지에 이유)."""


def _b(s):
    return s.encode("utf-8")


def _s(b):
    return b.decode("utf-8") if b else ""


class Joint(object):
    """오브젝트의 관절 하나 (지금 값을 읽는다). 값을 바꾸는 것은 Object.set_joint / move_joint 로."""

    def __init__(self, sim, obj, index, name):
        self._sim, self._obj, self._index, self.name = sim, obj, index, name

    def _info(self):
        arr = (ctypes.c_double * 6)()
        if not self._sim._lib.simul_joint_info(self._sim._h, self._obj._index(), self._index, arr):
            raise SimulError(self._sim._error())
        return arr

    value = property(lambda self: self._info()[0])
    lower = property(lambda self: self._info()[1])
    upper = property(lambda self: self._info()[2])
    velocity = property(lambda self: self._info()[3])          # 최대 속도 (없으면 0)
    has_limit = property(lambda self: bool(self._info()[4]))
    kind = property(lambda self: _KIND_NAMES.get(int(self._info()[5]), "fixed"))

    def __repr__(self):
        return "<Joint %s %.4f>" % (self.name, self.value)


class Object(object):
    """장면의 오브젝트. 읽는 값은 지금(마지막 step 뒤) 상태이고, 명령은 다음 step 에 적용된다."""

    def __init__(self, sim, name):
        self._sim = sim
        self.name = name

    def _index(self):
        i = self._sim._lib.simul_find_object(self._sim._h, _b(self.name))
        if i < 0:
            raise SimulError("오브젝트 '%s' 가 없습니다" % self.name)
        return i

    alive = property(lambda self: self._sim._lib.simul_find_object(self._sim._h, _b(self.name)) >= 0)
    kind = property(lambda self: _s(self._sim._lib.simul_object_kind(self._sim._h, self._index())))
    path = property(lambda self: _s(self._sim._lib.simul_object_path(self._sim._h, self._index())))

    def pose(self):
        arr = (ctypes.c_double * 7)()
        if not self._sim._lib.simul_object_pose(self._sim._h, self._index(), arr):
            raise SimulError(self._sim._error())
        return tuple(arr)

    position = property(lambda self: self.pose()[:3])
    quaternion = property(lambda self: self.pose()[3:])

    @property
    def joints(self):
        """{관절 이름: Joint}"""
        i = self._index()
        lib, h = self._sim._lib, self._sim._h
        out = {}
        for j in range(lib.simul_joint_count(h, i)):
            n = _s(lib.simul_joint_name(h, i, j))
            out[n] = Joint(self._sim, self, j, n)
        return out

    @property
    def moving(self):
        return bool(self._sim._lib.simul_moving(self._sim._h, _b(self.name)))

    # ---- 명령 ----
    def set_pose(self, position, quaternion=(0.0, 0.0, 0.0, 1.0)):
        self._sim._ok(self._sim._lib.simul_set_pose(self._sim._h, _b(self.name), position[0], position[1], position[2],
                                                    quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

    def set_position(self, position):
        self.set_pose(position, self.quaternion)

    def set_joint(self, joint, value):
        self._sim._ok(self._sim._lib.simul_set_joint(self._sim._h, _b(self.name), _b(joint), value))

    def move_to(self, position, speed):
        self._sim._ok(self._sim._lib.simul_move_to(self._sim._h, _b(self.name), position[0], position[1], position[2], speed))

    def move_joint(self, joint, target, speed=0.0):
        self._sim._ok(self._sim._lib.simul_move_joint(self._sim._h, _b(self.name), _b(joint), target, speed))

    def stop(self):
        self._sim._ok(self._sim._lib.simul_stop_motion(self._sim._h, _b(self.name)))

    def __repr__(self):
        return "<Object '%s'>" % self.name


class Contact(object):
    """지금 겹친 두 부분. a, b 는 오브젝트 이름, link_a, link_b 는 링크 이름, same 은 자기충돌."""

    def __init__(self, a, link_a, b, link_b, same):
        self.a, self.link_a, self.b, self.link_b, self.same = a, link_a, b, link_b, same

    def involves(self, name):
        return self.a == name or self.b == name

    def other(self, name):
        return self.b if self.a == name else self.a

    def __repr__(self):
        return "<Contact %s.%s <-> %s.%s>" % (self.a, self.link_a, self.b, self.link_b)


class Event(object):
    """simul_step 이 만든 이벤트. type: added removed arrived joint_reached command_failed"""

    def __init__(self, type_, name, joint, message, time):
        self.type, self.name, self.joint, self.message, self.time = type_, name, joint, message, time

    def __repr__(self):
        return "<Event %s %s%s>" % (self.type, self.name, (" " + self.joint) if self.joint else "")


class JointState(object):
    def __init__(self, name, value, lower, upper, velocity, has_limit):
        self.name, self.value, self.lower, self.upper, self.velocity, self.has_limit = name, value, lower, upper, velocity, has_limit

    def __repr__(self):
        return "<JointState %s %.4f>" % (self.name, self.value)


class ObjectState(object):
    """info.objects[이름]: 그 순간의 오브젝트 상태 (값의 복사본)."""

    def __init__(self, name, kind, pose, joints):
        self.name, self.kind = name, kind
        self.position, self.quaternion = tuple(pose[:3]), tuple(pose[3:])
        self.joints = joints

    def __repr__(self):
        return "<ObjectState %s %s>" % (self.name, self.kind)


class Info(object):
    """update 함수에게 넘기는 정보 묶음: 시간, 오브젝트 상태, 충돌. 한 step 안에서는 모두 같은 시각의 값이다."""

    def __init__(self, time, dt, objects, collisions):
        self.time, self.dt, self.objects, self.collisions = time, dt, objects, collisions

    def collided(self, name):
        return [c for c in self.collisions if c.involves(name)]

    def __repr__(self):
        return "<Info t=%.3f 오브젝트 %d개 충돌 %d건>" % (self.time, len(self.objects), len(self.collisions))


class Simulation(object):
    """시뮬레이션 하나. 모델을 올리고, 명령을 내리고, step 으로 시간을 진행한다."""

    def __init__(self):
        self._lib = _lib.load()
        self._h = self._lib.simul_create()
        if not self._h:
            raise SimulError("시뮬레이션을 만들 수 없습니다")
        self._sent = {}
        self._gui = None
        _live.add(self)

    def close(self):
        if getattr(self, "_h", None):
            if getattr(self, "_gui", None) is not None:
                self._gui.simul_window_close(self._h)
            self._lib.simul_destroy(self._h)
            self._h = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    def _error(self):
        return _s(self._lib.simul_last_error(self._h))

    def _ok(self, r):
        if not r:
            raise SimulError(self._error())

    # ---- 3D 창 (선택. simul_gui 와 Qt 가 있어야 한다) ----
    def show(self, title="Project Simul", width=960, height=640):
        """3D 창을 연다. 창은 보기 전용이고, update_window() 나 run() 이 시뮬레이션의 현재 상태를 그린다."""
        if self._gui is None:
            self._gui = _lib.load_gui()
            global _exit_registered
            if not _exit_registered:       # PyQt5 를 올린 뒤에 등록해야 (atexit 는 나중에 등록한 것이 먼저 돈다) 그 정리보다 먼저 창을 닫는다
                _atexit.register(_close_all)
                _exit_registered = True
        if not self._gui.simul_window_open(self._h, _b(title), width, height):
            raise SimulError(_s(self._gui.simul_gui_last_error()))

    @property
    def window_open(self):
        return self._gui is not None and bool(self._gui.simul_window_is_open(self._h))

    def update_window(self):
        """창을 현재 상태로 맞추고 이벤트를 처리한다. 열려 있으면 True, 닫혔으면 False. 루프 안에서 주기적으로 불러야 창이 반응한다."""
        if self._gui is None:
            return False
        return bool(self._gui.simul_window_update(self._h))

    def save_image(self, path):
        """지금 창의 화면을 이미지 파일로 저장한다 (창을 연 뒤에만)."""
        if self._gui is None or not self._gui.simul_window_save_image(self._h, _b(path)):
            raise SimulError(_s(self._gui.simul_gui_last_error()) if self._gui is not None else "창이 열려 있지 않습니다. 먼저 show() 를 부르세요")

    def fit_view(self):
        if self._gui is not None:
            self._gui.simul_window_fit_view(self._h)

    # ---- 장면 ----
    def add_search_dir(self, directory):
        self._lib.simul_add_search_dir(self._h, _b(directory))

    def add_model(self, path, position=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
        """모델 파일을 올리고 Object 를 돌려준다. rpy 는 rad."""
        i = self._lib.simul_add_model(self._h, _b(path), position[0], position[1], position[2], rpy[0], rpy[1], rpy[2])
        if i < 0:
            raise SimulError(self._error())
        return Object(self, _s(self._lib.simul_object_name(self._h, i)))

    def remove_model(self, name):
        self._ok(self._lib.simul_remove_model(self._h, _b(name)))

    def clear(self):
        self._lib.simul_clear(self._h)
        self._sent = {}

    @property
    def warnings(self):
        return [_s(self._lib.simul_warning(self._h, i)) for i in range(self._lib.simul_warning_count(self._h))]

    @property
    def objects(self):
        """{이름: Object}"""
        out = {}
        for i in range(self._lib.simul_object_count(self._h)):
            n = _s(self._lib.simul_object_name(self._h, i))
            out[n] = Object(self, n)
        return out

    def __getitem__(self, name):
        if self._lib.simul_find_object(self._h, _b(name)) < 0:
            raise KeyError(name)
        return Object(self, name)

    def collisions(self):
        """지금 겹친 쌍 [Contact]."""
        lib, h = self._lib, self._h
        return [Contact(_s(lib.simul_collision_object_a(h, i)), _s(lib.simul_collision_link_a(h, i)),
                        _s(lib.simul_collision_object_b(h, i)), _s(lib.simul_collision_link_b(h, i)),
                        bool(lib.simul_collision_same_object(h, i))) for i in range(lib.simul_collision_count(h))]

    # ---- 시간 ----
    @property
    def time(self):
        return self._lib.simul_time(self._h)

    @property
    def tick_count(self):
        return self._lib.simul_tick_count(self._h)

    def reset_time(self):
        self._lib.simul_reset_time(self._h)
        self._sent = {}

    def step(self, dt):
        """dt(s) 만큼 진행한다. 큐에 있던 명령이 먼저 적용되고, 움직임이 나아가고, 시계가 간다."""
        if not (dt > 0 and math.isfinite(dt)):
            raise ValueError("dt 는 0 보다 큰 유한한 수여야 합니다")
        self._ok(self._lib.simul_step(self._h, dt))

    def events(self):
        """마지막으로 꺼낸 뒤에 쌓인 이벤트 [Event]."""
        lib, h = self._lib, self._h
        return [Event(_EVENT_NAMES.get(lib.simul_event_type(h, i), "?"), _s(lib.simul_event_object(h, i)), _s(lib.simul_event_joint(h, i)),
                      _s(lib.simul_event_message(h, i)), lib.simul_event_time(h, i)) for i in range(lib.simul_take_events(h))]

    # ---- 정보 묶음과 update 방식 ----
    def info(self, dt=0.0):
        """지금 상태의 스냅샷(Info)을 만든다."""
        lib, h = self._lib, self._h
        objects = {}
        for i in range(lib.simul_object_count(h)):
            name = _s(lib.simul_object_name(h, i))
            kind = _s(lib.simul_object_kind(h, i))
            pose = (ctypes.c_double * 7)()
            lib.simul_object_pose(h, i, pose)
            joints = {}
            for j in range(lib.simul_joint_count(h, i)):
                arr = (ctypes.c_double * 6)()
                lib.simul_joint_info(h, i, j, arr)
                jn = _s(lib.simul_joint_name(h, i, j))
                joints[jn] = JointState(jn, arr[0], arr[1], arr[2], arr[3], bool(arr[4]))
            objects[name] = ObjectState(name, kind, tuple(pose), joints)
        return Info(self.time, dt, objects, self.collisions())

    def apply(self, movement):
        """움직임 정보 {오브젝트: {"joints": {관절: 목표 또는 (목표, 속도)}, "position": (x, y, z), "speed": m/s}} 를 명령으로 낸다.
        같은 목표는 다시 내지 않는다 (바뀔 때만 명령으로 낸다)."""
        if not movement:
            return
        if not isinstance(movement, dict):
            raise TypeError("움직임 정보는 {오브젝트: 움직임} dict 여야 합니다 (받은 것: %s)" % type(movement).__name__)
        for name, mv in movement.items():
            if not isinstance(mv, dict):
                raise TypeError("'%s' 의 움직임은 dict 여야 합니다: {\"joints\": {...}, \"position\": (x, y, z)}" % name)
            unknown = [k for k in mv if k not in ("joints", "position", "speed")]
            if unknown:
                raise SimulError("'%s' 의 움직임에 알 수 없는 항목이 있습니다: %s (joints, position, speed 만 됩니다)" % (name, ", ".join(unknown)))
            obj = self[name]
            for jn, v in (mv.get("joints") or {}).items():
                target, speed = (float(v[0]), float(v[1])) if isinstance(v, (tuple, list)) else (float(v), 0.0)
                if self._sent.get((name, jn)) != (target, speed):
                    obj.move_joint(jn, target, speed)
                    self._sent[(name, jn)] = (target, speed)
            pos = mv.get("position")
            if pos is not None:
                tgt = (float(pos[0]), float(pos[1]), float(pos[2]), float(mv.get("speed", 0.5)))
                if self._sent.get((name, "position")) != tgt:
                    obj.move_to(tgt[:3], tgt[3])
                    self._sent[(name, "position")] = tgt

    def run(self, update, dt=0.01, duration=None, until=None, realtime=False, on_step=None):
        """매 step 마다 update(info) 를 부르고, 돌려준 움직임 정보를 적용한 뒤 dt 만큼 진행한다.

        duration: 시뮬레이션 시간으로 이 만큼 돌고 끝낸다.  until(info) 가 True 를 돌려주면 끝낸다.
        realtime: 실제 시간에 맞춰 천천히 돌린다 (창을 보면서 쓸 때).  on_step(info): 매 step 뒤에 부르는 함수.
        창이 열려 있으면 매 step 뒤에 자동으로 갱신하고 (update_window), 사용자가 창을 닫으면 끝낸다.
        """
        t_end = None if duration is None else self.time + duration
        wall0, sim0 = _time.time(), self.time
        while True:
            info = self.info(dt)
            if until is not None and until(info):
                return info
            if t_end is not None and self.time >= t_end - 1e-12:
                return info
            self.apply(update(info))
            self.step(dt)
            if on_step is not None:
                on_step(self.info(dt))
            if self._gui is not None and not self.update_window():
                return self.info(dt)             # 창을 닫았다
            if realtime:
                lag = (self.time - sim0) - (_time.time() - wall0)
                if lag > 0:
                    _time.sleep(lag)
