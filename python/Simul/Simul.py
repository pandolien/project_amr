# -*- coding: utf-8 -*-
"""Project Simul 의 python 인터페이스 (simul_core 공유 라이브러리 위의 얇은 래퍼).

    import Simul
    sim = Simul.Simulation()
    arm = sim.add_model("arm.urdf", position=(0, 0, 0))
    arm.move_joint("joint_1", 1.57, speed=0.5)         # 명령은 다음 step 시작에 적용된다
    for _ in range(300):
        sim.step(0.01)                                  # 0.01 s 진행
    print(arm.joints["joint_1"].value, sim.time)

오브젝트마다 Code/<이름>.py 에 Simul.Controller 를 이어받은 class 를 만들고 Simul.Behaviors(폴더, sim) 로 묶어 sim.run(...) 에 넘기는 방식을 권한다.
함수 하나로 돌리려면 (정보를 받고 움직임을 돌려주는 함수):

    def update(info):                                   # info.time, info.dt, info.objects, info.collisions
        return {"arm": {"joints": {"joint_1": 1.57}}}
    sim.run(update, dt=0.01, duration=5.0)

단위: 길이 m, 각도 rad, 시간 s, 자세 쿼터니언 (x, y, z, w).

이 파일 하나가 Project Simul 의 python 쪽이다: simul_core / simul_gui 공유 라이브러리(DLL, so)를 ctypes 로 불러 오브젝트를 움직인다.
같은 폴더의 DLL 을 먼저 찾고, 없으면 환경 변수 SIMUL_LIB(파일 또는 폴더)를 본다.
"""
import atexit as _atexit
import ctypes
import json as _json
import math
import os
import sys
import time as _time
import traceback
import types as _types
import weakref as _weakref

# ---------------------------------------------------------------- 라이브러리 로더
API_VERSION = 1


def _names(base="simul_core"):
    if sys.platform.startswith("win"):
        return [base + ".dll", "lib" + base + ".dll"]
    if sys.platform == "darwin":
        return ["lib" + base + ".dylib", "lib" + base + ".so"]
    return ["lib" + base + ".so"]


def _candidates(base="simul_core"):
    env = os.environ.get("SIMUL_LIB")
    if env:
        yield env                      # 파일 경로 또는 폴더
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", ".."))
    dirs = [here, os.path.join(here, ".."), root,
            os.path.join(root, "build", "bin"), os.path.join(root, "build", "lib"),
            os.path.join(root, "build", "bin", "Release"), os.path.join(root, "build", "Release"),
            os.path.join(root, "out", "build"), os.getcwd()]
    for d in dirs:
        for n in _names(base):
            yield os.path.join(d, n)


def find_library(base="simul_core"):
    """공유 라이브러리(simul_core 또는 simul_gui)의 경로를 찾는다. 환경 변수 SIMUL_LIB(파일 또는 폴더)가 있으면 그것을 먼저 본다."""
    tried = []
    for c in _candidates(base):
        if os.path.isdir(c):
            for n in _names(base):
                p = os.path.join(c, n)
                tried.append(p)
                if os.path.isfile(p):
                    return p
        else:
            tried.append(c)
            if os.path.isfile(c):
                return c
    raise OSError("%s 라이브러리를 찾지 못했습니다. 빌드한 폴더를 환경 변수 SIMUL_LIB 에 지정하세요.\n찾아본 곳:\n  " % base + "\n  ".join(tried))


_corelib = None


def load():
    global _corelib
    if _corelib is not None:
        return _corelib
    path = find_library()
    if hasattr(os, "add_dll_directory"):       # Windows 에서 이 DLL 이 쓰는 다른 DLL 을 같은 폴더에서 찾게 한다
        try:
            os.add_dll_directory(os.path.dirname(os.path.abspath(path)))
        except OSError:
            pass
    lib = ctypes.CDLL(path)
    _declare(lib)
    v = lib.simul_api_version()
    if v != API_VERSION:
        raise RuntimeError("simul_core 인터페이스 버전이 맞지 않습니다 (라이브러리 %d, python 래퍼 %d): %s" % (v, API_VERSION, path))
    lib._path = path
    _corelib = lib
    return lib


_gui = None


def load_gui():
    """3D 창 라이브러리(simul_gui, Qt 필요)를 불러온다. 없으면 OSError."""
    global _gui
    if _gui is not None:
        return _gui
    core = load()
    path = find_library("simul_gui")
    if hasattr(os, "add_dll_directory"):
        try:
            os.add_dll_directory(os.path.dirname(os.path.abspath(path)))
        except OSError:
            pass
    gui = ctypes.CDLL(path)
    P, S, I = ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int

    def f(name, res, *args):
        fn = getattr(gui, name)
        fn.restype = res
        fn.argtypes = list(args)

    f("simul_gui_api_version", I)
    f("simul_window_open", I, P, S, I, I)
    f("simul_window_create_embedded", I, P)
    f("simul_window_widget", P, P)
    f("simul_window_update", I, P)
    f("simul_window_is_open", I, P)
    f("simul_window_close", None, P)
    f("simul_window_save_image", I, P, S)
    f("simul_window_fit_view", I, P)
    f("simul_gui_last_error", S)
    if gui.simul_gui_api_version() != API_VERSION:
        raise RuntimeError("simul_gui 인터페이스 버전이 맞지 않습니다: " + path)
    _gui = gui
    return gui


def _declare(lib):
    P = ctypes.c_void_p
    S = ctypes.c_char_p
    I = ctypes.c_int
    D = ctypes.c_double
    DA = ctypes.POINTER(ctypes.c_double)

    def f(name, res, *args):
        fn = getattr(lib, name)
        fn.restype = res
        fn.argtypes = list(args)

    f("simul_api_version", I)
    f("simul_version", S)
    f("simul_create", P)
    f("simul_destroy", None, P)
    f("simul_last_error", S, P)
    f("simul_add_model", I, P, S, D, D, D, D, D, D)
    f("simul_remove_model", I, P, S)
    f("simul_clear", None, P)
    f("simul_add_search_dir", None, P, S)
    f("simul_search_dir_count", I, P)
    f("simul_search_dir", S, P, I)
    f("simul_warning_count", I, P)
    f("simul_warning", S, P, I)
    f("simul_object_count", I, P)
    f("simul_find_object", I, P, S)
    f("simul_object_name", S, P, I)
    f("simul_object_kind", S, P, I)
    f("simul_object_path", S, P, I)
    f("simul_object_pose", I, P, I, DA)
    f("simul_joint_count", I, P, I)
    f("simul_joint_name", S, P, I, I)
    f("simul_joint_info", I, P, I, I, DA)
    f("simul_collision_count", I, P)
    f("simul_collision_object_a", S, P, I)
    f("simul_collision_link_a", S, P, I)
    f("simul_collision_object_b", S, P, I)
    f("simul_collision_link_b", S, P, I)
    f("simul_collision_same_object", I, P, I)
    f("simul_set_pose", I, P, S, D, D, D, D, D, D, D)
    f("simul_set_joint", I, P, S, S, D)
    f("simul_move_to", I, P, S, D, D, D, D)
    f("simul_move_joint", I, P, S, S, D, D)
    f("simul_stop_motion", I, P, S)
    f("simul_moving", I, P, S)
    f("simul_step", I, P, D)
    f("simul_time", D, P)
    f("simul_tick_count", ctypes.c_ulonglong, P)
    f("simul_reset_time", None, P)
    f("simul_take_events", I, P)
    f("simul_event_type", I, P, I)
    f("simul_event_object", S, P, I)
    f("simul_event_joint", S, P, I)
    f("simul_event_message", S, P, I)
    f("simul_event_time", D, P, I)

_lib = _types.SimpleNamespace(load=load, load_gui=load_gui, find_library=find_library, _names=_names, API_VERSION=API_VERSION)

# ---------------------------------------------------------------- 오브젝트별 python 파일
try:
    from importlib.util import module_from_spec, spec_from_file_location
except ImportError:                                        # python 2 (쓰지 않는 것이 좋다)
    spec_from_file_location = None


def _load(path, name):
    spec = spec_from_file_location("simul_obj_" + name, path)
    mod = module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Controller(object):
    """오브젝트 하나를 제어하는 class 의 바탕. Code/<오브젝트 이름>.py 안에 이 class 를 이어받은 class 를 만든다.

        class Arm(Simul.Controller):
            def start(self):                    # 시작할 때 한 번
                self.base = self.obj.joints["joint_1"]
            def update(self, info):             # 매 스텝. 움직임 정보를 돌려준다 (없으면 None)
                return {"joints": {"joint_1": 1.0}}
            def stop(self):                     # 끝날 때 한 번
                pass

    self.sim: Simulation, self.obj: 이 오브젝트(Object), self.name: 오브젝트 이름.
    update(info) 가 돌려주는 값은 {"joints": {관절: 목표 rad 또는 (목표, 속도)}, "position": (x, y, z), "speed": m/s}  (이 오브젝트의 것만)
    """

    def __init__(self, sim, obj):
        self.sim = sim
        self.obj = obj
        self.name = obj.name

    def start(self):
        pass

    def update(self, info):
        return None

    def stop(self):
        pass


class Behaviors(object):
    """폴더의 오브젝트별 파일(Code/<오브젝트 이름>.py)을 읽어 둔다. 파일 안의 Controller class 를 오브젝트마다 하나씩 만들어 쓴다
    (class 가 없으면 파일 안의 update(info) 함수를 쓴다). 객체 자체를 sim.run(...) 의 update 로 쓸 수 있다."""

    def __init__(self, folder, sim=None):
        self.folder = folder
        self.modules = {}
        self.controllers = {}        # 오브젝트 이름 -> Controller 객체
        self.failed = {}
        if not os.path.isdir(folder):
            raise OSError("오브젝트 python 폴더가 없습니다: " + folder)
        for fn in sorted(os.listdir(folder)):
            stem, ext = os.path.splitext(fn)
            if ext != ".py" or stem.startswith("_") or stem == "object":     # object.py 는 견본
                continue
            try:
                self.modules[stem] = _load(os.path.join(folder, fn), stem)
            except Exception:
                self.failed[stem] = traceback.format_exc()
                print("[%s] 읽을 수 없습니다:\n%s" % (fn, self.failed[stem]))
        if sim is not None:
            for stem in self.modules:
                if stem not in sim.objects:
                    print("[경고] %s.py 와 같은 이름의 오브젝트가 없습니다 (오브젝트: %s)" % (stem, ", ".join(sorted(sim.objects))))
            self.start(sim)

    @staticmethod
    def _controller_class(mod):
        found = [v for v in vars(mod).values()
                 if isinstance(v, type) and issubclass(v, Controller) and v is not Controller and v.__module__ == mod.__name__]
        return found[0] if found else None

    def start(self, sim):
        for stem, mod in list(self.modules.items()):
            cls = self._controller_class(mod)
            if cls is not None:
                if stem not in sim.objects:
                    continue
                obj = self._call(stem, lambda: cls(sim, sim[stem]))
                if obj is None:
                    continue
                self.controllers[stem] = obj
                self._call(stem, obj.start)
            else:
                f = getattr(mod, "on_start", None)
                if f is not None:
                    self._call(stem, f, sim)

    def stop(self):
        for stem, c in list(self.controllers.items()):
            self._call(stem, c.stop)
        self.controllers.clear()

    def _call(self, stem, f, *args):
        try:
            return f(*args)
        except Exception:
            print("[%s.py] 오류로 이 오브젝트의 코드를 멈춥니다:\n%s" % (stem, traceback.format_exc()))
            self.modules.pop(stem, None)
            self.controllers.pop(stem, None)
            self.failed[stem] = "error"
            return None

    def __call__(self, info):
        out = {}
        for stem, mod in list(self.modules.items()):
            if stem not in info.objects:
                continue
            f = self.controllers[stem].update if stem in self.controllers else getattr(mod, "update", None)
            if f is None:
                continue
            r = self._call(stem, f, info)
            if r:
                out[stem] = r
        return out

# ---------------------------------------------------------------- 시뮬레이션
__all__ = ["Simulation", "Controller", "Behaviors", "SimulError", "Object", "Joint", "Contact", "Event", "Info", "ObjectState", "JointState"]

_EVENT_NAMES = {0: "added", 1: "removed", 2: "arrived", 3: "joint_reached", 4: "command_failed"}
_KIND_NAMES = {1: "revolute", 2: "prismatic", 3: "continuous"}



_live = _weakref.WeakSet()
_exit_registered = False


def _close_all():
    """프로세스가 끝나기 전에 창과 시뮬레이션을 먼저 닫는다."""
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
        self._feed = os.environ.get("SIMUL_FEED") == "1"     # 에디터가 3D View 로 상태를 받아 갈 때 (환경 변수 SIMUL_FEED=1)
        self._feed_t = 0.0
        self._feed_dirty = False
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
            if not _exit_registered:
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
        if self._feed:
            self._emit_feed(False)

    def _emit_feed(self, force):
        """에디터에 오브젝트 상태를 한 줄(@@SIMUL {json})로 보낸다. 초당 30 번까지만 (force 면 바로)."""
        now = _time.time()
        if not force and now - self._feed_t < 1.0 / 30.0:
            self._feed_dirty = True
            return
        self._feed_t, self._feed_dirty = now, False
        lib, h = self._lib, self._h
        objs = {}
        for i in range(lib.simul_object_count(h)):
            pose = (ctypes.c_double * 7)()
            lib.simul_object_pose(h, i, pose)
            joints = {}
            for j in range(lib.simul_joint_count(h, i)):
                arr = (ctypes.c_double * 6)()
                lib.simul_joint_info(h, i, j, arr)
                joints[_s(lib.simul_joint_name(h, i, j))] = arr[0]
            objs[_s(lib.simul_object_name(h, i))] = {"p": list(pose), "j": joints}
        sys.stdout.write("@@SIMUL " + _json.dumps({"t": self.time, "o": objs}) + "\n")
        sys.stdout.flush()

    def flush_feed(self):
        """밀린 마지막 상태를 에디터에 보낸다 (python CMD 의 프롬프트 앞과 run 이 끝날 때 불린다)."""
        if self._feed and self._feed_dirty:
            self._emit_feed(True)

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
        if self._feed:
            self._emit_feed(True)                # 시작 상태
        wall0, sim0 = _time.time(), self.time
        while True:
            info = self.info(dt)
            if until is not None and until(info):
                return info
            if t_end is not None and self.time >= t_end - 1e-12:
                self.flush_feed()
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
