# -*- coding: utf-8 -*-
"""Main.py: Simul 과 Python 을 잇는 중심 파일.

    Simul(C++ 엔진) -> Python 인터프리터 -> Main.py -> 등록된 외부 파일의 클래스

  * 프로그램이 시작할 때 이 파일이 먼저 실행된다 (--python 으로 다른 Main.py 를 줄 수 있다).
  * 모든 오브젝트 데이터를 여기서 쥔다: 매 틱 엔진에서 오브젝트 정보를 한 번 읽어 hub.data 에 둔다 (한 틱 안에서는 모두 같은 값을 본다).
  * 외부 python 파일은 Behavior 를 상속한 클래스를 만들어 register 로 등록한다. 시뮬레이션이 시작되면 등록된 클래스가 만들어지고
    on_start / on_tick / on_event / on_stop 이 불린다.
  * 클래스마다 "움직일 수 있는 오브젝트"를 정한다 (권한). 읽기는 모든 오브젝트가 된다. 권한 밖의 오브젝트에 명령하면 PermissionError.
    (권한 검사는 약속이다. 악의적인 스크립트를 막는 보안 장치는 아니다.)
  * lockstep: 명령은 다음 틱 시작에 적용된다. 한 틱 안에서 명령을 내려도 on_tick 안에서 값은 바로 바뀌지 않는다.
  * 한 클래스에서 예외가 나면 그 클래스만 멈추고 (traceback 출력) 나머지는 계속 돈다.

외부 파일 예 (Data/Python/examples/*.py 참고):

    from Main import Behavior, register

    class Mover(Behavior):
        objects = ["arm"]                 # 움직일 수 있는 오브젝트 (register(objects=...) 로 덮어쓸 수 있다. "*" 는 전부)
        period = 0.0                      # on_tick 을 부르는 최소 간격 (s). 0 이면 매 틱
        def on_start(self):
            self.arm = self.objects["arm"]
            self.arm.move_joint("joint1", 1.0)          # 관절 최대 속도로
        def on_event(self, ev):
            if ev.type == "joint_reached":
                print("도착", ev.name, ev.joint)

    register(Mover)                        # 또는 @register / @register(objects=["arm"]) 데코레이터

단위: m, rad, s. 자세는 쿼터니언 (x, y, z, w).
"""
import os
import sys
import time as _time
import traceback

import simul

# 외부 파일이 `from Main import ...` 를 해도 지금 실행 중인 이 모듈을 쓰게 한다 (둘이 따로 만들어지면 허브가 둘이 된다)
sys.modules.setdefault("Main", sys.modules[__name__])

API_VERSION = 1
if getattr(simul, "API_VERSION", 0) != API_VERSION:
    raise RuntimeError("simul 모듈 API 버전이 맞지 않습니다: %s (Main.py 는 %d 용)" % (getattr(simul, "API_VERSION", "?"), API_VERSION))

_COMMANDS = ("set_pose", "set_position", "set_joint", "move_to", "move_joint", "stop", "command")


class View(object):
    """오브젝트의 읽기 전용 보기 (그 틱의 스냅샷). 명령 권한이 없는 오브젝트는 이것으로 보인다."""

    def __init__(self, hub, name):
        self._hub = hub
        self._name = name

    def _d(self):
        d = self._hub.data.get(self._name)
        if d is None:
            raise simul.CommandError("오브젝트 '%s' 가 없습니다" % self._name)
        return d

    name = property(lambda self: self._name)
    alive = property(lambda self: self._name in self._hub.data)
    kind = property(lambda self: self._d()["kind"])
    position = property(lambda self: self._d()["position"])
    quaternion = property(lambda self: self._d()["quaternion"])
    joints = property(lambda self: dict((k, v["value"]) for k, v in self._d()["joints"].items()))
    moving = property(lambda self: self._hub._raw(self._name).moving)

    def info(self):
        return self._d()

    def joint(self, name):
        return self._d()["joints"][name]["value"]

    def __getattr__(self, attr):
        if attr in _COMMANDS:
            raise PermissionError("오브젝트 '%s' 를 움직일 권한이 없습니다 (register 의 objects 에 넣어야 합니다)" % self._name)
        raise AttributeError(attr)

    def __repr__(self):
        return "<View '%s'>" % self._name


class Controller(View):
    """명령 권한이 있는 오브젝트. View 의 읽기에 더해 명령을 낸다. 명령은 다음 틱 시작에 적용된다."""

    def _o(self):
        o = self._hub._raw(self._name)
        if o is None:
            raise simul.CommandError("오브젝트 '%s' 가 없습니다" % self._name)
        return o

    def set_pose(self, x, y, z, qx, qy, qz, qw):
        self._o().set_pose(x, y, z, qx, qy, qz, qw)

    def set_position(self, x, y, z):
        self._o().set_position(x, y, z)

    def set_joint(self, name, value):
        self._o().set_joint(name, value)

    def move_to(self, x, y, z, speed):
        self._o().move_to(x, y, z, speed)

    def move_joint(self, name, target, speed=0.0):
        self._o().move_joint(name, target, speed)

    def stop(self):
        self._o().stop()

    def command(self, name, *args):
        if name not in _COMMANDS[:-1]:
            raise simul.CommandError("알 수 없는 명령입니다: %s (%s)" % (name, ", ".join(_COMMANDS[:-1])))
        return getattr(self, name)(*args)

    def __getattr__(self, attr):
        raise AttributeError(attr)

    def __repr__(self):
        return "<Controller '%s'>" % self._name


class Event(object):
    """엔진 이벤트. type: added removed arrived joint_reached command_failed"""

    def __init__(self, hub, d):
        self.type = d["type"]
        self.name = d["name"]
        self.joint = d["joint"]
        self.message = d["message"]
        self.time = d["time"]
        self.object = View(hub, d["name"]) if d["name"] else None

    def __repr__(self):
        return "<Event %s %s%s>" % (self.type, self.name, (" " + self.joint) if self.joint else "")


class Behavior(object):
    """등록해서 쓰는 클래스의 부모. 필요한 것만 덮어쓴다.

    클래스 속성:  objects (움직일 수 있는 오브젝트 이름 목록, "*" 는 전부), period (on_tick 최소 간격 s)
    인스턴스 속성 (Main 이 시작 전에 채운다):
        self.objects   {이름: Controller}   움직일 수 있는 오브젝트 (지금 있는 것만. 늦게 생기면 나중에 들어온다)
        self.world     {이름: View}         모든 오브젝트 (읽기 전용)
        self.time, self.dt                  시뮬레이션 시계, 마지막 on_tick 간격
    """

    objects = []
    period = 0.0

    def on_start(self):
        pass

    def on_tick(self, t, dt):
        pass

    def on_event(self, ev):
        pass

    def on_stop(self):
        pass


class _Entry(object):
    def __init__(self, cls, grants, period, source):
        self.cls = cls
        self.grants = list(grants)
        self.period = period
        self.source = source
        self.inst = None
        self.state = "ready"     # ready running error stopped
        self.acc = 0.0

    def allowed(self, name):
        return "*" in self.grants or name in self.grants

    def label(self):
        return "%s%s" % (self.cls.__name__, (" (" + os.path.basename(self.source) + ")") if self.source else "")


class Hub(object):
    """모든 오브젝트 데이터와 등록된 클래스를 쥔다."""

    def __init__(self):
        self.data = {}          # 이름 -> info dict (그 틱의 스냅샷)
        self.entries = []
        self.started = False
        self.time = 0.0
        self._loading = None
        self._raws = {}
        self.slow_warn = 0.1     # 한 클래스의 한 콜백이 이보다 오래 걸리면 경고 (s). 틱 안에서 오래 걸리면 화면이 멈춘다
        self._warned = {}

    # ---- 오브젝트 데이터 ----
    def _raw(self, name):
        o = self._raws.get(name)
        if o is None or not o.alive:
            o = simul.find(name)
            if o is not None:
                self._raws[name] = o
        return o

    def refresh(self):
        data = {}
        raws = {}
        for o in simul.objects():
            try:
                data[o.name] = o.info()
                raws[o.name] = o
            except simul.CommandError:
                pass
        self.data = data
        self._raws = raws

    def world(self):
        return dict((n, View(self, n)) for n in self.data)

    def objects_of(self, entry):
        out = {}
        for n in self.data:
            if entry.allowed(n):
                out[n] = Controller(self, n)
        return out

    def describe(self):
        lines = ["[Main] 오브젝트 %d개" % len(self.data)]
        for n in sorted(self.data):
            d = self.data[n]
            p = d["position"]
            js = ", ".join(sorted(d["joints"])) if d["joints"] else "-"
            lines.append("  %-16s %-6s 위치(%.2f, %.2f, %.2f) 관절: %s" % (n, d["kind"], p[0], p[1], p[2], js))
        lines.append("[Main] 등록된 클래스 %d개" % len(self.entries))
        for e in self.entries:
            lines.append("  %-24s 권한: %s" % (e.label(), ", ".join(e.grants) if e.grants else "(없음, 읽기만)"))
        return "\n".join(lines)

    # ---- 등록 ----
    def register(self, cls, objects=None, period=None):
        if not (isinstance(cls, type) and issubclass(cls, Behavior)):
            raise TypeError("register 에는 Behavior 를 상속한 클래스를 줘야 합니다")
        grants = objects
        if grants is None and self._loading is not None:
            grants = self._loading["objects"]
        if grants is None:
            grants = cls.objects
        if isinstance(grants, str):
            grants = [grants]
        src = self._loading["path"] if self._loading is not None else (sys.modules.get(cls.__module__).__file__ if cls.__module__ in sys.modules and hasattr(sys.modules[cls.__module__], "__file__") else "")
        entry = _Entry(cls, grants, cls.period if period is None else period, src)
        self.entries.append(entry)
        if self._loading is not None:
            self._loading["count"] += 1
        if self.started:
            self._launch(entry)
        return entry

    def unregister_file(self, path):
        """그 파일에서 등록된 클래스를 모두 뺀다 (돌고 있으면 on_stop 을 부르고 멈춘다). 빼낸 개수를 돌려준다."""
        path = os.path.abspath(path)
        gone = [e for e in self.entries if e.source and os.path.abspath(e.source) == path]
        for e in gone:
            if e.state == "running":
                try:
                    e.inst.on_stop()
                except Exception:
                    self._fail(e, "on_stop")
            self.entries.remove(e)
        return len(gone)

    def reload_file(self, path, objects=None):
        """저장한 파일을 다시 읽는다: 이전에 등록된 것을 빼고 새로 등록한다 (돌고 있으면 새 클래스가 바로 시작된다)."""
        n = self.unregister_file(path)
        if n:
            print("[Main] %s: 이전 클래스 %d개를 내리고 다시 읽습니다" % (os.path.basename(path), n))
        return self.register_file(path, objects)

    def register_file(self, path, objects=None):
        """외부 python 파일을 읽어서 그 안의 register(...) 를 실행한다. objects 를 주면 그 파일의 클래스에 모두 그 권한을 준다."""
        import importlib.util
        path = os.path.abspath(path)
        if not os.path.isfile(path):
            print("[Main] 파일이 없습니다: %s" % path, file=sys.stderr)
            return False
        if isinstance(objects, str):
            objects = [o for o in objects.split(",") if o]
        d = os.path.dirname(path)
        if d not in sys.path:
            sys.path.insert(0, d)
        modname = "simul_user_%d" % len(self.entries) + "_" + os.path.splitext(os.path.basename(path))[0]
        self._loading = {"path": path, "objects": objects, "count": 0}
        try:
            spec = importlib.util.spec_from_file_location(modname, path)
            mod = importlib.util.module_from_spec(spec)
            sys.modules[modname] = mod
            spec.loader.exec_module(mod)
        except SystemExit:
            raise
        except BaseException:
            traceback.print_exc()
            print("[Main] 파일을 읽지 못했습니다: %s" % path, file=sys.stderr)
            return False
        finally:
            count = self._loading["count"]
            self._loading = None
        if count == 0:
            print("[Main] 경고: %s 에서 register 된 클래스가 없습니다" % path, file=sys.stderr)
        return count > 0

    # ---- 실행 ----
    def _fail(self, entry, where):
        entry.state = "error"
        traceback.print_exc()
        print("[Main] %s 의 %s 에서 예외가 나서 그 클래스를 멈춥니다" % (entry.label(), where), file=sys.stderr)

    def _check_slow(self, entry, where, seconds):
        if seconds > self.slow_warn:
            now = _time.perf_counter()
            if now - self._warned.get(id(entry), -1e9) > 5.0:     # 같은 클래스는 5 초에 한 번만
                self._warned[id(entry)] = now
                print("[Main] 경고: %s 의 %s 가 %.0f ms 걸렸습니다 (화면이 멈춥니다. 오래 걸리는 일은 나눠서 하세요)" % (entry.label(), where, seconds * 1000), file=sys.stderr)

    def _launch(self, entry):
        try:
            inst = entry.cls()
            entry.inst = inst
            self._bind(entry)
            entry.state = "running"
            inst.on_start()
        except Exception:
            self._fail(entry, "on_start")

    def _bind(self, entry):
        inst = entry.inst
        inst.objects = self.objects_of(entry)
        inst.world = self.world()
        inst.time = self.time
        inst.dt = 0.0

    def _start(self):
        self.started = True
        self.refresh()
        self.time = simul.time()
        print(self.describe())
        for e in list(self.entries):
            if e.state in ("ready", "stopped", "error"):
                e.state = "ready"
                e.acc = 0.0
                self._launch(e)

    def _tick(self, t, dt):
        self.time = t
        self.refresh()
        for e in list(self.entries):
            if e.state != "running":
                continue
            e.acc += dt
            if e.acc + 1e-12 < e.period:
                continue
            step, e.acc = e.acc, 0.0
            t0 = _time.perf_counter()
            try:
                self._bind(e)
                e.inst.time = t
                e.inst.dt = step
                e.inst.on_tick(t, step)
            except Exception:
                self._fail(e, "on_tick")
            self._check_slow(e, "on_tick", _time.perf_counter() - t0)

    def _event(self, d):
        ev = Event(self, d)
        for e in list(self.entries):
            if e.state != "running":
                continue
            # 그 오브젝트를 움직일 권한이 있는 클래스에게, 오브젝트가 생기거나 사라지는 이벤트는 모두에게
            if ev.type in ("added", "removed") or e.allowed(ev.name):
                try:
                    self._bind(e)
                    e.inst.on_event(ev)
                except Exception:
                    self._fail(e, "on_event")

    def _stop(self):
        for e in list(self.entries):
            if e.state == "running":
                try:
                    e.inst.on_stop()
                except Exception:
                    self._fail(e, "on_stop")
                else:
                    e.state = "stopped"
        self.started = False


hub = Hub()


def register(cls=None, objects=None, period=None):
    """클래스를 등록한다. register(Cls), register(Cls, objects=[...]), @register, @register(objects=[...]) 모두 된다."""
    if cls is None:
        def deco(c):
            hub.register(c, objects=objects, period=period)
            return c
        return deco
    hub.register(cls, objects=objects, period=period)
    return cls


def register_file(path, objects=None):
    return hub.register_file(path, objects)


def reload_file(path, objects=None):
    return hub.reload_file(path, objects)


# 엔진에 연결: 이 파일이 실행되면 hub 가 시뮬레이션 시작/틱/이벤트/정지를 받는다
simul.clear_callbacks()
simul.on_start(hub._start)
simul.on_tick(hub._tick)
simul.on_event(lambda d: hub._event(d))
simul.on_stop(hub._stop)
