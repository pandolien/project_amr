# -*- coding: utf-8 -*-
"""오브젝트마다 python 파일 하나: <폴더>/<오브젝트 이름>.py 안의 update(info) 가 그 오브젝트의 움직임을 돌려준다.

    # objects/arm.py
    def update(info):
        me = info.objects["arm"]
        return {"joints": {"base_yaw": 1.0}}        # 자기 오브젝트의 움직임만 돌려준다 (position, speed 도 가능)

선택: on_start(sim) 은 시작할 때 한 번, 파일은 바뀌면 다시 읽지 않는다.
"""
import os
import traceback

try:
    from importlib.util import module_from_spec, spec_from_file_location
except ImportError:                                        # python 2 (쓰지 않는 것이 좋다)
    spec_from_file_location = None


def _load(path, name):
    spec = spec_from_file_location("simul_obj_" + name, path)
    mod = module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Behaviors(object):
    """폴더의 오브젝트별 파일을 읽어 둔다. 객체 자체를 sim.run(...) 의 update 로 쓸 수 있다."""

    def __init__(self, folder, sim=None):
        self.folder = folder
        self.modules = {}
        self.failed = {}
        if not os.path.isdir(folder):
            raise OSError("오브젝트 python 폴더가 없습니다: " + folder)
        for fn in sorted(os.listdir(folder)):
            stem, ext = os.path.splitext(fn)
            if ext != ".py" or stem.startswith("_"):
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

    def start(self, sim):
        for stem, mod in list(self.modules.items()):
            f = getattr(mod, "on_start", None)
            if f is not None:
                self._call(stem, f, sim)

    def _call(self, stem, f, *args):
        try:
            return f(*args)
        except Exception:
            print("[%s.py] 오류로 이 오브젝트의 코드를 멈춥니다:\n%s" % (stem, traceback.format_exc()))
            self.modules.pop(stem, None)
            self.failed[stem] = "error"
            return None

    def __call__(self, info):
        out = {}
        for stem, mod in list(self.modules.items()):
            f = getattr(mod, "update", None)
            if f is None or stem not in info.objects:
                continue
            r = self._call(stem, f, info)
            if r:
                out[stem] = r
        return out
