# -*- coding: utf-8 -*-
"""simul_core 공유 라이브러리(C API, api/include/simul_api.h)를 ctypes 로 불러온다."""
import ctypes
import os
import sys

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


_lib = None


def load():
    global _lib
    if _lib is not None:
        return _lib
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
    _lib = lib
    return lib


_gui = None
_qapp = None


def _prepare_qt():
    """python 환경(venv)에 PyQt5 가 있으면 그 Qt 를 먼저 올리고 QApplication 도 거기서 만든다.
    simul_gui 는 이미 올라온 같은 이름의 Qt 라이브러리와 이미 있는 QApplication 을 그대로 쓰므로 Qt 가 한 벌만 돈다.
    (PyQt5 와 simul_gui 의 Qt 는 같은 5.12 계열이어야 한다. 쓰지 않으려면 환경 변수 SIMUL_NO_PYQT=1)"""
    global _qapp
    if _qapp is not None or os.environ.get("SIMUL_NO_PYQT"):
        return
    try:
        from PyQt5 import QtCore, QtWidgets
    except ImportError:
        return
    _qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1] or ["simul"])
    qt = QtCore.QT_VERSION_STR
    if not qt.startswith("5.12."):
        sys.stderr.write("[simul] 경고: PyQt5 의 Qt 가 %s 입니다. simul_gui 는 Qt 5.12 로 만들었으니 5.12 PyQt5 (pip install PyQt5==5.12.3) 를 쓰세요.\n" % qt)


def load_gui():
    """3D 창 라이브러리(simul_gui, Qt 필요)를 불러온다. 없으면 OSError."""
    global _gui
    if _gui is not None:
        return _gui
    core = load()
    _prepare_qt()
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
