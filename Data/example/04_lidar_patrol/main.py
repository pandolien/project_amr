# -*- coding: utf-8 -*-
"""프로젝트 시작 파일.  python main.py [--no-window]
DLL(simul_core)은 Simul/ 폴더에 있고, 오브젝트는 OBJECT/*.urdf, 오브젝트별 제어 class 는 Code/<오브젝트 이름>.py 에 있다."""
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "Simul"))     # Simul/Simul.py : DLL 을 부르는 python 파일
import Simul  # noqa: E402


class Main(object):
    """프로젝트 전체: 시뮬레이션을 만들고, OBJECT/ 의 오브젝트를 올리고, Code/ 의 오브젝트 제어 class 들을 연결해 돌린다."""

    TITLE = "04_lidar_patrol"
    DT = 0.02                                       # 한 스텝 (s)
    DURATION = None                                 # 돌 시간 (s). None 이면 창을 닫을 때까지 (창이 없으면 5 초)

    def __init__(self, window=True, editor=False):
        self.window = window                        # 3D 창 (simul_gui, Qt 필요)
        self.editor = editor                        # 에디터가 3D View 로 보여 준다 (창 없이, 닫을 때까지 실시간으로 돈다)
        self.sim = Simul.Simulation()               # simul_core DLL 위의 시뮬레이션
        self.load_objects()
        self.controllers = Simul.Behaviors(os.path.join(HERE, "Code"), self.sim)   # Code/<오브젝트>.py 의 Controller 들

    def load_objects(self):
        for f in sorted(glob.glob(os.path.join(HERE, "OBJECT", "*.urdf"))):       # OBJECT/ 의 URDF 를 모두 올린다 (추가하려면 파일만 넣으면 된다)
            self.sim.add_model(f)

    def run(self):
        if self.window:
            self.sim.show(self.TITLE)
        try:
            live = self.window or self.editor
            self.sim.run(self.controllers, dt=self.DT, duration=self.DURATION if live else (self.DURATION or 5.0), realtime=live)
        finally:
            self.controllers.stop()


if __name__ == "__main__":
    editor = "--editor" in sys.argv
    Main(window="--no-window" not in sys.argv and not editor, editor=editor).run()
