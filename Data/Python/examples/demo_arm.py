# -*- coding: utf-8 -*-
"""등록 예: 팔 관절을 하나씩 위쪽 한계의 절반까지 움직였다가 0 으로 되돌린다.
실행:  simul_main Data/Object/samples/arm.urdf --python-script Data/Python/examples/demo_arm.py:arm --python-run
       (메뉴 Python > 시작 / 정지 / 한 틱. "arm" 은 이 파일의 클래스가 움직일 수 있는 오브젝트다)
"""
from Main import Behavior, register


class SwingArm(Behavior):
    objects = ["arm"]          # 이 클래스가 움직일 수 있는 오브젝트 (등록할 때 :arm 처럼 덮어쓸 수도 있다)

    def on_start(self):
        self.index = 0         # 지금 움직이는 관절 번호
        self.out = True        # True: 목표로 가는 중, False: 0 으로 돌아가는 중
        self.arm = self.objects["arm"]
        print("SwingArm 시작: 관절", sorted(self.arm.joints))
        self.go()

    def go(self):
        names = sorted(self.arm.joints)
        if not names:
            return
        j = names[self.index % len(names)]
        info = self.arm.info()["joints"][j]
        high = info["upper"] * 0.5 if info["has_limit"] else 0.5
        target = high if self.out else 0.0
        print("[t=%.2f] %s -> %.2f" % (self.time, j, target))
        self.arm.move_joint(j, target)         # 속도를 안 주면 관절의 최대 속도

    def on_event(self, ev):
        if ev.type == "joint_reached":
            if self.out:
                self.out = False               # 목표에 닿았으니 돌아간다
            else:
                self.out = True                # 돌아왔으니 다음 관절
                self.index += 1
            self.go()


register(SwingArm)
