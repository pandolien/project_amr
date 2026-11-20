# -*- coding: utf-8 -*-
"""프로젝트 예제: 팔 관절을 하나씩 위쪽 한계의 절반까지 움직였다가 0 으로 되돌린다 (끝없이 반복)."""
from Main import Behavior, register


class SwingArm(Behavior):
    objects = ["arm"]

    def on_start(self):
        self.index = 0
        self.out = True
        self.arm = self.objects["arm"]
        print("SwingArm 시작: 관절", sorted(self.arm.joints))
        self.go()

    def go(self):
        names = sorted(self.arm.joints)
        j = names[self.index % len(names)]
        info = self.arm.info()["joints"][j]
        high = info["upper"] * 0.5 if info["has_limit"] else 0.5
        self.arm.move_joint(j, high if self.out else 0.0)

    def on_event(self, ev):
        if ev.type == "joint_reached":
            if self.out:
                self.out = False
            else:
                self.out = True
                self.index += 1
            self.go()


register(SwingArm)
