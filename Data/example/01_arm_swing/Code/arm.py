# -*- coding: utf-8 -*-
"""arm: 사인파로 팔을 흔든다. 다른 오브젝트와 부딪히면 알려 준다."""
import math

import Simul


class Arm(Simul.Controller):
    def start(self):
        self.hit = False

    def update(self, info):
        hit = [c for c in info.collisions if c.involves(self.name) and not c.same]   # 자기 몸끼리는 뺀다
        if hit and not self.hit:
            print("[t=%.2f] 충돌: %s" % (info.time, hit[0]))
        self.hit = bool(hit)
        t = info.time
        return {"joints": {"base_yaw": 0.8 * math.sin(t),
                           "shoulder_pitch": 0.5 + 0.4 * math.sin(2 * t),
                           "elbow_pitch": 0.6 * math.sin(1.5 * t)}}
