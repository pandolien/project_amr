# -*- coding: utf-8 -*-
"""arm: 마커 쪽으로 몸통을 돌렸다가 제자리로 돌아오기를 되풀이한다."""
import math

import Simul


class Arm(Simul.Controller):
    def update(self, info):
        t = info.time
        return {"joints": {"base_yaw": 0.6 * math.sin(0.8 * t),
                           "shoulder_pitch": 0.6 + 0.2 * math.sin(1.6 * t)}}
