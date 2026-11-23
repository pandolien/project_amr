# -*- coding: utf-8 -*-
"""aruco_4x4_id0: 마커가 팔 앞에서 원을 그리며 돈다 (중심 (0.40, 0) m, 반지름 0.25 m, 8초에 한 바퀴)."""
import math

import Simul

CENTER = (0.40, 0.0)
RADIUS = 0.25
PERIOD = 8.0


class Aruco4x4Id0(Simul.Controller):
    def update(self, info):
        a = 2 * math.pi * info.time / PERIOD
        self.obj.set_position((CENTER[0] + RADIUS * math.cos(a), CENTER[1] + RADIUS * math.sin(a), 0.0))
        return None
