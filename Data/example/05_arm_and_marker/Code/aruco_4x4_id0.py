# -*- coding: utf-8 -*-
"""aruco_4x4_id0: 마커를 팔 앞에 놓고 좌우로 천천히 옮긴다. mesh(.stl)는 OBJECT/OBJ/ 에 있다."""
import math

import Simul


class Aruco4x4Id0(Simul.Controller):
    def start(self):
        self.obj.set_pose((0.5, 0.0, 0.0))                # 시작 위치 (m)

    def update(self, info):
        return {"position": (0.5, 0.3 * math.sin(0.5 * info.time), 0.0), "speed": 0.3}
