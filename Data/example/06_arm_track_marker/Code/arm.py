# -*- coding: utf-8 -*-
"""arm: 돌고 있는 마커를 따라 몸통(base_yaw)을 돌리고, 마커가 멀면 어깨(shoulder_pitch)를 더 숙인다.
다른 오브젝트의 위치는 info.objects["이름"].position 으로 읽는다."""
import math

import Simul

TARGET = "aruco_4x4_id0"        # 따라갈 오브젝트 (OBJECT/ 의 URDF 파일 이름)


class Arm(Simul.Controller):
    def update(self, info):
        t = info.objects.get(TARGET)
        if t is None:
            return None
        base = info.objects[self.name].position
        dx, dy = t.position[0] - base[0], t.position[1] - base[1]
        yaw = math.atan2(dy, dx)                           # 마커가 있는 방향
        reach = math.hypot(dx, dy)                         # 마커까지 거리 (m)
        pitch = max(0.0, min(1.2, (reach - 0.15) * 2.5))   # 멀수록 많이 숙인다
        return {"joints": {"base_yaw": (yaw, 2.0), "shoulder_pitch": (pitch, 1.5)}}
