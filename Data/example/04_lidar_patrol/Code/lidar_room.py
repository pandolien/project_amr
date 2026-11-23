# -*- coding: utf-8 -*-
"""lidar_room: 라이다가 달린 몸체가 방 안의 네 지점을 돌며 순찰한다 (x_slide, y_slide, yaw)."""
import math

import Simul

WAYPOINTS = [(-2.5, -1.5), (2.5, -1.5), (2.5, 1.5), (-2.5, 1.5)]   # (x, y) m
SPEED = 0.8                                                           # m/s


class LidarRoom(Simul.Controller):
    def start(self):
        self.k = 0
        self.sent = None

    def update(self, info):
        o = info.objects[self.name]
        x, y = o.joints["x_slide"].value, o.joints["y_slide"].value
        tx, ty = WAYPOINTS[self.k]
        if math.hypot(tx - x, ty - y) < 0.05:             # 도착하면 다음 지점
            self.k = (self.k + 1) % len(WAYPOINTS)
            tx, ty = WAYPOINTS[self.k]
        yaw = math.atan2(ty - y, tx - x)                  # 가는 쪽을 본다
        goal = (tx, ty, yaw)
        if goal == self.sent:
            return None
        self.sent = goal
        return {"joints": {"x_slide": (tx, SPEED), "y_slide": (ty, SPEED), "yaw": (yaw, 1.5)}}
