# -*- coding: utf-8 -*-
"""camera_scene: 카메라를 좌우(pan)로 훑고 위아래(tilt)로 천천히 끄덕인다."""
import math

import Simul


class CameraScene(Simul.Controller):
    def update(self, info):
        t = info.time
        return {"joints": {"pan": 1.2 * math.sin(0.5 * t),
                           "tilt": 0.5 * math.sin(0.9 * t)}}
