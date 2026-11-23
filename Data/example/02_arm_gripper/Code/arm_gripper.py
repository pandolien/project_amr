# -*- coding: utf-8 -*-
"""arm_gripper: 자세를 순서대로 바꾸며 그리퍼를 열고 닫는다 (상태 기계). 오른쪽 손가락은 왼쪽을 따라 움직인다 (mimic)."""
import Simul

# (이름, 관절 목표 rad / m, 걸리는 시간 s)
STEPS = [
    ("준비",   {"base_yaw": 0.0, "shoulder_pitch": 0.0, "elbow_pitch": 0.0, "finger_left_slide": 0.03}, 2.0),
    ("내리기", {"shoulder_pitch": 0.9, "elbow_pitch": 1.0}, 3.0),
    ("잡기",   {"finger_left_slide": 0.0}, 2.0),
    ("들기",   {"shoulder_pitch": 0.2, "elbow_pitch": 0.3}, 3.0),
    ("돌리기", {"base_yaw": 1.5}, 3.0),
    ("놓기",   {"finger_left_slide": 0.03}, 2.0),
]


class ArmGripper(Simul.Controller):
    def start(self):
        self.index = -1          # 지금 단계
        self.until = 0.0         # 이 단계가 끝나는 시간

    def update(self, info):
        if info.time < self.until:
            return None
        self.index = (self.index + 1) % len(STEPS)
        name, targets, seconds = STEPS[self.index]
        self.until = info.time + seconds
        print("[t=%.1f] %s" % (info.time, name))
        out = {}
        for joint, value in targets.items():
            j = info.objects[self.name].joints[joint]
            speed = abs(value - j.value) / seconds       # 이 시간 안에 닿도록 속도를 정한다
            out[joint] = (value, max(speed, 1e-3))
        return {"joints": out}
