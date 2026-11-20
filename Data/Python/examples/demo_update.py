# -*- coding: utf-8 -*-
"""update(info) 방식 예제: Main 이 매 틱 정보를 넘기고, 함수는 오브젝트 움직임 정보를 돌려준다.

    Main  --(오브젝트 정보, 시간, 충돌)-->  update(info)
    Main  <--(오브젝트 움직임 정보)--------  update(info)

arm 의 shoulder_pitch 를 시간에 따라 흔들고, 무언가와 부딪히면 로그를 남기고 멈춘다.
"""
import math

from Main import register


def update(info):
    arm = info.objects.get("arm")
    if arm is None:
        return None

    hit = info.collided("arm")                         # arm 이 들어 있는 충돌 쌍
    if hit:
        print("[t=%.2f] 충돌: %s" % (info.time, hit[0]))
        return {"arm": {"joints": {"shoulder_pitch": (arm.joints["shoulder_pitch"].value, 1.0)}}}   # 지금 값에 머문다

    target = 0.6 * math.sin(info.time)                 # 오브젝트 움직임 정보
    return {"arm": {"joints": {"shoulder_pitch": target}}}


register(update)
