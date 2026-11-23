# -*- coding: utf-8 -*-
"""오브젝트 제어 class 의 견본. 이 파일을 복사해 <오브젝트 이름>.py 로 바꾸면 그 오브젝트의 코드가 된다 (오브젝트 이름 = OBJECT/ 의 URDF 파일 이름).
class 이름은 자유이고, Simul.Controller 를 이어받으면 main.py 가 오브젝트마다 하나씩 만들어 연결한다."""
import Simul


class Object(Simul.Controller):
    """self.sim: 시뮬레이션, self.obj: 이 오브젝트, self.name: 오브젝트 이름"""

    def start(self):
        """시작할 때 한 번"""

    def update(self, info):
        """매 스텝. info.time, info.dt, info.objects["이름"] (.position .quaternion .joints[관절].value/.lower/.upper), info.collided("이름")
        돌려주는 값: {"joints": {관절: 목표 rad 또는 (목표, 속도)}, "position": (x, y, z), "speed": m/s}   (없으면 None)"""
        return None

    def stop(self):
        """끝날 때 한 번"""
