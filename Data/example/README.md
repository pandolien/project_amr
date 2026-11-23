# 예제 프로젝트

각 폴더가 하나의 프로젝트이다 (main.py / OBJECT/ / Code/ / Simul/).
`Simul/` 의 Simul.py 와 DLL(so)은 저장소에 두지 않는다. 에디터(`simul_editor`)로 폴더를 열면 "Simul/ 을 채울까요?" 하고 물으니 "예" 를 누른다
(명령줄은 `simul_editor --upgrade 예제폴더`). 그 뒤 `python main.py` (창 없이는 `--no-window`) 로 돌려도 된다.

| 폴더 | 내용 |
|---|---|
| 01_arm_swing | arm 을 사인파로 흔들고 충돌을 알린다 |
| 02_arm_gripper | arm_gripper 가 내리기 → 잡기 → 들기 → 돌리기 → 놓기 를 되풀이 (상태 기계) |
| 03_camera_pan_tilt | 카메라 pan/tilt 로 훑기 |
| 04_lidar_patrol | 라이다 몸체가 방 안 네 지점을 순찰 (x_slide, y_slide, yaw) |
| 05_arm_and_marker | arm 과 ArUco 마커 두 오브젝트 (mesh 가 OBJECT/OBJ/ 에 모인 예) |
