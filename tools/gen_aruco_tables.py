#!/usr/bin/env python3
"""ArUco 사전 표를 OpenCV 에서 뽑아 Core/src/CMarker/ArucoTables.inc 를 만든다 (한 번만 돌리면 된다. 결과 파일을 저장소에 넣어 둔다).

    pip install opencv-contrib-python-headless
    python3 tools/gen_aruco_tables.py > Core/src/CMarker/ArucoTables.inc

OpenCV 의 DICT_4X4_1000, DICT_5X5_1000, DICT_6X6_1000, DICT_7X7_1000 (50/100/250 은 앞쪽 부분과 같다).
각 마커는 n*n 비트를 행 우선, 한 바이트에 8비트(앞 비트가 높은 자리)로 담고, 마지막 바이트가 모자라면 낮은 자리에 채운다 (회전 0 도 기준).
"""
import cv2

aruco = cv2.aruco
print("// 자동 생성: tools/gen_aruco_tables.py (OpenCV %s). 손으로 고치지 않는다." % cv2.__version__)
for n in (4, 5, 6, 7):
    d = aruco.getPredefinedDictionary(getattr(aruco, "DICT_%dX%d_1000" % (n, n)))
    nb = (n * n + 7) // 8
    print("static const unsigned char kTable%dx%d[1000 * %d] = {" % (n, n, nb))
    for i in range(1000):
        row = d.bytesList[i].reshape(-1)[:nb]
        print("    " + ",".join("%d" % v for v in row) + ",")
    print("};")
