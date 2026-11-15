#ifndef CORE_3D_DHPOSE_H
#define CORE_3D_DHPOSE_H

#include "3d/dhMat.h"
#include "3d/dhQuat.h"
#include "3d/dhVector.h"

// 위치 + 방향 (강체의 자세). 이 프로젝트에서 자세를 주고받을 때의 표준 형식이다.
//   위치  : 미터 (m)
//   방향  : 단위 쿼터니언. 전송할 때의 순서는 (x, y, z, w)  (glTF, ROS 와 같다)
//   각도  : 내부와 전송은 radian, 사람에게 보여줄 때만 degree (3d/Units.h)
//
// 전송 형식(7개 값): x, y, z, qx, qy, qz, qw.
//   - 보내기(ToArray)는 쿼터니언을 정규화하고 w >= 0 으로 맞춘다 (q 와 -q 는 같은 회전이므로 비교하기 쉽게).
//   - 받기(FromArray)는 값이 유한한지, 쿼터니언 길이가 0 이 아닌지 검사하고 정규화한다.
class dhPose {
public:
    dhVector position;   // m
    dhQuat orientation;  // 단위 쿼터니언

    dhPose();  // 원점, 회전 없음
    dhPose(const dhVector& p, const dhQuat& q);

    static dhPose FromMat(const dhMat& m);  // 회전 + 이동 행렬 (스케일이 섞여 있으면 안 된다)
    dhMat ToMat() const;

    dhPose Normalized() const;  // 쿼터니언만 정규화
    dhPose Inv() const;         // 역변환
    dhPose operator*(const dhPose& o) const;      // 변환의 합성: (this * o) 는 o 를 먼저 적용한 뒤 this 를 적용
    dhVector operator*(const dhVector& p) const;  // 점 변환

    void ToArray(double out[7]) const;
    static bool FromArray(const double in[7], dhPose* out);
};

#endif  // CORE_3D_DHPOSE_H
