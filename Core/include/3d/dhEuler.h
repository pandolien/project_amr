#ifndef CORE_3D_DHEULER_H
#define CORE_3D_DHEULER_H

class dhMat;
class dhQuat;

// 오일러각 (radian). R = Rz(a) * Ry(b) * Rx(r)
//   a: Z축 회전 (yaw), b: Y축 회전 (pitch), r: X축 회전 (roll)
class dhEuler {
public:
    dhEuler();
    dhEuler(float pa, float pb, float pr);
    ~dhEuler();

    float a, b, r;

    dhMat GetMat() const;
    dhQuat GetQuat() const;
};

#endif  // CORE_3D_DHEULER_H
