#include "3d/dhEuler.h"

#include "3d/dhMat.h"
#include "3d/dhQuat.h"

dhEuler::dhEuler() : a(0), b(0), r(0) {}

dhEuler::dhEuler(float pa, float pb, float pr) : a(pa), b(pb), r(pr) {}

dhEuler::~dhEuler() {
    a = 0;
    b = 0;
    r = 0;
}

dhMat dhEuler::GetMat() const {
    // dhMat::Hmatrix 와 같은 규약: Rz(a) * Ry(b) * Rx(r)
    return dhMat::RotZ(a) * dhMat::RotY(b) * dhMat::RotX(r);
}

dhQuat dhEuler::GetQuat() const { return GetMat().GetQuat(); }
