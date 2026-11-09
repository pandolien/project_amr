#include "3d/dhVector.h"

#include <cmath>

dhVector::dhVector() : x(0), y(0), z(0) {}

dhVector::dhVector(float px, float py) : x(px), y(py), z(0) {}

dhVector::dhVector(float px, float py, float pz) : x(px), y(py), z(pz) {}

dhVector dhVector::operator+(const dhVector& o) const {
    return dhVector(x + o.x, y + o.y, z + o.z);
}

dhVector dhVector::operator-(const dhVector& o) const {
    return dhVector(x - o.x, y - o.y, z - o.z);
}

dhVector dhVector::operator*(const dhVector& o) const {
    return dhVector(y * o.z - z * o.y,
                    z * o.x - x * o.z,
                    x * o.y - y * o.x);
}

dhVector dhVector::operator*(float s) const { return dhVector(x * s, y * s, z * s); }

dhVector dhVector::operator/(float s) const { return dhVector(x / s, y / s, z / s); }

float dhVector::Norm() const { return std::sqrt(x * x + y * y + z * z); }

dhVector dhVector::Unit() const {
    float n = Norm();
    return dhVector(x / n, y / n, z / n);
}

float dhVector::Norm(const dhVector& o) const {
    float dx = x - o.x;
    float dy = y - o.y;
    float dz = z - o.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

dhVector dhVector::Unit(const dhVector& o) const {
    float n = Norm(o);
    return dhVector((x - o.x) / n, (y - o.y) / n, (z - o.z) / n);
}

dhVector dhVector::Rand() const { return dhVector(); }

float dhVector::Dot(const dhVector& o) const { return x * o.x + y * o.y + z * o.z; }
