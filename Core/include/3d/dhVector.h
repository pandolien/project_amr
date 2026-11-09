#ifndef CORE_3D_DHVECTOR_H
#define CORE_3D_DHVECTOR_H

class dhVector {
public:
    dhVector();
    dhVector(float px, float py);
    dhVector(float px, float py, float pz);

    float x, y, z;

    dhVector operator+(const dhVector& o) const;
    dhVector operator-(const dhVector& o) const;
    dhVector operator*(const dhVector& o) const;  // 외적 (cross product)
    dhVector operator*(float s) const;
    dhVector operator/(float s) const;

    float Norm() const;
    dhVector Unit() const;
    float Norm(const dhVector& o) const;     // |this - o|
    dhVector Unit(const dhVector& o) const;  // (this - o) 방향의 단위벡터
    dhVector Rand() const;                   // TODO: 미구현 (영벡터 반환)
    float Dot(const dhVector& o) const;
};

#endif  // CORE_3D_DHVECTOR_H
