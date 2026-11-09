#ifndef CORE_3D_DHVERTEX_H
#define CORE_3D_DHVERTEX_H

#include "3d/dhVector.h"

class dhVertex {
public:
    dhVertex();
    ~dhVertex();

    void Initialization();
    void Close();

    float x, y, z;
    float nx, ny, nz;

    void SetPoint(float px, float py, float pz);
    void SetPoint(const dhVector& p);
    void SetNomalVector(float pnx, float pny, float pnz);
    void SetNomalVector(const dhVector& n);
    dhVector GetPoint() const;
    dhVector GetNormalVector() const;
    void AddNrom(const dhVector& n);
};

#endif  // CORE_3D_DHVERTEX_H
