#include "3d/dhVertex.h"

#include <cmath>

dhVertex::dhVertex() { Initialization(); }

dhVertex::~dhVertex() { Close(); }

void dhVertex::Initialization() {
    x = 0; y = 0; z = 0;
    nx = 0; ny = 0; nz = 0;
}

void dhVertex::Close() {
    x = 0; y = 0; z = 0;
    nx = 0; ny = 0; nz = 0;
}

void dhVertex::SetPoint(float px, float py, float pz) {
    x = px;
    y = py;
    z = pz;
}

void dhVertex::SetPoint(const dhVector& p) {
    x = p.x;
    y = p.y;
    z = p.z;
}

void dhVertex::SetNomalVector(float pnx, float pny, float pnz) {
    nx = pnx;
    ny = pny;
    nz = pnz;
}

void dhVertex::SetNomalVector(const dhVector& n) {
    nx = n.x;
    ny = n.y;
    nz = n.z;
}

dhVector dhVertex::GetPoint() const { return dhVector(x, y, z); }

dhVector dhVertex::GetNormalVector() const { return dhVector(nx, ny, nz); }

void dhVertex::AddNrom(const dhVector& n) {
    nx += n.x;
    ny += n.y;
    nz += n.z;
}

float dhVertex::distance(const dhVertex* vt) const {
    float xx = x - vt->x;
    float yy = y - vt->y;
    float zz = z - vt->z;
    return std::sqrt(xx * xx + yy * yy + zz * zz);
}

float dhVertex::at(int i) const {
    switch (i) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
    }
    return 0.0f;
}

float dhVertex::operator[](int i) const { return at(i); }
