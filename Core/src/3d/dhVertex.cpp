#include "3d/dhVertex.h"

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
