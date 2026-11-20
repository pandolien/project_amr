// 엔진 테스트용 가짜 Backend: 오브젝트 목록을 직접 가진다
#ifndef TESTS_FAKESIMBACKEND_H
#define TESTS_FAKESIMBACKEND_H

#include "CSimul/Engine.h"

namespace {

using simul::Array;
using simul::String;
using namespace simul::sim;

struct FakeBackend : simul::sim::Backend {
    Array<ObjectInfo> objs;
    Array<ContactInfo> contacts;
    int poseCalls;
    FakeBackend() : poseCalls(0) {}
    int ObjectCount() const override { return static_cast<int>(objs.size()); }
    bool ReadObject(int i, ObjectInfo* out) const override {
        if (i < 0 || static_cast<size_t>(i) >= objs.size()) return false;
        *out = objs[static_cast<size_t>(i)];
        return true;
    }
    void ReadCollisions(Array<ContactInfo>* out) const override { *out = contacts; }
    bool SetPose(int i, const dhPose& p, String*) override {
        poseCalls++;
        objs[static_cast<size_t>(i)].pose = p;
        return true;
    }
    bool SetJoint(int i, const String& j, double v, String* err) override {
        for (size_t k = 0; k < objs[static_cast<size_t>(i)].joints.size(); k++)
            if (objs[static_cast<size_t>(i)].joints[k].name == j) {
                objs[static_cast<size_t>(i)].joints[k].value = v;
                return true;
            }
        if (err) *err = "no joint";
        return false;
    }
    void add(const char* name, bool robot) {
        ObjectInfo o;
        o.name = name;
        o.kind = robot ? "robot" : "model";
        if (robot) {
            JointInfo j;
            j.name = "j1";
            j.kind = kJointRevoluteKind;
            j.lower = -1.0;
            j.upper = 1.0;
            j.hasLimit = true;
            j.velocity = 2.0;
            o.joints.push_back(j);
        }
        objs.push_back(o);
    }
};

}  // namespace

#endif
