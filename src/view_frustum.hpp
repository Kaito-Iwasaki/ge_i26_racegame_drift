#pragma once
#include "geom.hpp"
#include <cmath>
struct ViewFrustum {
    struct Plane { float x,y,z,d,normalLength; } planes[6];
    explicit ViewFrustum(const Mat4& m) {
        for(int i=0;i<6;++i){int axis=i/2;float sign=(i%2==0)?1.0f:-1.0f;
            Plane& p=planes[i];p.x=m.m[3]+sign*m.m[axis];p.y=m.m[7]+sign*m.m[4+axis];
            p.z=m.m[11]+sign*m.m[8+axis];p.d=m.m[15]+sign*m.m[12+axis];
            p.normalLength=std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);}
    }
    bool intersectsSphere(Vec3 p,float radius) const {
        for(const Plane& f:planes) if(f.x*p.x+f.y*p.y+f.z*p.z+f.d < -radius*f.normalLength) return false;
        return true;
    }
};
