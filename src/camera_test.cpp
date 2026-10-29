#include "camera.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const {return {};}
int main(){
    Car car;car.x=12;car.y=50;car.z=-8;car.driftAmount=1;car.slipAngle=.6f;car.velX=20;car.velZ=-10;
    for(float yaw:{-3.13f,-1.2f,0.f,1.4f,3.13f}) {
        car.yaw=yaw;
        Vec3 f{std::sin(yaw),0,std::cos(yaw)},r{std::cos(yaw),0,-std::sin(yaw)}, pos{car.x,car.y,car.z};
        for(int mode=0;mode<3;++mode){
            auto cam=carCameraPose(car,mode,.1f,.1f);
            assert(std::fabs(dot(cam.eye-pos,r))<.0001f);
            assert(dot(cam.eye-pos,f)<0);
            assert(std::fabs(dot(cam.look-cam.eye,r))<.0001f);
            float original=std::atan2(cam.look.x-cam.eye.x,cam.look.z-cam.eye.z);
            car.slipAngle=-.6f;auto flipped=carCameraPose(car,mode,.1f,.1f);
            assert(std::fabs(wrapPi(std::atan2(flipped.look.x-flipped.eye.x,flipped.look.z-flipped.eye.z)-original))<.0001f);
        }
        auto hood=carCameraPose(car,kHoodCameraMode,.12f,-.08f);
        Vec3 expected=carPointWorld(car,.12f,-.08f,{0,1.28f,.72f});
        assert(length(hood.eye-expected)<.0001f);
        Vec3 dir=normalize(hood.look-hood.eye), horizontal=normalize(cross(hood.up,dir)), vertical=normalize(cross(dir,horizontal));
        Vec3 bonnet=carPointWorld(car,.12f,-.08f,{0,.66f,1.9f})-hood.eye;
        assert(dot(bonnet,dir)>.15f);
        assert(std::fabs(dot(bonnet,vertical)/dot(bonnet,dir)) < std::tan(62.f*3.14159265f/360.f));
    }
    assert(kCameraModeCount==4);
    std::puts("PASS: chase rigid behind yaw through wrap and slip; hood mount, orientation and bonnet field of view");
}
