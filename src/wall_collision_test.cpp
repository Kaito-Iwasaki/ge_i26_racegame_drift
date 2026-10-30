#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
Track straight() {
 Track t; t.closed=false; t.touge=true; t.halfWidth=5; t.curbWidth=0; t.runoff=2; t.length=200;
 for(int i=0;i<=100;i++){ TrackSample s; s.z=i*2.f-100; s.s=i*2.f; t.samples.push_back(s); }
 return t;
}
void verifyCorners(const Track& t,const Car& c){
 float fx=std::sin(c.yaw),fz=std::cos(c.yaw),rx=std::cos(c.yaw),rz=-std::sin(c.yaw);
 for(float a:{-.95f,.95f}) for(float b:{-2.2f,2.2f}){
   auto p=t.query(c.x+rx*a+fx*b,c.z+rz*a+fz*b,c.y,c.ground.s);
   assert(p.found && std::fabs(p.lateral)<=t.wallLateral()-.05f+.004f);
 }
}
int main(){
 Track t=straight();
 for(float yaw:{0.f,0.5f,1.2f,1.570796f,-0.8f}) for(float side:{-1.f,1.f}) {
  Car c; c.reset(t); c.yaw=yaw; c.x=side*6.8f; c.z=0; c.ground=t.query(c.x,c.z,c.y);
  c.update(1.f/120.f,{},t); assert(c.wallContact && c.wallNewContact && c.wallSide==side); verifyCorners(t,c);
  assert(std::fabs(std::fabs(c.wallContactX)-(t.wallLateral()-.05f))<0.01f);
  c.update(1.f/120.f,{},t); verifyCorners(t,c);
 }
 Car c;c.reset(t); c.x=5.95f;c.z=0;c.ground=t.query(c.x,c.z,c.y);c.velX=12;c.velZ=20;
 c.update(1.f/120.f,{},t); assert(c.wallContact && c.wallImpact>1 && c.wallAlongSpeed>1);verifyCorners(t,c);
 float old=c.x; c.velX=-3; c.velZ=0; c.update(1.f/120.f,{},t);assert(c.x<old);
 puts("PASS: straight, angled and sideways body on both rails; impact, tangential speed and release");
}
