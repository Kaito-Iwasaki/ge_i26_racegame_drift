#include "vehicle_collision.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(){
 Car a,b;a.x=0;b.x=10;assert(!resolveVehicleContact(a,b).hit);
 b.x=1.6f;b.y=3;assert(!resolveVehicleContact(a,b).hit);b.y=0;
 a.velX=5;b.velX=-1;auto hit=resolveVehicleContact(a,b);
 assert(hit.hit && hit.impact>5.9f && a.x<0 && b.x>1.6f && a.velX<5 && b.velX>-1);
 assert(!resolveVehicleContact(a,b).hit);
 a=Car{};b=Car{};b.z=4.1f;a.velZ=10;b.velZ=0;hit=resolveVehicleContact(a,b);
 assert(hit.hit && hit.impact>9.9f && a.velZ<10 && b.velZ>0);
 a=Car{};b=Car{};b.x=2.0f;b.yaw=.6f;a.velX=-2;b.velX=2;
 float va=a.velX,vb=b.velX;hit=resolveVehicleContact(a,b);
 assert(hit.hit && hit.impact==0 && a.velX==va && b.velX==vb);
 puts("PASS: side, rear, angled, separation, impulse and different road heights");
}
