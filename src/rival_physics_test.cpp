#include "rival.hpp"
#include "vehicle_collision.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
Track straight(){Track t;t.closed=false;t.halfWidth=5;t.curbWidth=.7f;t.runoff=2.4f;t.length=1000;t.raceFinishS=900;
 for(int i=0;i<=200;i++){TrackSample p;p.z=i*5.f;p.s=i*5.f;t.samples.push_back(p);}t.spawnZ=100;return t;}
int main(){Track t=straight();
 for(int level=0;level<3;level++){
  Car p;p.reset(t);p.x=2.1f;p.ground=t.query(p.x,p.z,p.y);Rival r;r.reset(t,p.ground.s,level);
  assert(std::fabs(p.ground.s-r.s)<.01f && std::hypot(p.x-r.car.x,p.z-r.car.z)>4.1f);
  assert(!resolveVehicleContact(p,r.car).hit);
  DriveInput gas;gas.throttle=1;
  for(int i=0;i<120;i++){p.update(1.f/60,gas,t);r.update(t,1.f/60);assert(std::fabs(p.groundSpeed()-r.speed)<.05f);}
  assert(p.groundSpeed()<VehicleTuning::maxSpeed && r.speed<=VehicleTuning::maxSpeed);
  Car contact=p;contact.x=r.car.x+1.7f;contact.z=r.car.z;contact.velX=-8;contact.velZ=r.car.velZ;
  auto hit=resolveVehicleContact(contact,r.car);assert(hit.hit);r.acceptContact(t);
  float displaced=r.lane;assert(displaced<-2.1f);
  r.update(t,1.f/60);assert(r.lane<-2.1f && std::fabs(r.lane-displaced)<.3f);
 }
 puts("PASS: same straight acceleration and speed cap, wider grid and persistent collision response");}
