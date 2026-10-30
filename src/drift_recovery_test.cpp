#include "car.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
TrackPose Track::query(float,float,float,float) const {return {};}
void run(Car& c, const DriveInput& input, float duration, float dt) {
 Track track; for(int i=0;i<int(duration/dt+.5f);++i) c.update(dt,input,track);
}
int main(){
 for(float dt:{1.f/60,1.f/120}) for(float side:{-1.f,1.f}){
  Car entry; entry.velZ=40; DriveInput in;in.throttle=1;in.steer=side; run(entry,in,1.2f,dt);
  assert(entry.drifting && std::fabs(entry.slipAngle)>.4f);
  Car neutral=entry, counter=entry, held=entry;
  in.steer=0;run(neutral,in,1.5f,dt);
  in.steer=-.25f*side;run(counter,in,1.5f,dt);
  in.steer=side;run(held,in,1.5f,dt);
  assert(std::fabs(neutral.slipAngle)<.46f);
  assert(std::fabs(counter.slipAngle)<.34f);
  assert(std::fabs(counter.slipAngle)<std::fabs(neutral.slipAngle));
  assert(std::fabs(held.slipAngle)>.48f);
  assert(neutral.groundSpeed()*3.6f>100 && counter.groundSpeed()*3.6f>100);
 }
 puts("PASS: neutral and countersteer recovery, maintained drift, speed and symmetry");
}
