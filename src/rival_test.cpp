#include "rival.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(int argc,char**argv) {
 assert(argc>1); Track t; assert(t.load(argv[1]));
 for(bool uphill:{false,true}) { assert(t.setUphill(uphill)); float times[3];
  for(int level=0;level<3;level++) {
   Rival r; r.reset(t,t.query(t.spawnX,t.spawnZ,t.spawnY).s,level); Car replay=r.car;
   int frames=0, drifting=0, wallFrames=0, brakeFrames=0, exitAcceleration=0, driftThrottle=0, offRoadFrames=0; float maxGain=0, brakeTotal=0;
   while(!r.finished && frames<24000) {
    const bool wasDrifting=r.car.drifting; const float before=r.car.groundSpeed();
    r.update(t,1.25f/60); replay.update(1.25f/60,r.input,t);
    if(wasDrifting) assert(r.input.brake<=.08001f);
    if(!wasDrifting && r.driftExitTime>0 && r.input.throttle>0) { assert(r.input.brake==0); exitAcceleration++; }
    assert(std::hypot(replay.x-r.car.x,replay.z-r.car.z)<.001f);
    assert(std::hypot(replay.velX-r.car.velX,replay.velZ-r.car.velZ)<.001f);
    if(wasDrifting) { if(r.input.throttle>0) { assert(r.input.throttle<=1 && r.input.brake==0); driftThrottle++; } drifting++; maxGain=std::max(maxGain,r.car.groundSpeed()-before); }
    if(r.car.wallContact) wallFrames++;
    if(!r.car.onAsphalt) offRoadFrames++;
    if(r.input.brake>.05f) brakeFrames++; brakeTotal+=r.input.brake;
    frames++;
   }
   assert(r.finished && drifting>10); assert(wallFrames==0); times[level]=frames/60.f;
   if(level==0) assert(times[level]>223.0f && times[level]<229.0f);
   if(level==1) assert(times[level]>217.0f && times[level]<223.0f);
   if(level==2) { assert(times[level]<217.0f); assert(wallFrames==0); assert(offRoadFrames<240); assert(exitAcceleration>0); assert(driftThrottle>0); assert(brakeFrames<35*60); assert(brakeTotal/60.0f<8.0f); }
   printf("%s level %d %.1fs drift frames %d max gain %.4f m/s\n",uphill?"UP":"DOWN",level,times[level],drifting,maxGain);
   printf("  wall contact frames %d, off-road frames %d\n",wallFrames,offRoadFrames);
  }
  assert(times[0]>times[1] && times[1]>times[2]);
  // A far-ahead player must not make catch-up overspeed the corners.
  for(int level=0;level<2;level++) for(int gap : {-300,300}) {
   Rival r; r.reset(t,t.query(t.spawnX,t.spawnZ,t.spawnY).s,level);
   int frames=0, wallFrames=0, offRoadFrames=0;
   while(!r.finished && frames<24000) {
    r.update(t,1.25f/60,r.s+float(gap),35.0f);
    if(r.car.wallContact) wallFrames++;
    if(!r.car.onAsphalt) offRoadFrames++;
    frames++;
   }
   assert(r.finished && wallFrames==0);
  }
 }
 puts("PASS: both directions, all difficulties, shared physics input replay and controlled drift throttle");
}
