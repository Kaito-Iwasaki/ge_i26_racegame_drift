#include "rival.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(int argc,char**argv) {
 assert(argc>1); Track t; assert(t.load(argv[1]));
 for(bool uphill:{false,true}) { assert(t.setUphill(uphill)); float times[3];
  for(int level=0;level<3;level++) {
   Rival r; r.reset(t,t.query(t.spawnX,t.spawnZ,t.spawnY).s,level); Car replay=r.car;
   int frames=0, drifting=0, wallFrames=0; float maxGain=0;
   while(!r.finished && frames<24000) {
    const bool wasDrifting=r.car.drifting; const float before=r.car.groundSpeed();
    r.update(t,1.25f/60); replay.update(1.25f/60,r.input,t);
    assert(std::hypot(replay.x-r.car.x,replay.z-r.car.z)<.001f);
    assert(std::hypot(replay.velX-r.car.velX,replay.velZ-r.car.velZ)<.001f);
    if(wasDrifting) { assert(r.input.throttle==0); drifting++; maxGain=std::max(maxGain,r.car.groundSpeed()-before); }
    if(r.car.wallContact) wallFrames++;
    frames++;
   }
   assert(r.finished && drifting>10); times[level]=frames/60.f;
   if(level==2) { assert(times[level]<260.0f); assert(wallFrames==0); }
   printf("%s level %d %.1fs drift frames %d max gain %.4f m/s\n",uphill?"UP":"DOWN",level,times[level],drifting,maxGain);
   printf("  wall contact frames %d\n",wallFrames);
  }
  assert(times[0]>times[1] && times[1]>times[2]);
 }
 puts("PASS: both directions, all difficulties, shared physics input replay and drift throttle");
}
