#include "rival.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(int argc,char**argv){assert(argc>1);Track t;assert(t.load(argv[1]));
 for(bool up:{false,true}) {assert(t.setUphill(up)); Car player;player.reset(t);
  float times[3];
  for(int level=0;level<3;level++){Rival n;n.reset(t,player.ground.s,level);float last=n.s;int drifting=0;int steps=0;
   for(;steps<200000&&!n.finished;steps++){n.update(t,1.f/60);assert(std::isfinite(n.car.x)&&std::isfinite(n.car.yaw));assert(n.s>=last && n.s-last<1);assert(std::fabs(n.car.ground.lateral)<t.halfWidth);last=n.s;if(n.car.drifting)drifting++;}
   assert(n.finished && drifting>100);times[level]=steps/60.f;
   std::printf("%s level %d time %.1fs drift %.1fs\n",up?"UP":"DOWN",level,times[level],drifting/60.f);
  }
  assert(times[0]>times[1] && times[1]>times[2]);
 }
 puts("PASS: both directions finish, drift, road bounds and difficulty ordering");}
