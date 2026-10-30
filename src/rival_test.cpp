#include "rival.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(int argc,char**argv){assert(argc>1);Track t;assert(t.load(argv[1]));
 for(bool up:{false,true}){assert(t.setUphill(up));Car player;player.reset(t);
  player.x+=player.ground.rightX*1.25f;player.z+=player.ground.rightZ*1.25f;
  player.ground=t.query(player.x,player.z,player.y,player.ground.s);
  Rival grid;grid.reset(t,player.ground.s,1);
  assert(std::fabs(player.ground.s-grid.car.ground.s)<.01f);
  assert(std::hypot(player.x-grid.car.x,player.z-grid.car.z)>2.3f);
  for(int level=0;level<3;level++){
   Rival r;r.reset(t,player.ground.s,level);assert(std::fabs(r.s-player.ground.s)<.01f);
   float last=r.s;int drifting=0,steps=0;const float dt=1.25f/60;
   while(!r.finished && steps<40000){r.update(t,dt);assert(std::isfinite(r.car.x)&&std::isfinite(r.car.yaw));assert(r.s>=last && r.s-last<2);assert(std::fabs(r.car.ground.lateral)<t.halfWidth);last=r.s;drifting+=r.car.drifting;steps++;}
   const float time=steps/60.f;const float target[3]={260,240,225};
   std::printf("%s level %d %.1fs target %.0fs drift %.1fs\n",up?"UP":"DOWN",level,time,target[level],drifting/60.f);
   assert(r.finished && drifting>100 && std::fabs(time-target[level])<4.0f);
  }
  for(int level:{0,1}){
   Rival slow;slow.reset(t,player.ground.s,level);float ps=player.ground.s;
   for(int i=0;i<60*35;i++){ps+=10*1.25f/60;slow.update(t,1.25f/60,ps,10);}
   assert(std::fabs(slow.speed-10)<2.5f && std::fabs(slow.s-ps)<35);
   Rival fast;fast.reset(t,player.ground.s,level);float fastPlayer=player.ground.s;
   for(int i=0;i<60*20;i++){fastPlayer+=45*1.25f/60;fast.update(t,1.25f/60,fastPlayer,45);}
   assert(fast.speed>slow.speed+5);
   Rival late;late.reset(t,player.ground.s,level);late.s=t.raceStartS+(t.raceFinishS-t.raceStartS)*.65f;late.sync(t,0);
   for(int i=0;i<60*15;i++)late.update(t,1.25f/60,late.s-80,0);
   if(level==0)assert(late.speed<1);else assert(late.speed>15);
  }
 }
 puts("PASS: target times, follow behavior, difficulty handoff, drift and starts");}
