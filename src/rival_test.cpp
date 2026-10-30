#include "rival.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(int argc,char**argv){assert(argc>1);Track t;assert(t.load(argv[1]));
for(bool up:{false,true}) {assert(t.setUphill(up));Car player;player.reset(t);player.x+=player.ground.rightX*1.6f;player.z+=player.ground.rightZ*1.6f;player.ground=t.query(player.x,player.z,player.y,player.ground.s);
for(int l=0;l<3;l++){Rival r;r.reset(t,player.ground.s,l);assert(std::fabs(r.s-player.ground.s)<.01f);assert(std::hypot(player.x-r.car.x,player.z-r.car.z)>3.0f);int steps=0;float maxRoll=0,maxPitch=0;while(!r.finished&&steps<40000){r.update(t,1.25f/60);maxRoll=std::max(maxRoll,std::fabs(r.car.bodyRoll));maxPitch=std::max(maxPitch,std::fabs(r.car.bodyPitch));assert(std::isfinite(r.car.x));steps++;}printf("%s level %d %.1fs roll %.3f pitch %.3f\n",up?"UP":"DOWN",l,steps/60.f,maxRoll,maxPitch);const float faster[3]={250,235,223};assert(r.finished&&maxRoll>.01f&&maxPitch>.01f&&steps/60.f<faster[l]);}
for(int l:{0,1,2}){Car startPlayer=player;Rival launch;launch.reset(t,player.ground.s,l);DriveInput gas;gas.throttle=1;
 for(int i=0;i<120;i++){startPlayer.update(1.25f/60,gas,t);launch.update(t,1.25f/60,startPlayer.ground.s,startPlayer.groundSpeed());}
 assert(std::fabs(startPlayer.ground.s-launch.s)<4.0f);}
for(int l:{0,1}){Rival r;r.reset(t,player.ground.s,l);float ps=player.ground.s;for(int i=0;i<60*35;i++){ps+=10*1.25f/60;r.update(t,1.25f/60,ps,10);}assert(std::fabs(r.speed-10)<2.5f && std::fabs(r.s-ps)<35);
 Rival late;late.reset(t,player.ground.s,l);late.s=t.raceStartS+(t.raceFinishS-t.raceStartS)*.65f;late.sync(t,0);
 for(int i=0;i<60*15;i++)late.update(t,1.25f/60,late.s-80,0);
 if(l==0)assert(late.speed<1);else assert(late.speed>15);
}
}puts("PASS: speed, following, start spacing and suspension motion");}
