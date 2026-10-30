#include "rival.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
float findBend(const Track&t,bool late){for(const auto& x:t.samples){if((x.s>t.raceFinishS*.6f)!=late)continue;auto a=Rival::pose(t,x.s+4),b=Rival::pose(t,x.s+45);if(std::fabs(wrapPi(b.yaw-a.yaw))>.7f)return x.s;}assert(false);return 0;}
float runGap(const Track&t,int level,float at,float gap,float playerSpeed){Rival r;r.reset(t,at,level);r.s=at;r.speed=20;r.sync(t,0);for(int i=0;i<120;i++)r.update(t,1.f/60,r.s+gap,playerSpeed);return r.speed;}
int main(int argc,char**argv){assert(argc>1);Track t;assert(t.load(argv[1]));
for(bool up:{false,true}){assert(t.setUphill(up));Car player;player.reset(t);float times[3];
 for(int l=0;l<3;l++){Rival r;r.reset(t,player.ground.s,l);int n=0,fastDrift=0;while(!r.finished&&n<40000){r.update(t,1.25f/60);n++;if(r.car.drifting&&r.speed>30)fastDrift++;assert(std::isfinite(r.car.x));}times[l]=n/60.f;assert(r.finished&&fastDrift>100);printf("%s level %d %.1fs\n",up?"UP":"DOWN",l,times[l]);}
 assert(times[0]>times[1] && times[1]>times[2]);
 for(int l:{0,1})for(bool late:{false,true}){float at=findBend(t,late);float base=runGap(t,l,at,0,40),ahead=runGap(t,l,at,-300,0),behind=runGap(t,l,at,300,42),slower=runGap(t,l,at,300,5);
 printf("  lvl %d %s base %.1f lead %.1f catch %.1f slow %.1f\n",l,late?"late":"early",base,ahead,behind,slower);
 assert(behind>base+.5f && slower>=base-.1f && behind<=VehicleTuning::maxSpeed);
 if(l==0 || !late)assert(ahead<base-.5f && ahead>10);else assert(std::fabs(ahead-base)<.01f);
 Rival idle;idle.reset(t,at,l);for(int i=0;i<60*20;i++)idle.update(t,1.f/60,at,0);assert(idle.s-at>150 && idle.speed>10);
 }
}
puts("PASS: difficulty pace, limited lead adjustment and catch-up throughout course");}
