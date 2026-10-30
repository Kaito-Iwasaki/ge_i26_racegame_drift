#include "rival.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(int argc,char**argv){assert(argc>1);Track t;assert(t.load(argv[1]));
for(bool up:{false,true}){assert(t.setUphill(up));Car player;player.reset(t);float times[3]={};
 for(int l=0;l<3;l++){Rival r;r.reset(t,player.ground.s,l);int n=0,drift=0,fastDrift=0;while(!r.finished&&n<40000){r.update(t,1.25f/60);n++;if(r.car.drifting){drift++;if(r.speed>30)fastDrift++;}assert(std::isfinite(r.car.x));}
 times[l]=n/60.f;printf("%s level %d %.1fs drift %.1fs high-speed %.1fs\n",up?"UP":"DOWN",l,times[l],drift/60.f,fastDrift/60.f);assert(r.finished&&fastDrift>100);}
 assert(times[0]>times[1] && times[1]>times[2]);
 for(int l:{0,1}){Rival idle;idle.reset(t,player.ground.s,l);float ps=player.ground.s;for(int i=0;i<60*35;i++)idle.update(t,1.25f/60,ps,0);
 assert(idle.s-ps>300 && idle.speed>15);
 Rival a,b;a.reset(t,player.ground.s,l);b=a;for(int i=0;i<60*15;i++){float p=ps+i*1.25f/60*20;a.update(t,1.25f/60,p,0);b.update(t,1.25f/60,p,40);}
 assert(std::fabs(a.s-b.s)<.01f);}
}
puts("PASS: rival finishes, higher-speed drift and no direct speed matching");}
