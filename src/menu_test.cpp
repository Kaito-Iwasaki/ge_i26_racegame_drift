#include "menu.hpp"
#include <cassert>
#include <cstdio>
int main() {
 for(float scale:{.5f,1.f,2.f}) {MenuLayout m(1280*scale,720*scale);
  for(int i=0;i<5;++i){auto r=m.title(i);assert(m.hit(true,r.x+r.w/2,r.y+r.h/2)==i);}
  for(int i=0;i<3;++i){auto r=m.pause(i);assert(m.hit(false,r.x+r.w/2,r.y+r.h/2)==i);}
  assert(m.hit(true,0,0)==-1);
 }
 assert(menuMove(true,4,0,-1,false,false)==2);
 assert(menuMove(true,4,0,-1,true,true)==3);
 assert(menuMove(true,2,1,0,false,false)==3);
 assert(menuMove(true,3,0,1,true,true)==4);
 assert(menuMove(true,0,0,1,false,false)==2);
 assert(menuMove(false,0,0,-1,false)==2);
 puts("PASS: day, night, direction and start menu hit testing and navigation");
}
