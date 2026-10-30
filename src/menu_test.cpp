#include "menu.hpp"
#include <cassert>
#include <cstdio>
int main() {
 for (float scale : {0.5f,1.0f,2.0f}) {
  MenuLayout m(1280*scale,720*scale);
  for(int i=0;i<3;i++) for(bool title:{false,true}) {auto r=title?m.title(i):m.pause(i);assert(m.hit(title,r.x+r.w*.5f,r.y+r.h*.5f)==i);}
  assert(m.hit(true,0,0)==-1);
 }
 assert(menuMove(true,2,0,-1,false)==0); assert(menuMove(true,2,0,-1,true)==1);
 assert(menuMove(true,0,1,0,false)==1); assert(menuMove(true,1,0,1,true)==2);
 assert(menuMove(false,0,0,-1,false)==2); assert(menuMove(false,2,0,1,false)==0);
 puts("PASS: title and pause hit testing, scaling and navigation");
}
