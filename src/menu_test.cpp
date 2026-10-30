#include "menu.hpp"
#include <cassert>
#include <cstdio>
int main(){for(float s:{.5f,1.f,2.f}){MenuLayout m(1280*s,720*s);
 for(int i=0;i<9;i++){auto r=m.title(i);assert(m.hit(true,r.x+r.w*.5f,r.y+r.h*.5f)==i);}
 for(int i=0;i<3;i++){auto r=m.pause(i);assert(m.hit(false,r.x+r.w*.5f,r.y+r.h*.5f)==i);}}
 assert(menuMove(true,8,0,-1,false,false,true,2)==7);
 assert(menuMove(true,3,0,1,false,false,false,0)==4);
 assert(menuMove(true,4,1,0,false,false,false,0)==5);
 assert(menuMove(true,7,1,0,false,false,true,2)==4);
 assert(menuMove(false,0,0,-1,false)==2);
 puts("PASS: difficulty and title/pause menu navigation");}
