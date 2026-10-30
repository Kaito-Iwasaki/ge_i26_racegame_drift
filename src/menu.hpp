#pragma once
#include <algorithm>
struct MenuRect { float x,y,w,h; bool contains(float px,float py) const { return px>=x && px<x+w && py>=y && py<y+h; } };
struct MenuLayout {
 float scale,left,top;
 MenuLayout(float w,float h):scale(std::min(w/900.f,h/600.f)),left(w*.5f-405.f*scale),top(h*.5f-270.f*scale){}
 MenuRect rect(float x,float y,float w,float h) const{return {left+x*scale,top+y*scale,w*scale,h*scale};}
 MenuRect title(int i) const {
  if(i<6) return rect((i%2?421:44),233+(i/2)*58,345,36);
  if(i<9) return rect(44+(i-6)*245,407,232,36);
  return rect(44,467,722,42);
 }
 MenuRect pause(int i) const{return rect(205,200+i*78,400,60);}
 int hit(bool titleScreen,float x,float y) const {for(int i=0;i<(titleScreen?10:3);i++)if((titleScreen?title(i):pause(i)).contains(x,y))return i;return -1;}
};
inline int menuMove(bool title,int selected,int dx,int dy,bool night,bool uphill=false,bool battle=false,int difficulty=1){
 if(!title){int d=dy?dy:dx;return(selected+(d>0?1:d<0?-1:0)+3)%3;}
 int row=selected<6?selected/2:selected<9?3:4;
 if(dy){row=(row+(dy>0?1:-1)+5)%5;
  if(row==0)return night?1:0;if(row==1)return uphill?3:2;if(row==2)return battle?5:4;
  if(row==3)return 6+std::max(0,std::min(2,difficulty));return 9;}
 if(dx){if(row<3)return selected^1;if(row==3)return 6+(selected-6+(dx>0?1:2))%3;return 9;}
 return selected;
}
