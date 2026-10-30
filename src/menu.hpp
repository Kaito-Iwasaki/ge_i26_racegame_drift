#pragma once
#include <algorithm>
struct MenuRect { float x,y,w,h; bool contains(float px,float py) const { return px>=x && px<x+w && py>=y && py<y+h; } };
struct MenuLayout {
    float scale, left, top;
    MenuLayout(float width,float height) : scale(std::min(width/900.f,height/600.f)), left(width*.5f-405.f*scale), top(height*.5f-270.f*scale) {}
    MenuRect rect(float x,float y,float w,float h) const { return {left+x*scale,top+y*scale,w*scale,h*scale}; }
    MenuRect title(int i) const { if(i==0)return rect(44,378,345,50); if(i==1)return rect(421,378,345,50); return rect(44,448,722,62); }
    MenuRect pause(int i) const { return rect(205,200+i*78,400,60); }
    int hit(bool titleScreen,float x,float y) const { for(int i=0;i<3;i++) if((titleScreen?title(i):pause(i)).contains(x,y)) return i; return -1; }
};
inline int menuMove(bool title,int selected,int dx,int dy,bool night) {
    if (title) {
        if (dy!=0) return selected==2 ? (night?1:0) : 2;
        if (dx!=0) return selected==2 ? (night?1:0) : (selected==0?1:0);
        return selected;
    }
    int dir=dy!=0?dy:dx;
    return (selected + (dir>0?1:dir<0?-1:0)+3)%3;
}
