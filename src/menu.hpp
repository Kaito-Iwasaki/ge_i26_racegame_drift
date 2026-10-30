#pragma once
#include <algorithm>
struct MenuRect { float x,y,w,h; bool contains(float px,float py) const { return px>=x && px<x+w && py>=y && py<y+h; } };
struct MenuLayout {
    float scale, left, top;
    MenuLayout(float width,float height) : scale(std::min(width/900.f,height/600.f)), left(width*.5f-405.f*scale), top(height*.5f-270.f*scale) {}
    MenuRect rect(float x,float y,float w,float h) const { return {left+x*scale,top+y*scale,w*scale,h*scale}; }
    MenuRect title(int i) const { if(i==0)return rect(44,340,345,42); if(i==1)return rect(421,340,345,42); if(i==2)return rect(44,405,345,42); if(i==3)return rect(421,405,345,42); return rect(44,462,722,48); }
    MenuRect pause(int i) const { return rect(205,200+i*78,400,60); }
    int hit(bool titleScreen,float x,float y) const { for(int i=0;i<(titleScreen?5:3);i++) if((titleScreen?title(i):pause(i)).contains(x,y)) return i; return -1; }
};
inline int menuMove(bool title,int selected,int dx,int dy,bool night,bool uphill=false) {
    if (title) {
        if (dy<0) { if(selected==4)return uphill?3:2; if(selected>=2)return night?1:0; return 4; }
        if (dy>0) { if(selected==4)return night?1:0; if(selected<2)return uphill?3:2; return 4; }
        if (dx!=0) { if(selected==4)return uphill?3:2; return selected^1; }
        return selected;
    }
    int dir=dy!=0?dy:dx;
    return (selected + (dir>0?1:dir<0?-1:0)+3)%3;
}
