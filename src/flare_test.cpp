#include "flare.hpp"
#include <cassert>
#include <cstdio>
int main(){std::vector<float> v;std::vector<unsigned> ids;
 appendFlare(v,ids,{0,0,0},{0,0,1},{0,0,10},{1,0,0},{0,1,0},{1,.9f,.6f},1,.6f);
 assert(v.size()==13*7 && ids.size()==36 && v[6]>0);
 v.clear();ids.clear();appendFlare(v,ids,{0,0,0},{0,0,1},{0,0,-10},{1,0,0},{0,1,0},{1,0,0},1,.6f);
 assert(v.empty() && ids.empty());puts("PASS: flare geometry, falloff and viewing direction");}
