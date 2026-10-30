#include "sound.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
int main(){SampleLoop s;for(int i=0;i<97;i++)s.data.push_back(std::sin(i*.13f));for(int i=-5000;i<5000;i++){double pos=i*.137;double p=std::fmod(pos,double(s.data.size()));if(p<0)p+=s.data.size();size_t k=size_t(p);float f=float(p-k);float old=s.data[k]+(s.data[(k+1)%s.data.size()]-s.data[k])*f;assert(s.read(pos)==old);}puts("PASS: sample interpolation and wrap match original");}
