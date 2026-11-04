#include "scene.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
#include <chrono>
using namespace lj2;
int main(int argc,char**argv){
 Camera c{320,200,238.80898f,238.80898f,160.47871f,99.8069f,.3f,5.f};
 std::vector<uint16_t>d(c.width*c.height);std::vector<uint8_t>rgb(d.size()*3);for(size_t i=0;i<rgb.size();i++)rgb[i]=uint8_t(i*37);
 auto pose=identity();pose[0]=pose[5]=std::cos(.3f);pose[1]=-std::sin(.3f);pose[4]=std::sin(.3f);pose[3]=.1234f;pose[7]=-.12f;
 auto now=[](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();};
 uint64_t frames=0,points=0;double old_ms=0,cached_ms=0;
 auto check=[&]{auto begin=now();auto a=unproject(d.data(),rgb.data(),c,pose);auto middle=now();auto b=unproject_cached(d.data(),rgb.data(),c,pose);auto end=now();old_ms+=middle-begin;cached_ms+=end-middle;if(a.size()!=b.size()||std::memcmp(a.data(),b.data(),a.size()*sizeof(Point)))throw std::runtime_error("cached stencil changed a point or its order");++frames;points+=a.size();};
 for(int f=0;f<30;f++){for(int y=0;y<c.height;y++)for(int x=0;x<c.width;x++){auto i=y*c.width+x;d[i]=uint16_t(1000+(x*7+y*11+f*3)%3000);if((i+f)%31==0)d[i]=0;if((i+f)%37==0)d[i]=299;if((i+f)%41==0)d[i]=5001;}check();}
 if(argc>1)for(auto&file:std::filesystem::directory_iterator(argv[1]))if(file.path().extension()==".depth"){
  std::ifstream in(file.path(),std::ios::binary);std::vector<uint16_t>raw(640*400);in.read(reinterpret_cast<char*>(raw.data()),raw.size()*2);if(!in)throw std::runtime_error("raw depth size");
  for(int y=0;y<200;y++)for(int x=0;x<320;x++)d[y*320+x]=raw[(y*2)*640+x*2];check();
 }
 std::cout<<"{\"frames\":"<<frames<<",\"points_compared\":"<<points<<",\"bit_mismatches\":0,\"reference_ms\":"<<old_ms<<",\"cached_ms\":"<<cached_ms<<",\"scope\":\"identical 320x200 stencil inputs, including edges and holes; no acquisition-spec change\"}\n";
}
