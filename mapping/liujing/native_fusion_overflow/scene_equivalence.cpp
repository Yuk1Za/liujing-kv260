#include "scene.hpp"
#include "live_policy.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
using namespace lj2;
static uint64_t hash=1469598103934665603ULL;
template<class T>void digest(T value){const auto*p=reinterpret_cast<const unsigned char*>(&value);for(size_t i=0;i<sizeof(T);++i){hash^=p[i];hash*=1099511628211ULL;}}
static void surf(const Surface&s){for(float f:{s.x,s.y,s.z,s.nx,s.ny,s.nz,s.radius,s.weight})digest(f);digest(s.rgb);digest(s.id);}
int main(int argc,char**){
 Scene scene;auto start=std::chrono::steady_clock::now();
 for(int frame=0;frame<20;++frame){
  std::vector<Point>points;float move=.002f*std::sin(frame*.3f);
  for(int y=-45;y<45;y++)for(int x=-70;x<70;x++){
   points.push_back({x*.025f+move,y*.025f,2+.003f*std::sin(x+y+frame),0,0,1,uint32_t((x+70)*911+(y+45)*29)});
   if(x>35)points.push_back({1.001f+move,y*.025f,(x-35)*.025f+1,1,0,0,0xabcdef});
  }
  scene.integrate(points,frame+1);auto snap=scene.snapshot();digest(snap->count);digest(snap->version);
  for(const auto&b:snap->blocks)for(const auto&level:b.second->lod)for(const auto&s:level)surf(s);
  if(argc==1)for(int tier=1;tier<4;++tier){auto view=identity();view[3]=frame*.001f;auto task=prepare(*snap,resized(Camera{},ResolutionPolicy::widths[tier],ResolutionPolicy::heights[tier]),view);digest(task.budget_ok);digest(task.candidate_pixels);for(const auto&s:task.surfaces)surf(s);for(auto i:task.offsets)digest(i);for(auto i:task.indices)digest(i);}
  std::cout<<frame<<" "<<snap->count<<" "<<hash<<"\n";
 }
 std::cerr<<"elapsed_ms "<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<"\n";
}
