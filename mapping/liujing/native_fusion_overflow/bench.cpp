#include "scene.hpp"
#include <chrono>
#include <iostream>
#include <algorithm>
using namespace lj2;
static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(){
 Camera depth{320,200,238.8,238.8,160.5,99.8,.3,5},view;
 std::vector<uint16_t>d(320*200,2000);std::vector<uint8_t>rgb(320*200*3,128);auto points=unproject(d.data(),rgb.data(),depth,identity());Scene s;
 for(int i=0;i<12;i++){double a=now();s.integrate(points,i+1);double b=now();auto t=identity();t[3]=i*.002f;auto prepared=prepare(*s.snapshot(),view,t);double c=now();auto image=render_cpu(prepared);double e=now();size_t valid=std::count_if(image.ids.begin(),image.ids.end(),[](int id){return id>=0;});
  std::cout<<"{\"case\":\"synthetic_wall_not_acceptance\",\"iteration\":"<<i<<",\"map_surfels\":"<<s.snapshot()->count<<",\"visible\":"<<prepared.surfaces.size()<<",\"candidates\":"<<prepared.candidate_pixels<<",\"valid_pixels\":"<<valid<<",\"fusion_ms\":"<<(b-a)*1000<<",\"prepare_ms\":"<<(c-b)*1000<<",\"cpu_render_ms\":"<<(e-c)*1000<<"}\n";
 }
}
