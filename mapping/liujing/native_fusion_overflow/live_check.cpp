#include "pl.hpp"
#include "live_policy.hpp"
#include "encode.hpp"
#include <iostream>
using namespace lj2;
int main(int argc,char**argv){
 if(argc!=2)return 2;
 try{
  Pl pl(argv[1]);Scene wall,tilted,empty;std::vector<Point>points;
  for(int y=-35;y<=35;++y)for(int x=-50;x<=50;++x)points.push_back({x*.025f,y*.025f,2,0,0,1,uint32_t((x+50)*273+(y+35)*93)});
  wall.integrate(points,1);for(auto&p:points){p.z=2+.2f*p.x-.1f*p.y;p.nx=-.2f;p.ny=.1f;}tilted.integrate(points,1);
  for(int tier=1;tier<4;++tier)for(int test=0;test<8;++test){
   Camera c=resized(Camera{},ResolutionPolicy::widths[tier],ResolutionPolicy::heights[tier]);auto pose=identity();pose[3]=test*.025f;
   if(test>=4){float a=.08f*(test-3);pose[0]=pose[10]=std::cos(a);pose[2]=std::sin(a);pose[8]=-std::sin(a);}
   auto task=prepare(*(test==0?empty.snapshot():test<4?wall.snapshot():tilted.snapshot()),c,pose);
   for(bool debug:{true,false}){
    auto cpu=render_cpu(task,debug),hw=pl.render(task,debug);size_t bad=0;
    for(size_t i=0;i<cpu.rgb.size();++i)if(cpu.rgb[i]!=hw.rgb[i]||(debug&&(cpu.depth[i]!=hw.depth[i]||cpu.ids[i]!=hw.ids[i])))++bad;
    std::deque<std::array<float,3>>path{{0,0,1},{.1f,.1f,1}};auto encoded=encode_image(hw,c,pose,path,true);
    if(encoded.bgr.cols!=c.width||encoded.bgr.rows!=c.height||encoded.bytes.empty())throw std::runtime_error("encode dimensions");
    std::cout<<"{\"width\":"<<c.width<<",\"height\":"<<c.height<<",\"test\":"<<test<<",\"debug\":"<<(debug?"true":"false")<<",\"mismatched_pixels\":"<<bad<<",\"candidates\":"<<task.candidate_pixels<<",\"pl_execute_ms\":"<<pl.last_execute_ms<<"}"<<std::endl;
    if(bad)return 1;
   }
  }return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
