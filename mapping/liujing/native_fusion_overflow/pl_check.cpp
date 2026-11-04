#include "pl.hpp"
#include <iostream>
#include <chrono>
#include <ctime>
#include <cmath>
using namespace lj2;
static Prepared stress_task(Camera c,bool feedback){
 Prepared p;p.camera=c;p.view=identity();p.budget_ok=true;p.offsets.assign(1201,0);const int count=feedback?600:40000;
 for(int i=0;i<count;i++){Surface s{};int x=feedback?320:(i%64)*10,y=feedback?240:((i/64)%48)*10,side=feedback?1:10;s.z=2;s.nx=s.ny=0;s.nz=1;s.x=(x+(side-1)*.5f-c.cx)*s.z/c.fx;s.y=(y+(side-1)*.5f-c.cy)*s.z/c.fy;s.radius=(feedback?1.f:5.f)*s.z/c.fx;s.id=count-i;s.rgb=uint32_t(i*373)&0xffffff;s.x0=x;s.x1=x+side-1;s.y0=y;s.y1=y+side-1;p.surfaces.push_back(s);p.candidate_pixels+=side*side;
  for(int ty=s.y0/16;ty<=s.y1/16;ty++)for(int tx=s.x0/16;tx<=s.x1/16;tx++)++p.offsets[ty*40+tx+1];
 }
 for(int i=1;i<=1200;i++)p.offsets[i]+=p.offsets[i-1];p.indices.resize(p.offsets.back());auto cursor=p.offsets;
 for(uint32_t i=0;i<p.surfaces.size();i++){auto&s=p.surfaces[i];for(int y=s.y0/16;y<=s.y1/16;y++)for(int x=s.x0/16;x<=s.x1/16;x++)p.indices[cursor[y*40+x]++]=i;}
 if(p.indices.size()>100000||p.candidate_pixels>4000000)throw std::runtime_error("stress generator exceeded budget");return p;
}
int main(int argc,char**argv){
 if(argc<2||argc>3||(argc==3&&std::string(argv[2])!="--stress"))return 2;
 try{Pl pl(argv[1]);Camera c;Scene s;std::vector<Point>points;for(int y=-30;y<=30;y++)for(int x=-40;x<=40;x++)points.push_back({x*.025f,y*.025f,2,0,0,1,uint32_t(0x334455+x+40)});
  Scene tilted;std::vector<Point>tilted_points;for(int y=-30;y<=30;y++)for(int x=-40;x<=40;x++){float X=x*.025f,Y=y*.025f;tilted_points.push_back({X,Y,2+.2f*X-.1f*Y,-.2f,.1f,1,uint32_t((x+40)*177+(y+30)*331)});}tilted.integrate(tilted_points,1);
  for(int test=0;test<(argc==3?10:8);test++){if(test==1)s.integrate(points,1);if(test==3){for(auto&p:points){p.z=1.5;p.rgb=0xaa4422;}s.integrate(points,2);}auto t=identity();t[3]=test*.01f;Scene empty;if(test==6||test==7){float angle=(test-5)*.07f;t[0]=t[10]=std::cos(angle);t[2]=std::sin(angle);t[8]=-std::sin(angle);}auto prepared=test>=8?stress_task(c,test==9):prepare(*(test>=6?tilted.snapshot():test==5?empty.snapshot():s.snapshot()),c,t);
   auto start=std::chrono::steady_clock::now();auto cpuclock=std::clock();auto cpu=render_cpu(prepared);double cpu_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count(),cpu_process=1000.*(std::clock()-cpuclock)/CLOCKS_PER_SEC;
   start=std::chrono::steady_clock::now();cpuclock=std::clock();auto hw=pl.render(prepared,true);double pl_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count(),pl_process=1000.*(std::clock()-cpuclock)/CLOCKS_PER_SEC;
   size_t bad=0;for(size_t i=0;i<cpu.rgb.size();i++)if(cpu.rgb[i]!=hw.rgb[i]||cpu.ids[i]!=hw.ids[i]||cpu.depth[i]!=hw.depth[i])++bad;
   std::cout<<"{\"test\":"<<test<<",\"mismatched_pixels\":"<<bad<<",\"cpu_ms\":"<<cpu_ms<<",\"pl_call_ms\":"<<pl_ms<<",\"pl_execute_ms\":"<<pl.last_execute_ms<<",\"cpu_process_ms\":"<<cpu_process<<",\"pl_process_ms\":"<<pl_process<<",\"candidates\":"<<prepared.candidate_pixels<<",\"host_write_bytes\":"<<pl.host_write_bytes<<",\"host_read_bytes\":"<<pl.host_read_bytes<<"}"<<std::endl;
   if(bad)return 1;
   start=std::chrono::steady_clock::now();cpuclock=std::clock();auto cpu_color=render_cpu(prepared,false);cpu_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();cpu_process=1000.*(std::clock()-cpuclock)/CLOCKS_PER_SEC;
   start=std::chrono::steady_clock::now();cpuclock=std::clock();auto hw_color=pl.render(prepared,false);pl_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();pl_process=1000.*(std::clock()-cpuclock)/CLOCKS_PER_SEC;
   if(cpu_color.rgb!=hw_color.rgb)return 1;
   std::cout<<"{\"test\":"<<test<<",\"output_mode\":\"color_only\",\"mismatched_pixels\":0,\"cpu_ms\":"<<cpu_ms<<",\"pl_call_ms\":"<<pl_ms<<",\"pl_kernel_cycles\":"<<pl.metrics[1]<<",\"cpu_process_ms\":"<<cpu_process<<",\"pl_process_ms\":"<<pl_process<<",\"candidates\":"<<prepared.candidate_pixels<<",\"axi_read_bytes\":"<<(uint64_t(pl.metrics[2])*64+uint64_t(pl.metrics[6])*4)<<",\"axi_write_bytes\":"<<uint64_t(pl.metrics[8])*4<<",\"host_write_bytes\":"<<pl.host_write_bytes<<",\"host_read_bytes\":"<<pl.host_read_bytes<<"}"<<std::endl;
   std::cout<<"{\"test\":"<<test<<",\"counter_sample\":true,\"metric_order\":\"signature,cycles,surfel_read_beats,surfel_read_wait,surfel_read_backpressure,surfel_address_wait,index_read_beats,index_read_wait,output_write_beats,output_write_backpressure,output_address_wait,index_address_wait\",\"values\":[";
   for(size_t i=0;i<pl.metrics.size();i++)std::cout<<(i?",":"")<<pl.metrics[i];std::cout<<"]}"<<std::endl;
  }return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
