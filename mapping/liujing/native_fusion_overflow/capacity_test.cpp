// Synthetic capacity regression; not live sensor/performance acceptance.
#include "fusion_color_packet.hpp"
#include <cassert>
#include <iostream>
using namespace lj2;
Point point(size_t i){return {float(i%200)*.06f,float((i/200)%200)*.06f,float(i/40000)*.06f,0,0,1,0x332211};}
int main(){
 FusionStore map(2000000,{},0,true);map.enable_voxel_overflow(true);map.set_fast_tiles(true);map.set_cpu_threads(2);
 for(size_t base=0;base<2000000;base+=10000){std::vector<Point> batch;for(size_t i=base;i<base+10000;i++)batch.push_back(point(i));map.integrate(batch,double(base+1));}
 auto snap=map.snapshot();assert(snap->count==2000000);assert(map.stats().capacity_drops==0);
 auto old=snap->version;auto final=map.integrate({point(1999999),point(2000000)},2000001);
 assert(final.merged==1&&final.capacity_drops==1);assert(map.snapshot()->count==2000000);assert(snap->at(1999999).weight==1);
 auto delta=map.delta(old);assert(!delta.full&&delta.ids.size()==1);assert(map.snapshot()->at(delta.ids[0]).weight==2);
 auto packet=fusion_color_packet(map.delta(0),identity(),"synthetic-capacity",2000002,2000003);
 assert(packet.size()>40000000&&packet.size()<40065544);
 bool rejected=false;try{FusionStore bad(2000001);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
 std::cout<<"PASS 2000000 points, boundary drop, existing-point merge, immutable snapshot, delta and full packet bytes="<<packet.size()<<" sizeof_surfel="<<sizeof(FusionSurfel)<<" sizeof_cell="<<sizeof(FusionCell)<<std::endl;
}
