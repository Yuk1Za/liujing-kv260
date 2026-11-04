#include "fusion_store.hpp"
#include "rgbd_pairing.hpp"
#include <cassert>
#include <iostream>
#include <thread>
using namespace lj2;
int main(){
 // Move independently chosen IDs across a voxel boundary. Twelve distinct
 // normals occupy two overflow blocks; erasures shift across block edges.
 std::vector<Point>ring;
 for(int k=0;k<12;k++){float a=float(k)*6.28318530718f/12;ring.push_back({.019f,.005f,1,std::cos(a),std::sin(a),0,uint32_t(k+1)});}
 FusionStore multi(64,{},8,true);multi.enable_voxel_overflow(true);multi.integrate(ring,1);
 assert(multi.snapshot()->count==12&&multi.timing().overflow_surfels==8);
 auto immutable=multi.snapshot();double timestamp=2;
 for(int id:{7,4,10,5,11,8,6,9,0,3,1,2}){
  auto p=ring[id];p.x=.025f;multi.integrate({p},timestamp++);
  assert(multi.snapshot()->count==12);assert(multi.snapshot()->at(id).weight==2);
 }
 for(int repeat=0;repeat<3;repeat++){auto observations=ring;for(auto&p:observations)p.x=.025f;multi.integrate(observations,timestamp++);assert(multi.snapshot()->count==12);}
 for(uint32_t id=0;id<12;id++){assert(multi.snapshot()->at(id).weight==5);assert(immutable->at(id).weight==1);}
 assert(multi.stats().orientation_drops==0&&multi.stats().capacity_drops==0);
#if defined(__aarch64__) && !defined(LJ3_SCALAR)
 // Equal distances must retain encounter order, not the smaller stable ID.
 // Exercise a partial four-query batch and an inline/overflow tie.
 FusionCell tied;tied.n=4;tied.ids={9,8,7,6};
 FusionSurfel left{-.01f,0,1,0,0,1,0,1},right{.01f,0,1,0,0,1,0,1};
 for(int k=0;k<4;k++){auto v=left;if(k)v.nx=1,v.nz=0;tied.set(k,v);}
 tied.extra={1};tied.extra_values.resize(1);tied.extra_values[0].set(0,right);
 std::vector<AssociationQuery>tie_queries(5);for(auto&q:tie_queries)q.value={0,0,1,0,0,1,0,1};
 std::vector<AssociationGroup>tie_groups={{{0,0,0},0,1}};std::vector<const FusionCell*>tie_cells={&tied};uint64_t tested=0;double cpu=0;
 associate_query_lanes(tie_queries,tie_groups,tie_cells,1,tested,cpu);for(auto&q:tie_queries)assert(q.match==9);assert(tested==25);
 left.nz=0;left.nx=1;tied.set(0,left);associate_query_lanes(tie_queries,tie_groups,tie_cells,1,tested,cpu);for(auto&q:tie_queries)assert(q.match==1);
 right.nz=.94f;tied.extra_values[0].set(0,right);associate_query_lanes(tie_queries,tie_groups,tie_cells,1,tested,cpu);for(auto&q:tie_queries)assert(q.match==-1);
 FusionStore candidate_lanes(8192,{},0,true),query_lanes(8192,{},0,true);
 candidate_lanes.enable_voxel_overflow(true);query_lanes.enable_voxel_overflow(true);query_lanes.set_query_simd(true);query_lanes.set_fast_tiles(true);
 for(int frame=1;frame<=12;++frame){
  std::vector<Point>observations;
  for(int i=0;i<3007;++i){auto p=ring[i%12];p.x+=float((i/12)%17)*.019f+(frame%2?.001f:-.001f);p.y+=float(i/204)*.021f;observations.push_back(p);if(i%7==0)observations.push_back(p);}
  candidate_lanes.integrate(observations,frame);query_lanes.integrate(observations,frame);
  auto a=candidate_lanes.snapshot(),b=query_lanes.snapshot();assert(a->count==b->count);
  for(uint32_t id=0;id<a->count;++id)assert(!std::memcmp(&a->at(id),&b->at(id),sizeof(FusionSurfel)));
  assert(candidate_lanes.timing().candidates==query_lanes.timing().candidates);
 }
#endif
 // Overflow reference: six distinct directions in one voxel, migration of
 // inline and overflow owners, immutable snapshots and coalesced ordering.
 std::vector<Point>six={{.019f,.005f,1,1,0,0,1},{.019f,.005f,1,-1,0,0,2},{.019f,.005f,1,0,1,0,3},{.019f,.005f,1,0,-1,0,4},{.019f,.005f,1,0,0,1,5},{.019f,.005f,1,0,0,-1,6}};
 FusionStore limited(32);limited.integrate(six,1);assert(limited.snapshot()->count==4&&limited.stats().orientation_drops==2);
 FusionStore overflow(32,{},8,true),overflow_coalesced(32,{},0,true);overflow.enable_voxel_overflow(true);overflow_coalesced.enable_voxel_overflow(true);overflow_coalesced.set_coalesced_writes(true);
 overflow.integrate(six,1);overflow_coalesced.integrate(six,1);auto overflow_frozen=overflow.snapshot();assert(overflow_frozen->count==6&&overflow.timing().overflow_surfels==2);
 for(int frame=2;frame<30;frame++){
  auto observed=six;for(auto&point:observed)point.x=frame==2?.025f:frame==3?.013f:.028f;
  if(frame>=4&&frame<10)observed.erase(observed.begin(),observed.begin()+4);
  overflow.integrate(observed,frame);overflow_coalesced.integrate(observed,frame);
  auto one=overflow.snapshot(),two=overflow_coalesced.snapshot();assert(one->count==6&&two->count==6);
  for(uint32_t id=0;id<6;id++)assert(!std::memcmp(&one->at(id),&two->at(id),sizeof(FusionSurfel)));
 }
 assert(overflow.stats().orientation_drops==0&&overflow.stats().capacity_drops==0&&overflow.timing().max_cell_occupancy>=6);
 assert(overflow_frozen->at(5).weight==1&&overflow_frozen->at(5).x==.019f);
 FusionStore bounded_overflow(4);bounded_overflow.enable_voxel_overflow(true);bounded_overflow.integrate(six,1);assert(bounded_overflow.stats().capacity_drops==2&&bounded_overflow.stats().orientation_drops==0);
 RgbdPairer<int,int> pairs;auto frame=std::make_shared<int>(1);
 pairs.put_rgb(frame,1,1,0);pairs.put_depth(frame,1.033,1.033,0);assert(!pairs.take(1.04));
 pairs.put_rgb(frame,1.033,1.066,1);pairs.put_depth(frame,1.066,1.066,1);auto matched=pairs.take(1.07);assert(matched&&matched->rgb_id==1&&matched->depth_id==0);
 pairs.put_rgb(frame,1.066,1.1,2);matched=pairs.take(1.1);assert(matched&&matched->depth_id==1);assert(!pairs.take(1.1));
 pairs.put_rgb(frame,1.066,1.1,2);assert(pairs.stamp_rejected==1);
 pairs.put_depth(frame,1.2,1.2,2);pairs.put_rgb(frame,1.2,1.2,3);assert(!pairs.take(1.6));assert(pairs.age_rejected==2);
 for(int i=0;i<20;i++)pairs.put_depth(frame,2+i*.01,2+i*.01,3+i);assert(pairs.capacity_replaced==12);pairs.clear();
 FusionStore s(100);Point p{.999f,.01f,1,0,0,1,0x332211};s.integrate({p,p},1);auto old=s.snapshot();assert(old->count==1);auto oldx=old->at(0).x;
 p.x=1.01f;s.integrate({p,p},2);assert(s.snapshot()->count==1);assert(old->at(0).x==oldx);assert(old->version==1);assert(s.snapshot()->version==2);
 auto d=s.delta(1);assert(!d.full&&d.ids.size()==1&&d.base==1);assert(s.delta(2).ids.empty());assert(s.delta(99).full);
 Point wall=p;wall.nx=1;wall.nz=0;s.integrate({wall},3);assert(s.snapshot()->count==2);
 FusionStore crossing(16);Point edge{-.001f,.01f,1,0,0,1,0x332211};crossing.integrate({edge},1);auto frozen=crossing.snapshot();
 for(int i=2;i<82;i++){edge.x=i%2?.0015f:-.0015f;crossing.integrate({edge,edge,edge},i);assert(crossing.snapshot()->count==1);assert(std::isfinite(crossing.snapshot()->at(0).x));}
 assert(frozen->at(0).x==-.001f);
 // Compare against direct neighborhood lookup while repeatedly migrating,
 // creating and deleting cells, with deliberately tiny cache eviction bounds.
 FusionStore cache_reference(256,{},0),small_cache(256,{},8),warm_cache(256,{},65536),tiled(256,{},0,true),bounded_tiles(256,{},0,true,1),coalesced(256,{},0,true);
 bounded_tiles.set_fast_tiles(true);tiled.set_fast_tiles(true);bounded_tiles.set_cpu_threads(1);coalesced.set_coalesced_writes(true);
 for(int frame=1;frame<=80;frame++){
  std::vector<Point>observed;for(int i=0;i<48;i++){
   Point v{float(i%8)*.019f+(frame%2?.004f:-.004f),float(i/8)*.019f,1,0,0,1,uint32_t(i*357)};
   if(i%5==0){v.nx=1;v.nz=0;}observed.push_back(v);observed.push_back(v);
  }
  observed.push_back({float(frame)*.05f,1,1,0,0,1,0x123456});observed.push_back({-float(frame)*.05f,-1,-1,0,0,1,0x654321});
  cache_reference.integrate(observed,frame);small_cache.integrate(observed,frame);warm_cache.integrate(observed,frame);tiled.integrate(observed,frame);bounded_tiles.integrate(observed,frame);coalesced.integrate(observed,frame);
  auto ref=cache_reference.snapshot();for(auto store:{&small_cache,&warm_cache,&tiled,&bounded_tiles,&coalesced}){auto actual=store->snapshot();assert(ref->count==actual->count);for(uint32_t id=0;id<ref->count;id++)assert(!std::memcmp(&ref->at(id),&actual->at(id),sizeof(FusionSurfel)));}
 }
 // Four occupied orientations block a matched point's intermediate migration.
 // Coalescing writes must preserve those per-observation capacity decisions.
 FusionStore direct_full(8),coalesced_full(8);coalesced_full.set_coalesced_writes(true);
 std::vector<Point>full={{.005f,.005f,1,1,0,0,1},{.005f,.005f,1,-1,0,0,2},{.005f,.005f,1,0,1,0,3},{.005f,.005f,1,0,-1,0,4},{.021f,.005f,1,0,0,1,5}};
 direct_full.integrate(full,1);coalesced_full.integrate(full,1);assert(direct_full.snapshot()->count==5);
 for(int frame=2;frame<8;frame++){
  std::vector<Point>observations(40,Point{.015f,.005f,1,0,0,1,uint32_t(frame*57)});
  if(frame>=4)for(int i=0;i<4;i++)observations.push_back({-.005f,.005f,1,1,0,0,0x123456});
  direct_full.integrate(observations,frame);coalesced_full.integrate(observations,frame);
  auto a=direct_full.snapshot(),b=coalesced_full.snapshot();assert(a->count==b->count);
  for(uint32_t id=0;id<a->count;id++)assert(!std::memcmp(&a->at(id),&b->at(id),sizeof(FusionSurfel)));
 }
 FusionStore capacity(1);capacity.integrate({p},1);p.x=100;capacity.integrate({p},2);assert(capacity.stats().capacity_drops==1);assert(capacity.snapshot()->version==1);
 bool bad=false;try{s.integrate({wall},2);}catch(const std::invalid_argument&){bad=true;}assert(bad);
 std::thread writer([&]{for(int i=4;i<50;i++)s.integrate({wall},i);});for(int i=0;i<1000;i++){auto snap=s.snapshot();assert(snap->count==2);assert(std::isfinite(snap->at(1).x));auto event=s.delta(1);assert(event.snapshot->version>=3);}writer.join();assert(s.delta(1).full);
 std::cout<<"PASS fusion immutable pages, conflicts, migration, cap, ordering, concurrent reads, delta resync\n";
}
