#pragma once
#include "scene.hpp"
#include "fusion_association.hpp"
#include <unordered_map>
#include <deque>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <thread>
#include <ctime>
#ifdef __aarch64__
#include <arm_neon.h>
#endif

namespace lj2 {
inline double thread_cpu_millis(){timespec t{};if(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t))throw std::runtime_error("thread clock");return t.tv_sec*1000.+t.tv_nsec*1e-6;}
// Pages belong to an immutable committed version. Only the writer owns cells_.
constexpr size_t fusion_page_size=128;
struct FusionPage {std::array<FusionSurfel,fusion_page_size>surfaces{};std::array<uint64_t,fusion_page_size>changed{};uint64_t latest=0;FusionSurfel&operator[](size_t i){return surfaces[i];}const FusionSurfel&operator[](size_t i)const{return surfaces[i];}};
struct FusionSnapshot:Snapshot {
 std::vector<std::shared_ptr<const FusionPage>> pages;
 uint64_t rgb_id=0,depth_id=0;double committed_mono=0,received_mono=0;
 const FusionSurfel&at(uint32_t id)const{return (*pages.at(id/fusion_page_size))[id%fusion_page_size];}
};
struct FusionHash {size_t operator()(const Key&k)const{uint64_t h=uint32_t(k.x)*0x9e3779b185ebca87ULL;h^=uint32_t(k.y)*0xc2b2ae3d27d4eb4fULL;h^=uint32_t(k.z)*0x165667b19e3779f9ULL;return size_t(h^(h>>32));}};
struct FusionExtraBlock {alignas(16) float x[4]{},y[4]{},z[4]{},nx[4]{},ny[4]{},nz[4]{};
 void set(size_t k,const FusionSurfel&s){x[k]=s.x;y[k]=s.y;z[k]=s.z;nx[k]=s.nx;ny[k]=s.ny;nz[k]=s.nz;}
 void copy(size_t k,const FusionExtraBlock&s,size_t j){x[k]=s.x[j];y[k]=s.y[j];z[k]=s.z[j];nx[k]=s.nx[j];ny[k]=s.ny[j];nz[k]=s.nz[j];}
};
struct FusionCell {std::vector<uint32_t>extra;std::vector<FusionExtraBlock>extra_values;size_t size()const{return size_t(n)+extra.size();}std::array<uint32_t,4>ids{};uint8_t n=0;uint64_t dirty_epoch=0;alignas(16) float x[4]{},y[4]{},z[4]{},nx[4]{},ny[4]{},nz[4]{};
 void set(int k,const FusionSurfel&s){x[k]=s.x;y[k]=s.y;z[k]=s.z;nx[k]=s.nx;ny[k]=s.ny;nz[k]=s.nz;}
 void set_id(uint32_t id,const FusionSurfel&s){for(int k=0;k<n;k++)if(ids[k]==id){set(k,s);return;}for(size_t k=0;k<extra.size();k++)if(extra[k]==id){extra_values[k/4].set(k%4,s);return;}throw std::runtime_error("missing candidate cache ID");}
 void erase_extra(size_t k){for(size_t j=k;j+1<extra.size();j++)extra_values[j/4].copy(j%4,extra_values[(j+1)/4],(j+1)%4);extra.erase(extra.begin()+k);extra_values.resize((extra.size()+3)/4);}
 void erase(int k){for(int j=k+1;j<n;j++){ids[j-1]=ids[j];x[j-1]=x[j];y[j-1]=y[j];z[j-1]=z[j];nx[j-1]=nx[j];ny[j-1]=ny[j];nz[j-1]=nz[j];}--n;}
};
#include "fusion_query_simd.hpp"
struct FusionTiming {double grouping_ms=0,association_ms=0,commit_ms=0,total_ms=0;uint64_t queries=0,candidates=0,groups=0;double grouping_cpu_ms=0,association_cpu_ms=0,commit_cpu_ms=0;double order_ms=0,matched_ms=0,unmatched_ms=0,cache_update_ms=0,backend_sync_ms=0;uint64_t unmatched_queries=0,query_cache_hits=0,query_cache_rebuilds=0,query_cache_invalidations=0,index_tiles=0;bool index_overflow=false;uint64_t overflow_surfels=0,max_cell_occupancy=0;};
struct FusionDelta {std::shared_ptr<const FusionSnapshot>snapshot;uint64_t base=0;bool full=true;std::vector<uint32_t>ids;};
class FusionStore {
 std::unordered_map<Key,FusionCell,FusionHash>cells_;
 bool query_simd_=false;bool voxel_overflow_=false;uint64_t overflow_surfels_=0,max_cell_occupancy_=0;
 void append(FusionCell&c,uint32_t id){if(c.n<4)c.ids[c.n++]=id;else{if(c.extra.size()%4==0)c.extra_values.emplace_back();c.extra.push_back(id);++overflow_surfels_;}max_cell_occupancy_=std::max(max_cell_occupancy_,uint64_t(c.size()));}
 void erase_id(FusionCell&c,uint32_t id){for(int k=0;k<c.n;k++)if(c.ids[k]==id){c.erase(k);return;}auto it=std::find(c.extra.begin(),c.extra.end(),id);if(it==c.extra.end())throw std::runtime_error("missing cell owner");c.erase_extra(size_t(it-c.extra.begin()));--overflow_surfels_;}
 struct CellTile {std::array<FusionCell*,64>cells{};uint32_t count=0;};
 std::unordered_map<Key,CellTile,FusionHash>cell_tiles_;
 struct TileLookup {Key key{};const CellTile*tile=nullptr;uint64_t epoch=0;};
 mutable std::array<TileLookup,256>tile_lookup_{};uint64_t tile_epoch_=1;bool fast_tiles_=false;
 // Writer-only lookup cache. Association threads read the already-built cell
 // list. Rehash preserves node pointers; creation/deletion invalidates misses
 // and removed pointers before the next lookup.
 const CellTile*find_tile(Key key)const{
  if(!fast_tiles_){auto it=cell_tiles_.find(key);return it==cell_tiles_.end()?nullptr:&it->second;}
  auto&cached=tile_lookup_[FusionHash{}(key)&255];
  if(cached.epoch==tile_epoch_&&cached.key==key)return cached.tile;
  auto it=cell_tiles_.find(key);cached={key,it==cell_tiles_.end()?nullptr:&it->second,tile_epoch_};return cached.tile;
 }
 bool tiled_index_=false,tile_overflow_=false,coalesced_writes_=false;size_t tile_capacity_=65536;int cpu_threads_=2;
 static int tile_axis(int x){return x>=0?x/4:-int((uint32_t(-int64_t(x))+3)/4);}
 static Key tile_key(Key k){return {tile_axis(k.x),tile_axis(k.y),tile_axis(k.z)};}
 static size_t tile_slot(Key k,Key t){return (k.z-t.z*4)*16+(k.y-t.y*4)*4+(k.x-t.x*4);}
 FusionCell&ensure_cell(Key k){
  auto [it,added]=cells_.try_emplace(k);
  if(added&&tiled_index_&&!backend_){auto t=tile_key(k);auto tile=cell_tiles_.find(t);
   if(tile==cell_tiles_.end()&&cell_tiles_.size()<tile_capacity_){tile=cell_tiles_.try_emplace(t).first;++tile_epoch_;}
   if(tile==cell_tiles_.end())tile_overflow_=true;
   else{tile->second.cells[tile_slot(k,t)]=&it->second;++tile->second.count;}
  }return it->second;
 }
 template<class Output>void neighborhood(Key k,Output&out)const{
  if(!tiled_index_||backend_){for(int z=-1;z<=1;z++)for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++){auto it=cells_.find({k.x+x,k.y+y,k.z+z});if(it!=cells_.end()&&it->second.size())out.push_back(&it->second);}return;}
  auto lo=tile_key({k.x-1,k.y-1,k.z-1}),hi=tile_key({k.x+1,k.y+1,k.z+1});
  const CellTile*tiles[8]{};
  for(int z=lo.z;z<=hi.z;z++)for(int y=lo.y;y<=hi.y;y++)for(int x=lo.x;x<=hi.x;x++){
   tiles[(z-lo.z)*4+(y-lo.y)*2+x-lo.x]=find_tile({x,y,z});
  }
  if(fast_tiles_){
   int tx[3],ty[3],tz[3],sx[3],sy[3],sz[3];
   for(int i=0;i<3;++i){int x=k.x+i-1,y=k.y+i-1,z=k.z+i-1;int ax=tile_axis(x),ay=tile_axis(y),az=tile_axis(z);tx[i]=ax-lo.x;ty[i]=(ay-lo.y)*2;tz[i]=(az-lo.z)*4;sx[i]=x-ax*4;sy[i]=(y-ay*4)*4;sz[i]=(z-az*4)*16;}
   for(int z=0;z<3;++z)for(int y=0;y<3;++y)for(int x=0;x<3;++x){
    auto tile=tiles[tz[z]+ty[y]+tx[x]];const FusionCell*c=tile?tile->cells[sz[z]+sy[y]+sx[x]]:nullptr;
    if(!c&&tile_overflow_){auto it=cells_.find({k.x+x-1,k.y+y-1,k.z+z-1});if(it!=cells_.end())c=&it->second;}
    if(c&&c->size())out.push_back(c);
   }return;
  }
  for(int z=-1;z<=1;z++)for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++){
   Key q{k.x+x,k.y+y,k.z+z},t=tile_key(q);auto tile=tiles[(t.z-lo.z)*4+(t.y-lo.y)*2+t.x-lo.x];
   const FusionCell*c=tile?tile->cells[tile_slot(q,t)]:nullptr;
   // A bounded auxiliary index may omit cells after capacity exhaustion.
   // Direct lookup preserves exact behavior without dropping observations.
   if(!c&&tile_overflow_){auto it=cells_.find(q);if(it!=cells_.end())c=&it->second;}
   if(c&&c->size())out.push_back(c);
  }
 }
 struct NeighborCache {bool valid=false;std::vector<const FusionCell*>cells;};
 std::unordered_map<Key,NeighborCache,FusionHash>query_cache_;std::deque<Key>query_cache_order_;size_t query_cache_capacity_;
 // Stable unordered-map node addresses let ID-based commits bypass hashing.
 std::vector<FusionCell*>owners_;std::vector<Key>locations_;
 std::shared_ptr<AssociationBackend>backend_;
 std::shared_ptr<const FusionSnapshot>current_=std::make_shared<const FusionSnapshot>();
 mutable std::mutex mutex_;size_t capacity_;MapStats stats_;FusionTiming timing_;
 static double now(){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();}
 static Key cell(float x,float y,float z){return {int(std::floor(x/.02f)),int(std::floor(y/.02f)),int(std::floor(z/.02f))};}
 static uint32_t color(uint32_t a,uint32_t b,float w){uint32_t c=0;for(int k=0;k<3;k++)c|=uint32_t((float((a>>(k*8))&255)*w+float((b>>(k*8))&255))/(w+1)+.5f)<<(k*8);return c;}
 static FusionSurfel mixed(FusionSurfel a,const FusionSurfel&b){float w=std::min(a.weight,31.f);a.x=(a.x*w+b.x)/(w+1);a.y=(a.y*w+b.y)/(w+1);a.z=(a.z*w+b.z)/(w+1);a.rgb=color(a.rgb,b.rgb,w);float x=a.nx*w+b.nx,y=a.ny*w+b.ny,z=a.nz*w+b.nz,n=std::sqrt(x*x+y*y+z*z);a.nx=x/n;a.ny=y/n;a.nz=z/n;a.weight=std::min(32.f,w+1);return a;}
 static bool valid(const Point&p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::max({std::fabs(p.x),std::fabs(p.y),std::fabs(p.z)})<=10000&&std::isfinite(p.nx*p.nx+p.ny*p.ny+p.nz*p.nz)&&(p.nx*p.nx+p.ny*p.ny+p.nz*p.nz)>=.25f;}
 static FusionSurfel incoming(const Point&p){FusionSurfel s{};float n=std::sqrt((p.nx*p.nx+p.ny*p.ny)+p.nz*p.nz);s.x=p.x;s.y=p.y;s.z=p.z;s.nx=p.nx/n;s.ny=p.ny/n;s.nz=p.nz/n;s.rgb=p.rgb&0xffffff;s.weight=1;return s;}
 static bool better(const FusionSurfel&p,const FusionSurfel&s,float&distance){float dx=p.x-s.x,dy=p.y-s.y,dz=p.z-s.z,d=(dx*dx+dy*dy)+dz*dz;if(d<distance&&((s.nx*p.nx+s.ny*p.ny)+s.nz*p.nz)>.94f&&std::fabs((dx*s.nx+dy*s.ny)+dz*s.nz)<.012f){distance=d;return true;}return false;}
public:
 void set_fast_tiles(bool enabled){if(snapshot()->version||!tiled_index_)throw std::invalid_argument("fast tiles require tiled index before commit");fast_tiles_=enabled;}
 explicit FusionStore(size_t cap=1000000,std::shared_ptr<AssociationBackend> backend={},size_t query_cache_capacity=0,bool tiled_index=false,size_t tile_capacity=65536):tiled_index_(tiled_index),tile_capacity_(tile_capacity),query_cache_capacity_(query_cache_capacity),backend_(std::move(backend)),capacity_(cap){if(!cap||cap>2000000||(backend_&&cap>1000000)||query_cache_capacity>65536||tile_capacity>65536)throw std::invalid_argument("fusion capacity");cells_.reserve(65536);owners_.reserve(cap);locations_.reserve(cap);if(!backend_)query_cache_.reserve(query_cache_capacity);if(tiled_index_)cell_tiles_.reserve(std::min(size_t(4096),tile_capacity));}
 void set_query_simd(bool enabled){if(snapshot()->version||(enabled&&backend_))throw std::invalid_argument("query SIMD is CPU only, before commit");
#if !defined(__aarch64__) || defined(LJ3_SCALAR)
 if(enabled)throw std::invalid_argument("query SIMD requires AArch64 NEON");
#endif
 query_simd_=enabled;}
 void enable_voxel_overflow(bool enabled){if(snapshot()->version||(enabled&&backend_))throw std::invalid_argument("overflow is CPU reference only, before first commit");voxel_overflow_=enabled;}
 size_t capacity()const{return capacity_;}
 void set_backend(std::shared_ptr<AssociationBackend>backend){if(backend&&capacity_>1000000)throw std::invalid_argument("PL association capacity remains 1000000");if((voxel_overflow_||query_simd_)&&backend)throw std::invalid_argument("current PL ABI cannot represent overflow");if(snapshot()->version)throw std::runtime_error("backend change after commit");backend_=std::move(backend);}
 void set_cpu_threads(int n){if(snapshot()->version||n<1||n>2)throw std::invalid_argument("association CPU threads must be 1 or 2, before first commit");cpu_threads_=n;}
 void set_coalesced_writes(bool enabled){if(snapshot()->version)throw std::invalid_argument("commit mode change after commit");coalesced_writes_=enabled;}
 std::shared_ptr<const FusionSnapshot>snapshot()const{return std::atomic_load(&current_);}
 MapStats stats()const{std::lock_guard<std::mutex>l(mutex_);return stats_;}
 FusionTiming timing()const{std::lock_guard<std::mutex>l(mutex_);return timing_;}
 // Association reads one frozen version; conflicts commit in ID/input order.
 MapStats integrate(const std::vector<Point>&points,double stamp,uint64_t rgb=0,uint64_t depth=0,double received=0){
  auto start=now(),cpu_start=thread_cpu_millis();auto old=snapshot();if(!std::isfinite(stamp)||(old->version&&stamp<=old->source_stamp))throw std::invalid_argument("fusion timestamp regression");
  using Query=AssociationQuery;using Group=AssociationGroup;
  std::vector<Query>q;std::vector<Group>groups;std::vector<const FusionCell*>candidate_cells;std::unordered_map<Key,uint32_t,FusionHash>group_index;q.reserve(points.size());groups.reserve(points.size()/2);group_index.reserve(points.size()/2);candidate_cells.reserve(points.size()*2);
  Key previous{};uint32_t previous_group=0;bool have=false;
  for(const auto&p:points){if(!valid(p))continue;FusionSurfel v=incoming(p);auto k=cell(v.x,v.y,v.z);uint32_t gi;
   if(have&&k==previous)gi=previous_group;else{auto [it,added]=group_index.emplace(k,groups.size());if(added)groups.push_back({k,0,0});gi=it->second;previous=k;previous_group=gi;have=true;}q.push_back({v,-1,gi,-1});}
  uint64_t cache_hits=0,cache_rebuilds=0,cache_invalidations=0;
  if(!backend_)for(auto&g:groups){g.offset=candidate_cells.size();
   auto fill=[&](auto&out){neighborhood(g.key,out);};
   if(!query_cache_capacity_)fill(candidate_cells);
   else{
    auto it=query_cache_.find(g.key);if(it==query_cache_.end()){
     if(query_cache_.size()==query_cache_capacity_){query_cache_.erase(query_cache_order_.front());query_cache_order_.pop_front();}
     it=query_cache_.emplace(g.key,NeighborCache{}).first;query_cache_order_.push_back(g.key);
    }
    auto&cached=it->second;if(!cached.valid){cached.cells.clear();fill(cached.cells);cached.valid=true;++cache_rebuilds;}else ++cache_hits;
    candidate_cells.insert(candidate_cells.end(),cached.cells.begin(),cached.cells.end());
   }g.count=candidate_cells.size()-g.offset;
  }
  auto grouped=now(),grouped_cpu=thread_cpu_millis();uint64_t candidates=0;double association_cpu=0;
  if(backend_){backend_->associate(q,groups,old->version);association_cpu=thread_cpu_millis()-grouped_cpu;}else if(query_simd_){
#if defined(__aarch64__) && !defined(LJ3_SCALAR)
 associate_query_lanes(q,groups,candidate_cells,cpu_threads_,candidates,association_cpu);
#endif
 }else{
  #pragma omp parallel num_threads(cpu_threads_) reduction(+:candidates,association_cpu) if(q.size()>1000&&cpu_threads_>1)
  {auto worker_cpu_start=thread_cpu_millis();
  #pragma omp for schedule(static)
  for(size_t qi=0;qi<q.size();qi++){const auto&g=groups[q[qi].group];const auto&p=q[qi].value;float distance=.025f*.025f;for(uint32_t j=0;j<g.count;j++)candidates+=candidate_cells[g.offset+j]->size();
#if defined(__aarch64__) && !defined(LJ3_SCALAR)
   const auto px=vdupq_n_f32(p.x),py=vdupq_n_f32(p.y),pz=vdupq_n_f32(p.z),pnx=vdupq_n_f32(p.nx),pny=vdupq_n_f32(p.ny),pnz=vdupq_n_f32(p.nz);
   auto compare4=[&](const auto&b,const uint32_t*ids,size_t n){auto dx=vsubq_f32(px,vld1q_f32(b.x)),dy=vsubq_f32(py,vld1q_f32(b.y)),dz=vsubq_f32(pz,vld1q_f32(b.z));auto nx=vld1q_f32(b.nx),ny=vld1q_f32(b.ny),nz=vld1q_f32(b.nz);auto d=vaddq_f32(vaddq_f32(vmulq_f32(dx,dx),vmulq_f32(dy,dy)),vmulq_f32(dz,dz));auto dot=vaddq_f32(vaddq_f32(vmulq_f32(nx,pnx),vmulq_f32(ny,pny)),vmulq_f32(nz,pnz));auto plane=vabsq_f32(vaddq_f32(vaddq_f32(vmulq_f32(dx,nx),vmulq_f32(dy,ny)),vmulq_f32(dz,nz)));auto ok=vandq_u32(vandq_u32(vcltq_f32(d,vdupq_n_f32(distance)),vcgtq_f32(dot,vdupq_n_f32(.94f))),vcltq_f32(plane,vdupq_n_f32(.012f)));float ds[4];uint32_t good[4];vst1q_f32(ds,d);vst1q_u32(good,ok);for(size_t k=0;k<n;k++)if(good[k]&&ds[k]<distance){distance=ds[k];q[qi].match=ids[k];}};
   for(uint32_t j=0;j<g.count;j++){const auto&b=*candidate_cells[g.offset+j];compare4(b,b.ids.data(),b.n);for(size_t k=0;k<b.extra_values.size();k++)compare4(b.extra_values[k],b.extra.data()+k*4,std::min(size_t(4),b.extra.size()-k*4));}
#else
   for(uint32_t j=0;j<g.count;j++){const auto&b=*candidate_cells[g.offset+j];for(int k=0;k<b.n;k++){auto id=b.ids[k];if(better(p,old->at(id),distance))q[qi].match=int32_t(id);}for(auto id:b.extra)if(better(p,old->at(id),distance))q[qi].match=int32_t(id);}
#endif
  }

  association_cpu+=thread_cpu_millis()-worker_cpu_start;
  }
  }
  for(const auto&p:q)if(p.match< -1||(p.match>=0&&size_t(p.match)>=old->count))throw std::runtime_error("association returned invalid surface ID");
  auto associated=now(),associated_cpu=thread_cpu_millis();
  std::vector<std::pair<Key,FusionCell*>>dirty_cells;std::vector<Key>empty_cells,occupancy_changes;
  auto occupancy_changed=[&](FusionCell&c,Key k){if(!backend_&&query_cache_capacity_&&c.dirty_epoch!=old->version+1){c.dirty_epoch=old->version+1;occupancy_changes.push_back(k);}};
  auto dirty=[&](FusionCell&c,Key k){if(backend_&&c.dirty_epoch!=old->version+1){c.dirty_epoch=old->version+1;dirty_cells.push_back({k,&c});}};auto next=std::make_shared<FusionSnapshot>(*old);std::vector<FusionPage*>writable((capacity_+fusion_page_size-1)/fusion_page_size,nullptr);size_t dirty_count=0;
  auto edit=[&](uint32_t id)->FusionSurfel&{size_t page=id/fusion_page_size;if(!writable[page]){auto p=page<next->pages.size()?std::make_shared<FusionPage>(*next->pages[page]):std::make_shared<FusionPage>();if(page==next->pages.size())next->pages.push_back(p);else next->pages[page]=p;writable[page]=p.get();dirty_count++;}writable[page]->changed[id%fusion_page_size]=old->version+1;writable[page]->latest=old->version+1;return (*writable[page])[id%fusion_page_size];};
  std::vector<int32_t>heads(old->count+1,-1),tails(old->count+1,-1);for(uint32_t i=0;i<q.size();i++){size_t id=q[i].match<0?old->count:size_t(q[i].match);if(tails[id]>=0)q[tails[id]].next=i;else heads[id]=i;tails[id]=i;}
  std::vector<uint32_t>order;order.reserve(q.size());for(auto h:heads)for(int32_t i=h;i>=0;i=q[i].next)order.push_back(i);
  MapStats changes;std::vector<uint32_t>touched;touched.reserve(q.size());std::vector<uint8_t>seen(capacity_,0);auto touch=[&](uint32_t id){if(!seen[id]){seen[id]=1;touched.push_back(id);}};
  auto ordered_ready=now(),first_unmatched=ordered_ready;bool saw_unmatched=false;uint64_t unmatched_queries=0;
  int32_t pending_id=-1;FusionSurfel pending_value{};
  // Matched observations are already contiguous by ID and in input order.
  // Keep their value locally, while still applying every index migration and
  // capacity decision in order. Flush before unmatched observations can read it.
  auto flush_pending=[&]{if(pending_id>=0){edit(pending_id)=pending_value;touch(pending_id);pending_id=-1;}};
  for(auto qi:order){auto&p=q[qi];int32_t id=p.match;
   const bool coalesce=coalesced_writes_&&p.match>=0;
   if(!coalesce)flush_pending();
   else if(pending_id!=id){flush_pending();pending_id=id;pending_value=next->at(id);}
   // Unmatched points also search newly committed points, preventing duplicates.
   if(id<0){if(!saw_unmatched){first_unmatched=now();saw_unmatched=true;}++unmatched_queries;float distance=.025f*.025f;auto k=cell(p.value.x,p.value.y,p.value.z);struct LocalCells{const FusionCell*data[27]{};size_t count=0;void push_back(const FusionCell*c){data[count++]=c;}}nearby;neighborhood(k,nearby);for(size_t ci=0;ci<nearby.count;ci++){auto&c=*nearby.data[ci];for(int j=0;j<c.n;j++){auto candidate=c.ids[j];if(better(p.value,next->at(candidate),distance))id=candidate;}for(auto candidate:c.extra)if(better(p.value,next->at(candidate),distance))id=candidate;}}
   if(id>=0){auto before=coalesce?pending_value:next->at(id);auto updated=mixed(before,p.value);Key a=locations_.at(id),b=cell(updated.x,updated.y,updated.z);if(!(a==b)){auto&dest=ensure_cell(b);if(dest.n<4||voxel_overflow_){if(!dest.size())occupancy_changed(dest,b);append(dest,id);auto&src=*owners_.at(id);dirty(src,a);erase_id(src,uint32_t(id));if(!src.size()){occupancy_changed(src,a);empty_cells.push_back(a);}owners_[id]=&dest;locations_[id]=b;}else{updated.x=before.x;updated.y=before.y;updated.z=before.z;updated.nx=before.nx;updated.ny=before.ny;updated.nz=before.nz;}}if(coalesce)pending_value=updated;else{edit(id)=updated;touch(id);}changes.merged++;
   }else{if(next->count>=capacity_){changes.capacity_drops++;continue;}auto k=cell(p.value.x,p.value.y,p.value.z);auto&c=ensure_cell(k);if(c.n==4&&!voxel_overflow_){changes.orientation_drops++;continue;}if(!c.size())occupancy_changed(c,k);auto added=uint32_t(next->count++);edit(added)=p.value;append(c,added);owners_.push_back(&c);locations_.push_back(k);touch(added);changes.added++;}
  }
  flush_pending();auto integrated=now();if(!saw_unmatched)first_unmatched=integrated;
  std::sort(touched.begin(),touched.end());
  for(auto id:touched){const auto&s=next->at(id);auto key=locations_.at(id);auto&c=*owners_.at(id);dirty(c,key);if(!backend_)c.set_id(id,s);}

  // Invalidate every affected query neighborhood before any empty node is
  // erased. Valid entries may then safely keep stable unordered-map pointers.
  for(auto k:occupancy_changes)for(int z=-1;z<=1;z++)for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++){
   auto it=query_cache_.find({k.x+x,k.y+y,k.z+z});if(it!=query_cache_.end()&&it->second.valid){it->second.valid=false;++cache_invalidations;}
  }
  auto cache_ready=now();
  if(backend_&&!touched.empty()){
   std::vector<AssociationUpdate>updates;updates.reserve(dirty_cells.size());
   for(auto&entry:dirty_cells){AssociationUpdate u{};u.key=entry.first;u.cell.word[0]=uint32_t(u.key.x);u.cell.word[1]=uint32_t(u.key.y);u.cell.word[2]=uint32_t(u.key.z);
    if(entry.second->n){const auto&c=*entry.second;u.cell.word[3]=c.n;for(int j=0;j<c.n;j++){const auto&s=next->at(c.ids[j]);float values[6]={s.x,s.y,s.z,s.nx,s.ny,s.nz};std::memcpy(u.cell.word+4+j*7,values,24);u.cell.word[10+j*7]=c.ids[j];}}
    updates.push_back(u);
   }backend_->sync(updates,old->version+1);
  }
  for(auto key:empty_cells){auto it=cells_.find(key);if(it!=cells_.end()&&!it->second.size()){if(tiled_index_&&!backend_){auto t=tile_key(key);auto tile=cell_tiles_.find(t);if(tile!=cell_tiles_.end()&&tile->second.cells[tile_slot(key,t)]){tile->second.cells[tile_slot(key,t)]=nullptr;if(!--tile->second.count){cell_tiles_.erase(tile);++tile_epoch_;}}}cells_.erase(it);}}
  auto synchronized=now();changes.cloned_blocks=dirty_count;changes.index_ms=grouped-start;changes.integrate_ms=now()-grouped;changes.lod_ms=0;
  if(!touched.empty()){next->version++;next->source_stamp=stamp;next->rgb_id=rgb;next->depth_id=depth;next->received_mono=received;next->committed_mono=now()/1000.;}
  {std::lock_guard<std::mutex>l(mutex_);if(!touched.empty()){std::atomic_store(&current_,std::shared_ptr<const FusionSnapshot>(next));}stats_.added+=changes.added;stats_.merged+=changes.merged;stats_.capacity_drops+=changes.capacity_drops;stats_.orientation_drops+=changes.orientation_drops;stats_.cloned_blocks+=changes.cloned_blocks;timing_={grouped-start,associated-grouped,now()-associated,now()-start,q.size(),candidates,groups.size(),grouped_cpu-cpu_start,association_cpu,thread_cpu_millis()-associated_cpu};timing_.order_ms=ordered_ready-associated;timing_.matched_ms=first_unmatched-ordered_ready;timing_.unmatched_ms=integrated-first_unmatched;timing_.cache_update_ms=cache_ready-integrated;timing_.backend_sync_ms=synchronized-cache_ready;timing_.unmatched_queries=unmatched_queries;timing_.query_cache_hits=cache_hits;timing_.query_cache_rebuilds=cache_rebuilds;timing_.query_cache_invalidations=cache_invalidations;timing_.index_tiles=cell_tiles_.size();timing_.index_overflow=tile_overflow_;timing_.overflow_surfels=overflow_surfels_;timing_.max_cell_occupancy=max_cell_occupancy_;}
  return changes;
 }
 FusionDelta delta(uint64_t after)const{
  FusionDelta d;d.snapshot=snapshot();d.base=after;d.full=after==0||after>d.snapshot->version||d.snapshot->version-after>32;if(d.full)d.base=0;
  for(size_t page=0;page<d.snapshot->pages.size();page++){const auto&p=*d.snapshot->pages[page];if(!d.full&&p.latest<=after)continue;for(size_t k=0;k<fusion_page_size;k++){auto id=page*fusion_page_size+k;if(id>=d.snapshot->count)break;if(d.full||p.changed[k]>after)d.ids.push_back(id);}}
  return d;
 }
};
}
