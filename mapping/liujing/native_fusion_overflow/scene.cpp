#include "scene.hpp"

#include <algorithm>

#include <cmath>

#include <fstream>

#include <limits>

#include <set>

#include <cstring>

#include <unordered_map>

#include <chrono>



namespace lj2 {

Pose identity(){return {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};}

Pose multiply(const Pose&a,const Pose&b){Pose c{};for(int y=0;y<4;y++)for(int x=0;x<4;x++)for(int k=0;k<4;k++)c[y*4+x]+=a[y*4+k]*b[k*4+x];return c;}

Pose inverse(const Pose&a){Pose b=identity();for(int y=0;y<3;y++){for(int x=0;x<3;x++)b[y*4+x]=a[x*4+y];b[y*4+3]=-(b[y*4]*a[3]+b[y*4+1]*a[7]+b[y*4+2]*a[11]);}return b;}

void check_pose(const Pose&t){for(float v:t)if(!std::isfinite(v))throw std::invalid_argument("nonfinite pose");for(int y=0;y<3;y++)for(int x=0;x<3;x++){float v=0;for(int k=0;k<3;k++)v+=t[k*4+y]*t[k*4+x];if(std::fabs(v-(x==y))>.001f)throw std::invalid_argument("nonrigid pose");}float det=t[0]*(t[5]*t[10]-t[6]*t[9])-t[1]*(t[4]*t[10]-t[6]*t[8])+t[2]*(t[4]*t[9]-t[5]*t[8]);if(std::fabs(det-1)>.001f||std::fabs(t[12])+std::fabs(t[13])+std::fabs(t[14])+std::fabs(t[15]-1)>.001f)throw std::invalid_argument("invalid pose");}

static Key voxel(float x,float y,float z,float step=.02f){return {(int)std::floor(x/step),(int)std::floor(y/step),(int)std::floor(z/step)};}

static int floordiv(int a,int b){return a/b-(a%b<0);}

static Key blockkey(Key k){return {floordiv(k.x,50),floordiv(k.y,50),floordiv(k.z,50)};}

static uint32_t blend(uint32_t a,uint32_t b,float w){uint32_t c=0;for(int k=0;k<3;k++)c|=uint32_t((float((a>>(k*8))&255)*w+float((b>>(k*8))&255))/(w+1)+.5f)<<(k*8);return c;}

static float dot(const Surface&a,const Surface&b){return (a.nx*b.nx+a.ny*b.ny)+a.nz*b.nz;}

static void mix(Surface&a,const Surface&b){float w=std::min(a.weight,31.f);a.x=(a.x*w+b.x)/(w+1);a.y=(a.y*w+b.y)/(w+1);a.z=(a.z*w+b.z)/(w+1);a.rgb=blend(a.rgb,b.rgb,w);float nx=a.nx*w+b.nx,ny=a.ny*w+b.ny,nz=a.nz*w+b.nz,n=std::sqrt(nx*nx+ny*ny+nz*nz);a.nx=nx/n;a.ny=ny/n;a.nz=nz/n;a.weight=std::min(32.f,w+1);}

struct KeyHash{size_t operator()(const Key&k)const{uint64_t h=uint32_t(k.x)*0x9e3779b185ebca87ULL;h^=uint32_t(k.y)*0xc2b2ae3d27d4eb4fULL;h^=uint32_t(k.z)*0x165667b19e3779f9ULL;return size_t(h^(h>>32));}};

class CellIndex {

 struct Entry{Key key{};const std::vector<Surface>*value=nullptr;bool writable=false;};std::vector<Entry>table_;size_t mask_;uint64_t generation_=0;

public:

 explicit CellIndex(size_t maximum){size_t size=1;while(size<2*(maximum+1))size*=2;table_.resize(size);mask_=size-1;}

 void set(Key key,const std::vector<Surface>*value,bool writable=false){size_t i=KeyHash{}(key)&mask_;while(table_[i].value&&!(table_[i].key==key))i=(i+1)&mask_;table_[i]={key,value,writable};++generation_;}

 uint64_t generation()const{return generation_;}

 std::vector<Surface>*edit_existing(Key key)const{size_t i=KeyHash{}(key)&mask_;while(table_[i].value){if(table_[i].key==key)return table_[i].writable?const_cast<std::vector<Surface>*>(table_[i].value):nullptr;i=(i+1)&mask_;}return nullptr;}

 const std::vector<Surface>*get(Key key)const{size_t i=KeyHash{}(key)&mask_;while(table_[i].value){if(table_[i].key==key)return table_[i].value;i=(i+1)&mask_;}return nullptr;}

};

static void levels(Block&b){

 for(auto&l:b.lod)l.clear();for(auto&c:b.cells)for(auto&s:c.second)b.lod[0].push_back(s);

 for(int level=1;level<4;level++){

  // Contiguous bins avoid one allocation per voxel and normal group. Traverse

  // each bin in insertion order, preserving the original first-match rule.

  struct Bin{Key key{};int head=-1,tail=-1;};

  struct Node{Surface surface;int next=-1;};

  size_t capacity=1;while(capacity<2*(b.lod[0].size()+1))capacity*=2;

  std::vector<Bin>bins(capacity);std::vector<Node>nodes;nodes.reserve(b.lod[0].size());float size=.02f*(1<<level);

  for(const auto&s:b.lod[0]){

   Key key=voxel(s.x,s.y,s.z,size);size_t index=KeyHash{}(key)&(capacity-1);

   while(bins[index].head>=0&&!(bins[index].key==key))index=(index+1)&(capacity-1);

   auto&bin=bins[index];int match=bin.head;

   while(match>=0&&!(dot(nodes[match].surface,s)>.94f))match=nodes[match].next;

   if(match>=0){auto&target=nodes[match].surface;uint32_t id=std::min(target.id,s.id);mix(target,s);target.id=id;}

   else{int added=static_cast<int>(nodes.size());nodes.push_back({s,-1});if(bin.tail>=0)nodes[bin.tail].next=added;else{bin.key=key;bin.head=added;}bin.tail=added;}

  }

  b.lod[level].reserve(nodes.size());for(auto&node:nodes){node.surface.radius=size*.8f;b.lod[level].push_back(node.surface);}

 }

 for(auto&v:b.lod)std::sort(v.begin(),v.end(),[](const Surface&a,const Surface&b){return a.id<b.id;});

}

Scene::Scene(size_t cap):current_(std::make_shared<const Snapshot>()),capacity_(cap){if(!cap||cap>1000000)throw std::invalid_argument("map capacity");}

MapStats Scene::integrate(const std::vector<Point>&points,double stamp){

 std::lock_guard<std::mutex>lock(writer_);auto old=snapshot();if(!std::isfinite(stamp)||(old->version&&stamp<=old->source_stamp))throw std::invalid_argument("map timestamp regression");

 auto clock=[](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();};double started=clock();

 auto next=std::make_shared<Snapshot>(*old);std::map<Key,std::shared_ptr<Block>>dirty;

 // Writer-local lookup index preserves ordered storage, iteration and tie rules.

 // Copy-on-write must redirect every indexed vector to the copied block.

 CellIndex lookup(old->count+points.size());

 for(const auto&b:old->blocks)for(const auto&cell:b.second->cells)lookup.set(cell.first,&cell.second);

 auto edit=[&](Key k)->Block&{auto it=dirty.find(k);if(it!=dirty.end())return *it->second;auto b=std::make_shared<Block>();auto before=old->blocks.find(k);if(before!=old->blocks.end())b->cells=before->second->cells;dirty[k]=b;for(const auto&cell:b->cells)lookup.set(cell.first,&cell.second,true);return *b;};

 auto cell_edit=[&](Key k)->std::vector<Surface>&{if(auto v=lookup.edit_existing(k))return *v;auto&v=edit(blockkey(k)).cells[k];lookup.set(k,&v,true);return v;};

 MapStats changes;changes.index_ms=clock()-started;double indexed=clock();

 std::array<const std::vector<Surface>*,27> neighbors{};Key cached_cell{};uint64_t cached_generation=~uint64_t(0);

 for(const auto&p:points){

  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||std::max({std::fabs(p.x),std::fabs(p.y),std::fabs(p.z)})>10000)continue;

  float length=std::sqrt((p.nx*p.nx+p.ny*p.ny)+p.nz*p.nz);if(!std::isfinite(length)||length<.5f)continue;

  Surface incoming{};incoming.x=p.x;incoming.y=p.y;incoming.z=p.z;incoming.nx=p.nx/length;incoming.ny=p.ny/length;incoming.nz=p.nz/length;incoming.radius=.016f;incoming.rgb=p.rgb&0xffffff;incoming.weight=1;

  Key cell=voxel(p.x,p.y,p.z),best{};size_t index=0;float distance=.025f*.025f;bool found=false;

  // Adjacent depth pixels often occupy the same voxel. Reuse its ordered

  // neighborhood until copy-on-write or cell insertion redirects an index.

  if(!(cell==cached_cell)||cached_generation!=lookup.generation()){

   int j=0;for(int z=-1;z<=1;++z)for(int y=-1;y<=1;++y)for(int x=-1;x<=1;++x)neighbors[j++]=lookup.get({cell.x+x,cell.y+y,cell.z+z});

   cached_cell=cell;cached_generation=lookup.generation();

  }

  int neighbor=0;for(int z=-1;z<=1;z++)for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++){

   Key k{cell.x+x,cell.y+y,cell.z+z};auto c=neighbors[neighbor++];if(!c)continue;

   for(size_t i=0;i<c->size();i++){const auto&s=(*c)[i];float dx=p.x-s.x,dy=p.y-s.y,dz=p.z-s.z,d=(dx*dx+dy*dy)+dz*dz;if(d<distance&&dot(s,incoming)>.94f&&std::fabs((dx*s.nx+dy*s.ny)+dz*s.nz)<.012f){distance=d;best=k;index=i;found=true;}}

  }

  if(found){auto&v=cell_edit(best);Surface s=v[index];mix(s,incoming);Key dest=voxel(s.x,s.y,s.z);

   if(dest==best)v[index]=s;

   else {auto&target=cell_edit(dest);if(target.size()<4){v.erase(v.begin()+index);target.push_back(s);}else{Surface keep=v[index];keep.rgb=s.rgb;keep.weight=s.weight;v[index]=keep;}}

   ++changes.merged;

  }else{if(next->count>=capacity_){++changes.capacity_drops;continue;}auto&v=cell_edit(cell);if(v.size()>=4){++changes.orientation_drops;continue;}if(next->count>=capacity_){++changes.capacity_drops;continue;}incoming.id=next_id_++;v.push_back(incoming);++next->count;++changes.added;}

 }

 changes.integrate_ms=clock()-indexed;double integrated=clock();

 for(auto&b:dirty){for(auto i=b.second->cells.begin();i!=b.second->cells.end();)if(i->second.empty())i=b.second->cells.erase(i);else ++i;if(b.second->cells.empty())next->blocks.erase(b.first);else{levels(*b.second);next->blocks[b.first]=b.second;}}changes.cloned_blocks=dirty.size();if(changes.added||changes.merged){next->version++;next->source_stamp=stamp;}

 changes.lod_ms=clock()-integrated;

 std::atomic_store(&current_,std::shared_ptr<const Snapshot>(next));stats_.added+=changes.added;stats_.merged+=changes.merged;stats_.capacity_drops+=changes.capacity_drops;stats_.orientation_drops+=changes.orientation_drops;stats_.cloned_blocks+=changes.cloned_blocks;return changes;

}

static bool project(Surface&s,const Camera&c,const Pose&inv){

 float x=s.x,y=s.y,z=s.z,nx=s.nx,ny=s.ny,nz=s.nz;

 s.x=(inv[0]*x+inv[1]*y)+inv[2]*z+inv[3];s.y=(inv[4]*x+inv[5]*y)+inv[6]*z+inv[7];s.z=(inv[8]*x+inv[9]*y)+inv[10]*z+inv[11];

 s.nx=(inv[0]*nx+inv[1]*ny)+inv[2]*nz;s.ny=(inv[4]*nx+inv[5]*ny)+inv[6]*nz;s.nz=(inv[8]*nx+inv[9]*ny)+inv[10]*nz;

 if(s.z+s.radius<c.near_z||s.z-s.radius>c.far_z)return false;

 float zn=std::max(c.near_z,s.z-s.radius),zf=std::max(zn,s.z+s.radius),xs[4]={(s.x-s.radius)/zn,(s.x-s.radius)/zf,(s.x+s.radius)/zn,(s.x+s.radius)/zf},ys[4]={(s.y-s.radius)/zn,(s.y-s.radius)/zf,(s.y+s.radius)/zn,(s.y+s.radius)/zf};

 float l=c.fx**std::min_element(xs,xs+4)+c.cx,r=c.fx**std::max_element(xs,xs+4)+c.cx,t=c.fy**std::min_element(ys,ys+4)+c.cy,b=c.fy**std::max_element(ys,ys+4)+c.cy;

 // Roundoff margin also protects the near-plane sphere fallback.
 l-=.01f;r+=.01f;t-=.01f;b+=.01f;
 // Tight perspective bounds of the oriented disk. Fall back to the old sphere

 // bound near the near plane or for ill-conditioned normals. Numeric raster

 // math is unchanged; expand extrema before outward pixel rounding.

 double norm=double(s.nx)*s.nx+double(s.ny)*s.ny+double(s.nz)*s.nz;

 if(norm>.99&&norm<1.01&&s.z-s.radius>c.near_z){

  double nx=s.nx/std::sqrt(norm),ny=s.ny/std::sqrt(norm),nz=s.nz/std::sqrt(norm),rr=double(s.radius)*s.radius;

  double A=double(s.z)*s.z-rr*(1-nz*nz);

  auto bounds=[&](double center,double normal,double focal,double principal,float&lo,float&hi){

   double mid=center*s.z+rr*normal*nz,C=center*center-rr*(1-normal*normal),disc=mid*mid-A*C;

   if(A>1e-10&&disc>=0){double root=std::sqrt(disc);double lower=focal*(mid-root)/A+principal,upper=focal*(mid+root)/A+principal;

    if(std::isfinite(lower)&&std::isfinite(upper)){lo=std::max(lo,float(lower-.01));hi=std::min(hi,float(upper+.01));}}

  };

  bounds(s.x,nx,c.fx,c.cx,l,r);bounds(s.y,ny,c.fy,c.cy,t,b);

 }



 if(r<0||b<0||l>c.width-1||t>c.height-1)return false;

 // Raster samples integer pixel centers: only ceil(lower)..floor(upper) can hit.
 // The analytic bounds already include a 0.01-pixel numerical margin.
 s.x0=(int)std::ceil(std::max(0.f,l));s.x1=(int)std::floor(std::min(float(c.width-1),r));s.y0=(int)std::ceil(std::max(0.f,t));s.y1=(int)std::floor(std::min(float(c.height-1),b));return s.x0<=s.x1&&s.y0<=s.y1;

}

Prepared prepare(const Snapshot&map,const Camera&c,const Pose&view,Budget budget){

 check_pose(view);for(float v:{c.fx,c.fy,c.cx,c.cy,c.near_z,c.far_z})if(!std::isfinite(v))throw std::invalid_argument("nonfinite camera");if(c.width<1||c.height<1||c.width>640||c.height>480||!(c.fx>0&&c.fy>0&&c.near_z>0&&c.far_z>c.near_z))throw std::invalid_argument("camera");

 Prepared out;out.map_version=map.version;out.map_stamp=map.source_stamp;out.view=view;out.camera=c;Pose inv=inverse(view);

 struct Projection{bool ready=false;std::vector<Surface>visible;uint64_t refs=0,pixels=0;};
 struct Choice{Key key;const Block*b;float distance;int level=0;std::array<Projection,4>projections;};std::vector<Choice>choices;
 auto fill=[&](Choice&a,int level)->Projection&{auto&v=a.projections[level];if(!v.ready){v.ready=true;v.visible.reserve(a.b->lod[level].size());for(auto surface:a.b->lod[level])if(project(surface,c,inv)){v.refs+=uint64_t(surface.x1/16-surface.x0/16+1)*(surface.y1/16-surface.y0/16+1);v.pixels+=uint64_t(surface.x1-surface.x0+1)*(surface.y1-surface.y0+1);v.visible.push_back(surface);}}return v;};
 uint64_t count=0,refs=0,pixels=0;
 // Small maps start at the finest representation. Larger maps start from
 // projected LOD to bound preparation work; every replacement is cost-checked.
 for(auto&b:map.blocks){float dx=b.first.x+.5f-view[3],dy=b.first.y+.5f-view[7],dz=b.first.z+.5f-view[11],dist=std::sqrt(dx*dx+dy*dy+dz*dz);if(dist>c.far_z+1)continue;float lod_distance=dist*477.6f/c.fx;int initial=map.count<=budget.surfaces?0:lod_distance<2?0:lod_distance<4?1:lod_distance<8?2:3;Choice a{b.first,b.second.get(),dist,initial,{}};auto&v=fill(a,initial);count+=v.visible.size();refs+=v.refs;pixels+=v.pixels;choices.push_back(std::move(a));}
 std::sort(choices.begin(),choices.end(),[](const Choice&a,const Choice&b){return a.distance!=b.distance?a.distance>b.distance:a.key<b.key;});
 out.before={count,refs,pixels};
 auto score=[&](uint64_t n,uint64_t r,uint64_t p){double a=double(n)/std::max(1u,budget.surfaces),b=double(r)/std::max(1u,budget.references),d=double(p)/std::max(uint64_t(1),budget.pixels);return std::pair<double,double>{std::max({a,b,d}),a+b+d};};
 for(int pass=0;pass<4&&(count>budget.surfaces||refs>budget.references||pixels>budget.pixels);++pass){
  bool changed=false;
  for(auto&a:choices){
   if(count<=budget.surfaces&&refs<=budget.references&&pixels<=budget.pixels)break;
   const auto&old=fill(a,a.level);auto best_score=score(count,refs,pixels);int best=a.level;
   for(int level=0;level<4;++level){if(level==a.level)continue;const auto&candidate=fill(a,level);auto cost=score(count-old.visible.size()+candidate.visible.size(),refs-old.refs+candidate.refs,pixels-old.pixels+candidate.pixels);if(cost<best_score){best_score=cost;best=level;}}
   if(best!=a.level){const auto&selected=fill(a,best);count=count-old.visible.size()+selected.visible.size();refs=refs-old.refs+selected.refs;pixels=pixels-old.pixels+selected.pixels;if(best>a.level)++out.coarsened_blocks;++out.lod_changes;a.level=best;changed=true;}
  }if(!changed)break;
 }
 // A single-block descent can get stuck between the count and coverage limits.
 // Try 71 deterministic whole-map weighted assignments, accepting only a strict
 // decrease in the same measured pressure. Each block/level is projected once.
 if(count>budget.surfaces||refs>budget.references||pixels>budget.pixels){
  const double weights[]={0,.25,.5,1,2,4};auto best_score=score(count,refs,pixels);std::vector<int>best_levels,candidate_levels(choices.size());WorkCounts best_counts{count,refs,pixels};
  for(double wn:weights)for(double wr:{0.,1.})for(double wp:weights){
   if(wn==0&&wr==0&&wp==0)continue;WorkCounts cost;
   for(size_t i=0;i<choices.size();++i){auto&a=choices[i];int selected=0;double minimum=std::numeric_limits<double>::infinity();for(int level=0;level<4;++level){const auto&v=fill(a,level);double weighted=wn*v.visible.size()/std::max(1u,budget.surfaces)+wr*v.refs/std::max(1u,budget.references)+wp*v.pixels/std::max(uint64_t(1),budget.pixels);if(weighted<minimum){minimum=weighted;selected=level;}}candidate_levels[i]=selected;const auto&v=fill(a,selected);cost.surfaces+=v.visible.size();cost.references+=v.refs;cost.pixels+=v.pixels;}
   auto pressure=score(cost.surfaces,cost.references,cost.pixels);if(pressure<best_score){best_score=pressure;best_counts=cost;best_levels=candidate_levels;}
  }
  if(!best_levels.empty()){for(size_t i=0;i<choices.size();++i)if(best_levels[i]!=choices[i].level){if(best_levels[i]>choices[i].level)++out.coarsened_blocks;++out.lod_changes;choices[i].level=best_levels[i];}count=best_counts.surfaces;refs=best_counts.references;pixels=best_counts.pixels;}
 }
 out.after={count,refs,pixels};out.exceeded=(count>budget.surfaces?1u:0u)|(refs>budget.references?2u:0u)|(pixels>budget.pixels?4u:0u);

 out.budget_ok=out.exceeded==0;out.candidate_pixels=pixels;

 if(!out.budget_ok)return out;

 std::vector<const Surface*> ordered;ordered.reserve(count);for(auto&a:choices)for(const auto&s:a.projections[a.level].visible)ordered.push_back(&s);std::sort(ordered.begin(),ordered.end(),[](const Surface*a,const Surface*b){return a->id<b->id;});out.surfaces.reserve(count);for(auto s:ordered)out.surfaces.push_back(*s);

 int txs=(c.width+15)/16,tiles=txs*((c.height+15)/16);out.offsets.assign(tiles+1,0);

 for(auto&s:out.surfaces)for(int y=s.y0/16;y<=s.y1/16;y++)for(int x=s.x0/16;x<=s.x1/16;x++)++out.offsets[y*txs+x+1];

 for(int i=1;i<=tiles;i++)out.offsets[i]+=out.offsets[i-1];out.indices.resize(out.offsets.back());auto cursor=out.offsets;

 for(uint32_t i=0;i<out.surfaces.size();i++){auto&s=out.surfaces[i];for(int y=s.y0/16;y<=s.y1/16;y++)for(int x=s.x0/16;x<=s.x1/16;x++)out.indices[cursor[y*txs+x]++]=i;}

 return out;

}

Image render_cpu(const Prepared&p,bool debug){

 if(!p.budget_ok)throw std::runtime_error("visibility budget exceeded");auto&c=p.camera;Image out;out.width=c.width;out.height=c.height;size_t n=c.width*c.height;out.rgb.assign(n,0);if(debug){out.depth.assign(n,0);out.ids.assign(n,-1);}

 const int txs=(c.width+15)/16,tiles=txs*((c.height+15)/16);

 float rays_x[640],rays_y[480];for(int x=0;x<c.width;x++)rays_x[x]=(x-c.cx)/c.fx;for(int y=0;y<c.height;y++)rays_y[y]=(y-c.cy)/c.fy;

 #pragma omp parallel for num_threads(2) schedule(dynamic,4)

 for(int tile=0;tile<tiles;tile++){

  int tx=tile%txs,ty=tile/txs;float depths[256]={};int32_t owners[256];uint32_t colors[256]={};std::fill(owners,owners+256,-1);

  for(uint32_t j=p.offsets[tile];j<p.offsets[tile+1];j++){

   const auto&s=p.surfaces[p.indices[j]];float num=(s.nx*s.x+s.ny*s.y)+s.nz*s.z,r2=s.radius*s.radius;

   for(int y=std::max(ty*16,s.y0);y<=std::min(ty*16+15,s.y1);y++)for(int x=std::max(tx*16,s.x0);x<=std::min(tx*16+15,s.x1);x++){

    float rx=rays_x[x],ry=rays_y[y],den=(s.nx*rx+s.ny*ry)+s.nz,z=num/den,dx=rx*z-s.x,dy=ry*z-s.y,dz=z-s.z;

    if(std::fabs(den)<1e-6f||!(z>=c.near_z&&z<=c.far_z&&((dx*dx+dy*dy)+dz*dz)<r2))continue;

    int k=(y-ty*16)*16+x-tx*16;if(owners[k]<0||z<depths[k]||(z==depths[k]&&s.id<uint32_t(owners[k]))){depths[k]=z;owners[k]=s.id;colors[k]=s.rgb;}

   }

  }

  for(int y=0;y<std::min(16,c.height-ty*16);y++)for(int x=0;x<std::min(16,c.width-tx*16);x++){int local=y*16+x,global=(ty*16+y)*c.width+tx*16+x;out.rgb[global]=colors[local];if(debug){out.depth[global]=depths[local];out.ids[global]=owners[local];}}

 }return out;

}

std::vector<Point> unproject(const uint16_t*d,const uint8_t*rgb,const Camera&c,const Pose&t,float scale){

 check_pose(t);std::vector<Point>out;out.reserve(c.width*c.height);auto pt=[&](int x,int y){float z=d[y*c.width+x]*scale;return std::array<float,3>{(x-c.cx)*z/c.fx,(y-c.cy)*z/c.fy,z};};

 for(int y=0;y<c.height-1;y++)for(int x=0;x<c.width-1;x++){auto a=pt(x,y),b=pt(x+1,y),e=pt(x,y+1);if(a[2]<c.near_z||a[2]>c.far_z||b[2]<c.near_z||e[2]<c.near_z||std::fabs(b[2]-a[2])>.03f||std::fabs(e[2]-a[2])>.03f)continue;

  // Preserve each measured point and its original edge validity. Estimate the

  // tangent over a symmetric five-pixel footprint when all endpoints belong to

  // the same depth surface; fall back at discontinuities and image boundaries.

  auto tangent=[&](int dx,int dy,std::array<float,3> fallback){

   int x0=x-2*dx,y0=y-2*dy,x1=x+2*dx,y1=y+2*dy;

   if(x0>=0&&y0>=0&&x1<c.width&&y1<c.height){auto lo=pt(x0,y0),hi=pt(x1,y1);

    if(lo[2]>=c.near_z&&hi[2]>=c.near_z&&lo[2]<=c.far_z&&hi[2]<=c.far_z&&std::fabs(lo[2]-a[2])<=.03f&&std::fabs(hi[2]-a[2])<=.03f){for(int k=0;k<3;++k)hi[k]-=lo[k];return hi;}}

   for(int k=0;k<3;++k)fallback[k]-=a[k];return fallback;

  };

  b=tangent(1,0,b);e=tangent(0,1,e);float nx=b[1]*e[2]-b[2]*e[1],ny=b[2]*e[0]-b[0]*e[2],nz=b[0]*e[1]-b[1]*e[0],norm=std::sqrt(nx*nx+ny*ny+nz*nz);if(norm<1e-10f)continue;

  nx/=norm;ny/=norm;nz/=norm;Point p{};p.x=(t[0]*a[0]+t[1]*a[1])+t[2]*a[2]+t[3];p.y=(t[4]*a[0]+t[5]*a[1])+t[6]*a[2]+t[7];p.z=(t[8]*a[0]+t[9]*a[1])+t[10]*a[2]+t[11];p.nx=(t[0]*nx+t[1]*ny)+t[2]*nz;p.ny=(t[4]*nx+t[5]*ny)+t[6]*nz;p.nz=(t[8]*nx+t[9]*ny)+t[10]*nz;int k=(y*c.width+x)*3;p.rgb=rgb[k]|(uint32_t(rgb[k+1])<<8)|(uint32_t(rgb[k+2])<<16);out.push_back(p);

 }return out;

}

std::vector<Point> unproject_cached(const uint16_t*d,const uint8_t*rgb,const Camera&c,const Pose&t,float scale){

 check_pose(t);std::vector<Point>out;out.reserve(c.width*c.height);// Reuse exact per-pixel camera coordinates across the seven-point stencil.
 // Preserve operation order; no reciprocal approximation or depth resampling.
 thread_local std::vector<std::array<float,3>> camera_points;camera_points.resize(size_t(c.width)*c.height);
 for(int y=0;y<c.height;y++)for(int x=0;x<c.width;x++){float z=d[y*c.width+x]*scale;camera_points[y*c.width+x]={(x-c.cx)*z/c.fx,(y-c.cy)*z/c.fy,z};}
 auto pt=[&](int x,int y){return camera_points[y*c.width+x];};

 for(int y=0;y<c.height-1;y++)for(int x=0;x<c.width-1;x++){auto a=pt(x,y),b=pt(x+1,y),e=pt(x,y+1);if(a[2]<c.near_z||a[2]>c.far_z||b[2]<c.near_z||e[2]<c.near_z||std::fabs(b[2]-a[2])>.03f||std::fabs(e[2]-a[2])>.03f)continue;

  // Preserve each measured point and its original edge validity. Estimate the

  // tangent over a symmetric five-pixel footprint when all endpoints belong to

  // the same depth surface; fall back at discontinuities and image boundaries.

  auto tangent=[&](int dx,int dy,std::array<float,3> fallback){

   int x0=x-2*dx,y0=y-2*dy,x1=x+2*dx,y1=y+2*dy;

   if(x0>=0&&y0>=0&&x1<c.width&&y1<c.height){auto lo=pt(x0,y0),hi=pt(x1,y1);

    if(lo[2]>=c.near_z&&hi[2]>=c.near_z&&lo[2]<=c.far_z&&hi[2]<=c.far_z&&std::fabs(lo[2]-a[2])<=.03f&&std::fabs(hi[2]-a[2])<=.03f){for(int k=0;k<3;++k)hi[k]-=lo[k];return hi;}}

   for(int k=0;k<3;++k)fallback[k]-=a[k];return fallback;

  };

  b=tangent(1,0,b);e=tangent(0,1,e);float nx=b[1]*e[2]-b[2]*e[1],ny=b[2]*e[0]-b[0]*e[2],nz=b[0]*e[1]-b[1]*e[0],norm=std::sqrt(nx*nx+ny*ny+nz*nz);if(norm<1e-10f)continue;

  nx/=norm;ny/=norm;nz/=norm;Point p{};p.x=(t[0]*a[0]+t[1]*a[1])+t[2]*a[2]+t[3];p.y=(t[4]*a[0]+t[5]*a[1])+t[6]*a[2]+t[7];p.z=(t[8]*a[0]+t[9]*a[1])+t[10]*a[2]+t[11];p.nx=(t[0]*nx+t[1]*ny)+t[2]*nz;p.ny=(t[4]*nx+t[5]*ny)+t[6]*nz;p.nz=(t[8]*nx+t[9]*ny)+t[10]*nz;int k=(y*c.width+x)*3;p.rgb=rgb[k]|(uint32_t(rgb[k+1])<<8)|(uint32_t(rgb[k+2])<<16);out.push_back(p);

 }return out;

}

void save_map(const Snapshot&s,const char*path){std::ofstream f(path,std::ios::binary);if(!f)throw std::runtime_error("map save open failed");f<<"ply\nformat binary_little_endian 1.0\ncomment map_version "<<s.version<<"\nelement vertex "<<s.count<<"\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";for(auto&b:s.blocks)for(auto&p:b.second->lod[0]){float xyz[]={p.x,p.y,p.z};char rgb[]={char(p.rgb),char(p.rgb>>8),char(p.rgb>>16)};f.write((char*)xyz,12);f.write(rgb,3);}if(!f)throw std::runtime_error("map save failed");}

// Quaternion interpolation, with exact endpoints and no extrapolation.

static std::array<float,4>quat(const Pose&m){std::array<float,4>q{};float trace=m[0]+m[5]+m[10];if(trace>0){float s=2*std::sqrt(trace+1);q={ (m[9]-m[6])/s,(m[2]-m[8])/s,(m[4]-m[1])/s,s/4};}else{int i=m[5]>m[0]?1:0;if(m[10]>m[i*5])i=2;int j=(i+1)%3,k=(i+2)%3;float s=2*std::sqrt(1+m[i*5]-m[j*5]-m[k*5]);q[i]=s/4;q[j]=(m[i*4+j]+m[j*4+i])/s;q[k]=(m[i*4+k]+m[k*4+i])/s;q[3]=(m[k*4+j]-m[j*4+k])/s;}return q;}

bool Poses::sample(const std::vector<TimedPose>&v,double stamp,Pose&out){for(size_t i=0;i<v.size();i++){if(std::fabs(v[i].stamp-stamp)<1e-7){out=v[i].pose;return true;}if(i&&v[i-1].stamp<stamp&&stamp<v[i].stamp){auto&a=v[i-1];auto&b=v[i];if(b.stamp-a.stamp>.3)return false;float t=(stamp-a.stamp)/(b.stamp-a.stamp);auto qa=quat(a.pose),qb=quat(b.pose);float dot=0;for(int k=0;k<4;k++)dot+=qa[k]*qb[k];if(dot<0){for(float&x:qb)x=-x;dot=-dot;}float wa=1-t,wb=t;if(dot<.9995f){float angle=std::acos(std::clamp(dot,-1.f,1.f));wa=std::sin((1-t)*angle)/std::sin(angle);wb=std::sin(t*angle)/std::sin(angle);}float n=0;for(int k=0;k<4;k++){qa[k]=wa*qa[k]+wb*qb[k];n+=qa[k]*qa[k];}for(float&x:qa)x/=std::sqrt(n);float x=qa[0],y=qa[1],z=qa[2],w=qa[3];out={1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),0,2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),0,2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),0,0,0,0,1};for(int k:{3,7,11})out[k]=(1-t)*a.pose[k]+t*b.pose[k];return true;}}return false;}

bool Poses::add(bool mapped,double stamp,const Pose&p){check_pose(p);std::lock_guard<std::mutex>l(mutex_);if(failed_)return false;auto&v=mapped?mapped_:raw_;if(!std::isfinite(stamp)||(!v.empty()&&stamp<=v.back().stamp)){failed_=true;return false;}if(!v.empty()){auto&a=v.back();float d=0;for(int k:{3,7,11})d+=(p[k]-a.pose[k])*(p[k]-a.pose[k]);auto q=quat(p),old=quat(a.pose);float dot=0;for(int i=0;i<4;i++)dot+=q[i]*old[i];double dt=stamp-a.stamp;if(std::sqrt(d)/dt>2||2*std::acos(std::min(1.f,std::fabs(dot)))/dt>2){failed_=true;return false;}}v.push_back({stamp,p});if(v.size()>512)v.erase(v.begin());return true;}

bool Poses::corrected(double stamp,Pose&out,double&anchor){std::lock_guard<std::mutex>l(mutex_);if(failed_)return false;Pose current,old;if(!sample(raw_,stamp,current))return false;for(auto i=mapped_.rbegin();i!=mapped_.rend();++i)if(i->stamp<=stamp){if(stamp-i->stamp>1)return false;if(!sample(raw_,i->stamp,old))continue;out=multiply(multiply(i->pose,inverse(old)),current);anchor=i->stamp;return true;}return false;}

}

