#include "diagnostic.hpp"
#include "live_policy.hpp"
#include <iostream>
#include <random>
using namespace lj2;
static Prepared unrestricted(Prepared p){
 int tx=(p.camera.width+15)/16,tiles=tx*((p.camera.height+15)/16);p.offsets.clear();p.indices.clear();p.offsets.push_back(0);
 for(auto&s:p.surfaces){s.x0=s.y0=0;s.x1=p.camera.width-1;s.y1=p.camera.height-1;}
 for(int tile=0;tile<tiles;++tile){for(uint32_t i=0;i<p.surfaces.size();++i)p.indices.push_back(i);p.offsets.push_back(p.indices.size());}return p;
}
int main(){
 std::mt19937 rng(123456);std::uniform_real_distribution<float>u(-1,1);
 for(int trial=0;trial<160;++trial){
  Snapshot map;auto block=std::make_shared<Block>();
  for(int i=0;i<12;++i){Surface s{};s.x=u(rng)*1.3f;s.y=u(rng);s.z=.31f+(u(rng)+1)*1.2f;s.radius=.008f+(u(rng)+1)*.12f;s.nx=u(rng);s.ny=u(rng);s.nz=u(rng);float n=std::sqrt(s.nx*s.nx+s.ny*s.ny+s.nz*s.nz);s.nx/=n;s.ny/=n;s.nz/=n;s.id=i;s.rgb=rng()&0xffffff;for(auto&lod:block->lod)lod.push_back(s);}
  map.blocks[{0,0,0}]=block;map.count=12;Camera c{48,32,30,30,23.5f,15.5f,.3f,5};auto p=prepare(map,c,identity(),Budget{40000,100000,4000000});auto full=unrestricted(p);
  // Include surfaces culled by the tight projection too, so false culling is caught.
  full.surfaces=block->lod[0];full=unrestricted(full);auto a=render_cpu(p),b=render_cpu(full);
  if(a.rgb!=b.rgb||a.depth!=b.depth||a.ids!=b.ids){std::cerr<<"bounds mismatch "<<trial<<"\n";return 1;}
 }
 std::cout<<"PASS 160 deterministic scenes vs all-pixel/all-surface reference\n";
}
