#include "diagnostic.hpp"
#include "live_policy.hpp"
#include <iostream>
using namespace lj2;
int main(int argc,char**argv){
 if(argc!=2)return 2;
 try{Camera base;Pose view;auto map=load_diagnostic(argv[1],base,view);
  for(int tier=1;tier<4;++tier){auto c=resized(base,ResolutionPolicy::widths[tier],ResolutionPolicy::heights[tier]);auto task=prepare(map,c,view);
   std::cout<<"{\"type\":\"task\",\"width\":"<<c.width<<",\"ok\":"<<task.budget_ok<<",\"surfaces\":"<<task.after.surfaces<<",\"refs\":"<<task.after.references<<",\"pixels\":"<<task.after.pixels<<"}\n";
   for(const auto&entry:map.blocks)for(int level=0;level<4;++level){Snapshot one;auto b=std::make_shared<Block>();for(auto&lod:b->lod)lod=entry.second->lod[level];one.blocks.emplace(entry.first,b);auto p=prepare(one,c,view,Budget{0,0,0});
    std::cout<<"{\"type\":\"block\",\"width\":"<<c.width<<",\"key\":["<<entry.first.x<<","<<entry.first.y<<","<<entry.first.z<<"],\"lod\":"<<level<<",\"surfaces\":"<<p.before.surfaces<<",\"refs\":"<<p.before.references<<",\"pixels\":"<<p.before.pixels<<"}\n";
   }
  }return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
