#include "diagnostic.hpp"
#include "live_policy.hpp"
#include <iostream>
using namespace lj2;
template<class T>uint64_t digest(const std::vector<T>&data){uint64_t h=1469598103934665603ULL;auto*p=reinterpret_cast<const unsigned char*>(data.data());for(size_t i=0;i<data.size()*sizeof(T);++i){h^=p[i];h*=1099511628211ULL;}return h;}
int main(int argc,char**argv){if(argc!=2)return 2;Camera base;Pose view;auto map=load_diagnostic(argv[1],base,view);
 for(int tier=1;tier<4;++tier){auto c=resized(base,ResolutionPolicy::widths[tier],ResolutionPolicy::heights[tier]);
  // Offline CPU reference only: prevent LOD budget decisions from obscuring the
  // independent question whether tighter bounds omit any visible pixels.
  auto p=prepare(map,c,view,Budget{1000000,10000000,1000000000});auto im=render_cpu(p);
  std::cout<<c.width<<" "<<digest(im.rgb)<<" "<<digest(im.depth)<<" "<<digest(im.ids)<<"\n";
 }return 0;
}
