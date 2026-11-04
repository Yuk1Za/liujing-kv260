#include "fusion_store.hpp"
#include <iostream>
#include <fstream>
using namespace lj2;
int main(int argc,char**argv){
 bool baseline=argc>1&&std::string(argv[1])=="baseline";Scene old;FusionStore fast;
 auto now=[](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();};
 for(int frame=0;frame<40;frame++){
  std::vector<Point>p;for(int y=0;y<200;y++)for(int x=0;x<320;x++){float px=(x-160)*.007f+frame*.003f,py=(y-100)*.007f;p.push_back({px,py,2.f+.002f*std::sin(x*.1f),0,0,1,uint32_t((x*13+y*7)&0xffffff)});}
  double begin=now();if(baseline)old.integrate(p,frame+1);else fast.integrate(p,frame+1);double ms=now()-begin;
  auto t=fast.timing();std::cout<<"{\"frame\":"<<frame<<",\"ms\":"<<ms<<",\"groups_ms\":"<<t.grouping_ms<<",\"associate_ms\":"<<t.association_ms<<",\"commit_ms\":"<<t.commit_ms<<",\"count\":"<<(baseline?old.snapshot()->count:fast.snapshot()->count)<<"}\n";
 }
 if(argc>2){std::ofstream out(argv[2],std::ios::binary);if(baseline){for(auto&b:old.snapshot()->blocks)for(auto&s:b.second->lod[0])out.write((char*)&s,sizeof s);}else{auto snap=fast.snapshot();for(uint32_t id=0;id<snap->count;id++){auto legacy=legacy_surface(snap->at(id));out.write((char*)&legacy,sizeof(legacy));}}}
}
