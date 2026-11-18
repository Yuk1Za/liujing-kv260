#include "fusion_frame.hpp"
#include <cstdio>
#include <vector>
#include <stdexcept>
#include <cstring>
#ifdef NATIONAL_INTEGER_RGB
#include "merge_surface.hpp"
static void compare_all_channel_updates(){
    for(unsigned w=1;w<=31;++w)for(unsigned old=0;old<256;++old)for(unsigned sample=0;sample<256;++sample){
        unsigned expected=unsigned((float(old)*float(w)+float(sample))/float(w+1)+.5f);
        if(merge_rgb(old*0x010101u,sample*0x010101u,w)!=expected*0x010101u)
            throw std::runtime_error("integer RGB differs from frozen binary32 rounding");
    }
}
#endif
static_assert(sizeof(Observation)==32 && sizeof(ResidentNode)==64 && sizeof(UpdateResult)==64);
int main() {
#ifdef NATIONAL_INTEGER_RGB
    compare_all_channel_updates();
#endif
    std::vector<unsigned> buckets(1<<21);
    std::vector<ResidentNode> nodes(8);
    unsigned touched[8]{};
    UpdateResult delta[8]{};
    int matches[8]{};
    Observation inputs[8]={{-.03f,0,0,0,0,1,1},{.03f,0,0,0,0,1,2},{.03f,0,0,0,0,1,2}};
    FusionResult r{};
#ifdef NATIONAL_HOST_NORMALS
    for(auto& input:inputs)input.pad=valid_unit_normal;
#endif
#ifdef NATIONAL_NARROW
    FrameBeat in[16]{},resident[32]{},output[32]{},status[4]{};
    auto submit=[&](unsigned count,unsigned job){
        std::memcpy(in,inputs,sizeof inputs);
#ifdef NATIONAL_CYCLE_COUNTERS
        volatile uint32_t tick=1234;
        fusion_frame(in,buckets.data(),resident,touched,output,matches,3,count,2,job,status,tick);
        if(status[3].words[2]!=job || status[3].words[3]!=0x4c4a504d)throw std::runtime_error("cycle report identity");
#else
        fusion_frame(in,buckets.data(),resident,touched,output,matches,3,count,2,job,status);
#endif
        std::memcpy(&r,status,sizeof r);
        std::memcpy(nodes.data(),resident,nodes.size()*sizeof(ResidentNode));
        std::memcpy(delta,output,sizeof delta);
    };
#else
    auto submit=[&](unsigned count,unsigned job){
        fusion_frame(inputs,buckets.data(),nodes.data(),touched,delta,matches,3,count,2,job,&r);
    };
#endif
    submit(0,1);
    if(r.count!=2 || r.dirty!=2 || r.merged!=1 || nodes[1].surface.weight!=2) return 1;
    inputs[2].x=10;
    submit(2,2);
    if(matches[0]!=0 || matches[1]!=1 || matches[2]!=-1 || r.dropped!=1 || r.merged!=2)return 2;
#if defined(NATIONAL_CHAIN_WAVE) && defined(NATIONAL_CYCLE_COUNTERS)
    inputs[1].pad=0;
    std::memcpy(in,inputs,sizeof inputs);
    volatile uint32_t raw_tick=0;
    fusion_frame(in,buckets.data(),resident,touched,output,matches,3,2,2,3,status,raw_tick);
    std::memcpy(&r,status,sizeof r);
    if(r.job!=0 || r.dirty!=0)return 3;
#endif
    puts("PASS frame insert, stable matches, duplicate and capacity semantics");
}
