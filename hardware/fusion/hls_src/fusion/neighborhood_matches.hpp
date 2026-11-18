#pragma once

// All points in one center voxel see the same 27 immutable chains. Reuse
// their filtered, compact candidate list across consecutive batches of points.
struct LocalCandidate {
    SurfaceWord surface;
    unsigned id, cell, primary;
    uint64_t ordinal;
};
static constexpr unsigned neighborhood_limit=256;
#ifdef NATIONAL_TRACE
uint64_t frame_neighborhood_refills=0,frame_query_batches=0,frame_query_iterations=0,frame_dense_fallbacks=0;
#endif
#ifdef NATIONAL_CHAIN_WAVE
#include "chain_wave.hpp"
#endif
#ifdef NATIONAL_CELL_PRUNE
#include "cell_prune.hpp"
#endif

static unsigned load_neighborhood(Key3 group,const unsigned* buckets,const NodePort* nodes,
                                  BucketLine* bc,NodeLine* nc,LocalCandidate* candidates,uint64_t& visits
#ifdef NATIONAL_OCTANTS
                                  ,OctantLists& octants
#endif
                                  ){
#ifdef NATIONAL_OCTANTS
    for(unsigned b=0;b<9;++b)octants.sizes[b]=0;
#endif
#ifdef NATIONAL_TRACE
    ++frame_neighborhood_refills;
#endif
#ifdef NATIONAL_CHAIN_WAVE
    return load_chain_wave(group,buckets,nodes,bc,nc,candidates,visits);
#else
    Key3 keys[27];
    unsigned links[27];
    prefetch_group_heads(group,buckets,bc,keys,links);
    unsigned count=0;
    for(unsigned ci=0;ci<27;++ci){
        for(unsigned link=links[ci];link;){
            const unsigned id=link-1;
            const ResidentNode n=cached_node(id,nodes,nc);
            link=n.next;
            ++visits;
            if(!same(keys[ci],n))continue;
            if(count==neighborhood_limit){
#ifdef NATIONAL_TRACE
                ++frame_dense_fallbacks;
#endif
                return neighborhood_limit+1;
            }
#ifdef NATIONAL_OCTANTS
            const unsigned bin=normal_norm2(n.surface)<=1.001f?normal_octant(n.surface):8;
            octants.index[bin][octants.sizes[bin]++]=uint8_t(count);
#endif
            candidates[count++]={n.surface,id,ci,n.primary,n.insertion};
        }
    }
    return count;
#endif
}

static void query_neighborhood(const LocalCandidate* candidates,unsigned count,const SurfaceWord points[4],
                               unsigned used,LaneBest best[4]
#ifdef NATIONAL_OCTANTS
                               ,const OctantLists& octants
#endif
                               ){
#ifdef NATIONAL_OCTANTS
    unsigned allowed=0;
    for(unsigned lane=0;lane<4;++lane){
#pragma HLS UNROLL
        if(lane<used)allowed|=possible_octants(points[lane]);
    }
    unsigned ends[9];
#pragma HLS ARRAY_PARTITION variable=ends complete
    count=0;
    for(unsigned b=0;b<9;++b){
#pragma HLS UNROLL
        if(allowed&(1u<<b))count+=octants.sizes[b];
        ends[b]=count;
    }
#endif
#ifdef NATIONAL_TRACE
    ++frame_query_batches;frame_query_iterations+=count;
#endif
    for(unsigned i=0;i<count;++i){
#pragma HLS PIPELINE II=1
#ifdef NATIONAL_OCTANTS
        unsigned bin=0;
        for(unsigned b=0;b<8;++b){
#pragma HLS UNROLL
            if(i>=ends[b])bin=b+1;
        }
        const unsigned start=bin?ends[bin-1]:0;
        const auto c=candidates[octants.index[bin][i-start]];
#else
        const auto c=candidates[i];
#endif
        ResidentNode node{};
        node.surface=c.surface;node.primary=c.primary;node.insertion=c.ordinal;
        for(unsigned lane=0;lane<4;++lane){
#pragma HLS UNROLL
            if(lane<used)update_lane(points[lane],node,c.id,c.cell,best[lane]);
        }
    }
}
