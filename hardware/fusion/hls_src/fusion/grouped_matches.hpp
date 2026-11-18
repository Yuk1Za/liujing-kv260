#pragma once
// Query phase reads one immutable map. Commit intentionally bypasses these caches.
#ifdef NATIONAL_TRACE
uint64_t frame_bucket_misses=0,frame_node_misses=0,frame_bucket_visits=0;
#endif
static unsigned spatial_order(Key3 k){
    // Wrap only the ordering key. Group equality still checks all three signed coordinates.
#ifdef NATIONAL_MORTON
    unsigned key=0;
    for(unsigned bit=0;bit<8;++bit){
#pragma HLS UNROLL
        key|=((unsigned(k.x)>>bit)&1u)<<(3*bit);
        key|=((unsigned(k.y)>>bit)&1u)<<(3*bit+1);
        key|=((unsigned(k.z)>>bit)&1u)<<(3*bit+2);
    }
    return key;
#else
    return ((unsigned(k.z)&255u)<<16)|((unsigned(k.y)&255u)<<8)|(unsigned(k.x)&255u);
#endif
}
#ifdef NATIONAL_TILE_HEADS
struct BucketLine {unsigned tag;unsigned heads[64];};
static constexpr unsigned bucket_cache_lines=256;
#else
struct BucketLine { unsigned tag,head; };
static constexpr unsigned bucket_cache_lines=4096;
#endif
struct NodeLine { unsigned tag;ResidentNode value; };
#ifdef NATIONAL_OCTANTS
#include "normal_octants.hpp"
#endif
#ifdef NATIONAL_GROUP_PREFETCH
// A 3x3x3 voxel neighborhood touches at most two 4-voxel tiles along each axis.
// Adjacent tile coordinates have different parity, so all rows needed by one
// neighborhood can coexist, even when their DDR hash rows collide.
static void prefetch_group_heads(Key3 center,const unsigned* buckets,BucketLine cache[bucket_cache_lines],
                                 Key3 keys[27],unsigned links[27]){
    unsigned rows[8]={},addresses[27];
#pragma HLS ARRAY_PARTITION variable=rows complete
    for(unsigned ci=0;ci<27;++ci){
#pragma HLS PIPELINE II=1
        Key3 k{center.x+int(ci%3)-1,center.y+int(ci/3%3)-1,center.z+int(ci/9)-1};
        const unsigned bank=((unsigned(k.x)>>2)&1)|((unsigned(k.y)>>1)&2)|(unsigned(k.z)&4);
        const unsigned slot=hash(k),row=slot>>6,line=((row&31)<<3)|bank;
        keys[ci]=k;rows[bank]=row+1;
        addresses[ci]=(line<<6)|(slot&63);
    }
    for(unsigned bank=0;bank<8;++bank){
        if(!rows[bank])continue;
        const unsigned row=rows[bank]-1,line=((row&31)<<3)|bank;
        if(cache[line].tag==row+1)continue;
#ifdef NATIONAL_TRACE
        ++frame_bucket_misses;
#endif
        for(unsigned i=0;i<64;++i){
#pragma HLS PIPELINE II=1
            cache[line].heads[i]=buckets[(row<<6)+i];
        }
        cache[line].tag=row+1;
    }
    for(unsigned ci=0;ci<27;++ci){
#pragma HLS PIPELINE II=1
        const unsigned address=addresses[ci];
        links[ci]=cache[address>>6].heads[address&63];
    }
#ifdef NATIONAL_TRACE
    frame_bucket_visits+=27;
#endif
}
#endif
static unsigned cached_head(unsigned slot,const unsigned* buckets,BucketLine cache[bucket_cache_lines]) {
#ifdef NATIONAL_TILE_HEADS
    unsigned row=slot>>6,line=row&(bucket_cache_lines-1);
#ifdef NATIONAL_TRACE
    ++frame_bucket_visits;
    if(cache[line].tag!=row+1)++frame_bucket_misses;
#endif
    if(cache[line].tag!=row+1){
        for(unsigned i=0;i<64;++i){
#pragma HLS PIPELINE II=1
            cache[line].heads[i]=buckets[(row<<6)+i];
        }
        cache[line].tag=row+1;
    }
    return cache[line].heads[slot&63];
#else
    const unsigned line=slot&4095;
    BucketLine entry=cache[line];
#ifdef NATIONAL_TRACE
    ++frame_bucket_visits;if(entry.tag!=slot+1)++frame_bucket_misses;
#endif
    if(entry.tag!=slot+1){entry={slot+1,buckets[slot]};cache[line]=entry;}
    return entry.head;
#endif
}
static ResidentNode cached_node(unsigned id,const NodePort* nodes,NodeLine cache[2048]) {
    const unsigned line=id&2047;
    NodeLine entry=cache[line];
#ifdef NATIONAL_TRACE
    if(entry.tag!=id+1)++frame_node_misses;
#endif
    if(entry.tag!=id+1){entry={id+1,load_node(nodes,id)};cache[line]=entry;}
    return entry.value;
}
struct LaneBest {float d;int id;unsigned index,primary;uint64_t ordinal;};
static void update_lane(const SurfaceWord& p,const ResidentNode& n,unsigned id,unsigned cell,LaneBest& b) {
#pragma HLS INLINE
    const SurfaceWord s=n.surface;
    float dx=p.x-s.x,dy=p.y-s.y,dz=p.z-s.z;
    float d=(dx*dx+dy*dy)+dz*dz;
    bool earlier=b.id>=0 && ((b.index==cell &&
        (n.primary>b.primary || (n.primary==b.primary && n.insertion<b.ordinal)))
#if defined(NATIONAL_OCTANTS) || defined(NATIONAL_CHAIN_WAVE)
        || cell<b.index
#endif
        );
    if((d<b.d || (d==b.d&&earlier)) && ((s.nx*p.nx+s.ny*p.ny)+s.nz*p.nz)>.94f &&
        std::fabs((dx*s.nx+dy*s.ny)+dz*s.nz)<.012f) b={d,int(id),cell,n.primary,n.insertion};
}
#ifdef NATIONAL_NEIGHBORHOOD
#include "neighborhood_matches.hpp"
#endif
static unsigned grouped_matches(const ObservationPort* obs,unsigned size,unsigned initial,const unsigned* buckets,
                                const NodePort* nodes,int* matches,Ordered* first,Ordered* second,uint64_t& visits) {
#ifdef NATIONAL_TRACE
    frame_bucket_misses=0;frame_node_misses=0;frame_bucket_visits=0;
#ifdef NATIONAL_NEIGHBORHOOD
    frame_neighborhood_refills=0;frame_query_batches=0;frame_query_iterations=0;frame_dense_fallbacks=0;
#endif
#endif
    unsigned n=0;
#ifdef NATIONAL_CHAIN_WAVE
    ObservationWindow input;
#endif
    for(unsigned i=0;i<size;++i){
        SurfaceWord p;
#ifdef NATIONAL_CHAIN_WAVE
        if(i%ObservationWindow::length==0)input.sequential(obs,i,ObservationWindow::remaining(i,size));
        const Observation raw=input.at(i);
        if(raw.pad!=valid_unit_normal && raw.pad!=invalid_unit_normal)return UINT32_MAX;
        if(!prepare(raw,p)){matches[i]=-2;continue;}
#else
        if(!prepare(load_observation(obs,i),p)){matches[i]=-2;continue;}
#endif
        matches[i]=-1;
#ifdef NATIONAL_SPATIAL
#ifdef NATIONAL_OCTANTS
        auto k=voxel(p);
        const unsigned spatial=((unsigned(k.z)&127)<<14)|((unsigned(k.y)&127)<<7)|(unsigned(k.x)&127);
        first[n++]={(spatial<<3)|normal_octant(p),i};
#else
        first[n++]={spatial_order(voxel(p)),i};
#endif
#else
        first[n++]={hash(voxel(p)),i};
#endif
    }
    if(!initial)return n;
    radix(first,second,n,0);radix(second,first,n,8);radix(first,second,n,16);
    BucketLine bc[bucket_cache_lines];NodeLine nc[2048];
#pragma HLS BIND_STORAGE variable=bc type=ram_2p impl=bram
#pragma HLS BIND_STORAGE variable=nc type=ram_2p impl=uram
    for(unsigned i=0;i<bucket_cache_lines;++i)bc[i].tag=0;
    for(unsigned i=0;i<2048;++i)nc[i].tag=0;
#ifdef NATIONAL_NEIGHBORHOOD
    Key3 last_group{};
    LocalCandidate neighborhood[neighborhood_limit];
#pragma HLS BIND_STORAGE variable=neighborhood type=ram_2p impl=bram
    unsigned candidate_count=0;
    bool have_neighborhood=false;
#ifdef NATIONAL_CELL_PRUNE
    CellCandidates cells;
#pragma HLS ARRAY_PARTITION variable=cells.count complete
#endif
#ifdef NATIONAL_OCTANTS
    OctantLists octants;
#pragma HLS ARRAY_PARTITION variable=octants.sizes complete
#endif
#endif
    for(unsigned start=0;start<n;){
        SurfaceWord points[4]; LaneBest best[4]; unsigned indices[4];
#pragma HLS ARRAY_PARTITION variable=points complete
#pragma HLS ARRAY_PARTITION variable=indices complete
#pragma HLS ARRAY_PARTITION variable=best complete
        Key3 group{};unsigned used=0;
        unsigned limit=n;
#ifdef NATIONAL_CHAIN_WAVE
        const unsigned base=start&~(ObservationWindow::length-1);
        limit=base+ObservationWindow::remaining(base,n);
        if(start==base)input.ordered(obs,second,base,limit-base);
#endif
        for(;used<4 && start+used<limit;++used){
            Ordered q=second[start+used];
            if(q.group_key()!=second[start].group_key())break;
            SurfaceWord p;
#ifdef NATIONAL_CHAIN_WAVE
            prepare(input.at(start+used),p);
#else
            prepare(load_observation(obs,q.input_index()),p);
#endif
            Key3 k=voxel(p);
            if(!used)group=k;
            else if(k.x!=group.x||k.y!=group.y||k.z!=group.z)break;
            points[used]=p;indices[used]=q.input_index();best[used]={.025f*.025f,-1,27,0,0};
        }
#ifdef NATIONAL_NEIGHBORHOOD
        if(!have_neighborhood || group.x!=last_group.x || group.y!=last_group.y || group.z!=last_group.z){
            candidate_count=load_neighborhood(group,buckets,nodes,bc,nc,neighborhood,visits
#ifdef NATIONAL_OCTANTS
                                              ,octants
#endif
                                              );
            last_group=group;have_neighborhood=true;
#ifdef NATIONAL_CELL_PRUNE
            if(candidate_count<=neighborhood_limit)cells.build(neighborhood,candidate_count);
#endif
        }
        if(candidate_count<=neighborhood_limit){
#ifdef NATIONAL_CELL_PRUNE
            query_pruned(neighborhood,cells,group,points,used,best);
#else
            query_neighborhood(neighborhood,candidate_count,points,used,best
#ifdef NATIONAL_OCTANTS
                               ,octants
#endif
                               );
#endif
        }else{
            // Unbounded overflow retains every chain entry in the frozen traversal.
            for(unsigned lane=0;lane<used;++lane)best[lane].id=nearest(points[lane],buckets,nodes,visits);
        }
#elif defined(NATIONAL_GATHER)
        // Each cell has an independent chain cursor. Gather the 27 heads before
        // touching node DDR, and compare one node from each chain per pass.
        Key3 keys[27];unsigned links[27];LaneBest local[27][4];
#pragma HLS ARRAY_PARTITION variable=local complete dim=2
#ifdef NATIONAL_GROUP_PREFETCH
        prefetch_group_heads(group,buckets,bc,keys,links);
#endif
        for(unsigned ci=0;ci<27;++ci){
#pragma HLS PIPELINE
#ifndef NATIONAL_GROUP_PREFETCH
            Key3 k{group.x+int(ci%3)-1,group.y+int((ci/3)%3)-1,group.z+int(ci/9)-1};
            keys[ci]=k;links[ci]=cached_head(hash(k),buckets,bc);
#endif
            for(unsigned lane=0;lane<4;++lane)local[ci][lane]={.025f*.025f,-1,ci,0,0};
        }
        bool again=true;
        while(again){
            again=false;
            for(unsigned ci=0;ci<27;++ci){
#pragma HLS PIPELINE
                unsigned link=links[ci];if(!link)continue;
                unsigned id=link-1;auto node=cached_node(id,nodes,nc);
                ++visits;links[ci]=node.next;again|=node.next!=0;
                if(!same(keys[ci],node))continue;
                for(unsigned lane=0;lane<4;++lane){
#pragma HLS UNROLL
                    if(lane<used)update_lane(points[lane],node,id,ci,local[ci][lane]);
                }
            }
        }
        // Strict distance reduction in frozen z/y/x order selects the first
        // cell on a tie. Each local result already resolved primary/overflow ties.
        for(unsigned ci=0;ci<27;++ci){
#pragma HLS PIPELINE
            for(unsigned lane=0;lane<4;++lane){
#pragma HLS UNROLL
                if(lane<used && local[ci][lane].d<best[lane].d)best[lane]=local[ci][lane];
            }
        }
#else
        unsigned cell=0;
        for(int z=-1;z<=1;++z)for(int y=-1;y<=1;++y)for(int x=-1;x<=1;++x,++cell){
            Key3 k{group.x+x,group.y+y,group.z+z};
            unsigned link=cached_head(hash(k),buckets,bc);
            while(link){
                unsigned id=link-1;auto node=cached_node(id,nodes,nc);link=node.next;++visits;
                if(!same(k,node))continue;
                for(unsigned lane=0;lane<4;++lane){
#pragma HLS UNROLL
                    if(lane<used)update_lane(points[lane],node,id,cell,best[lane]);
                }
            }
        }
#endif
        for(unsigned lane=0;lane<used;++lane)matches[indices[lane]]=best[lane].id;
        start+=used;
    }
    return n;
}
