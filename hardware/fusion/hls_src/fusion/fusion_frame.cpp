#include "fusion_frame.hpp"
#include "merge_surface.hpp"
#include "frame_memory.hpp"

struct Key3 { int x,y,z; };
static Key3 voxel(const SurfaceWord& p) {
    return {int(std::floor(p.x/.02f)),int(std::floor(p.y/.02f)),int(std::floor(p.z/.02f))};
}
static unsigned hash(Key3 k) {
#ifdef NATIONAL_TILE_HEADS
    // Floor division by four over signed voxel coordinates, including negatives.
    unsigned x=unsigned(k.x),y=unsigned(k.y),z=unsigned(k.z);
    unsigned h=(x>>2)*73856093u ^ (y>>2)*19349663u ^ (z>>2)*83492791u;
    unsigned row=(h^(h>>15)^(h>>23))&32767;
    return (row<<6)|((z&3)<<4)|((y&3)<<2)|(x&3);
#else
    unsigned h=unsigned(k.x)*73856093u ^ unsigned(k.y)*19349663u ^ unsigned(k.z)*83492791u;
    return (h^(h>>21)) & ((1u<<21)-1);
#endif
}
static bool same(Key3 k,const ResidentNode& n) { return k.x==n.x && k.y==n.y && k.z==n.z; }
static bool prepare(Observation raw, SurfaceWord& p) {
#ifdef NATIONAL_HOST_NORMALS
    if(raw.pad!=valid_unit_normal)return false;
    p={raw.x,raw.y,raw.z,raw.nx,raw.ny,raw.nz,raw.rgb&0xffffff,1};
#else
    float norm2=(raw.nx*raw.nx+raw.ny*raw.ny)+raw.nz*raw.nz;
    if(!(std::fabs(raw.x)<=10000 && std::fabs(raw.y)<=10000 && std::fabs(raw.z)<=10000 &&
         std::isfinite(norm2) && norm2>=.25f)) return false;
    float n=std::sqrt(norm2);
    p={raw.x,raw.y,raw.z,raw.nx/n,raw.ny/n,raw.nz/n,raw.rgb&0xffffff,1};
#endif
    return true;
}
// Entries are linked in reverse insertion order. Resolve equal distances within
// a voxel by frozen primary/overflow order and original insertion ordinal.
static int nearest(const SurfaceWord& p, const unsigned* buckets,const NodePort* nodes,uint64_t& visits) {
    Key3 k=voxel(p);
    float best=.025f*.025f;
    int match=-1;
    for(int z=-1;z<=1;++z) for(int y=-1;y<=1;++y) for(int x=-1;x<=1;++x) {
        Key3 cell{k.x+x,k.y+y,k.z+z};
        int local=-1;
        unsigned primary=0;
        uint64_t ordinal=0;
        for(unsigned link=buckets[hash(cell)];link;) {
            unsigned id=link-1;
            ResidentNode n=load_node(nodes,id);link=n.next;
            ++visits;
            if(!same(cell,n))continue;
            SurfaceWord s=n.surface;
            float dx=p.x-s.x,dy=p.y-s.y,dz=p.z-s.z;
            float d=(dx*dx+dy*dy)+dz*dz;
            bool earlier=local>=0 && (n.primary>primary || (n.primary==primary && n.insertion<ordinal));
            if((d<best || (d==best && earlier)) &&
               ((s.nx*p.nx+s.ny*p.ny)+s.nz*p.nz)>.94f &&
               std::fabs((dx*s.nx+dy*s.ny)+dz*s.nz)<.012f) {
                best=d;local=id;primary=n.primary;ordinal=n.insertion;
            }
        }
        if(local>=0) match=local;
    }
    return match;
}
static void unlink_node(unsigned* buckets,NodePort* nodes,unsigned id,const ResidentNode& n) {
    unsigned h=hash({n.x,n.y,n.z});
    unsigned link=buckets[h],previous=0;
    while(link!=id+1) { previous=link;link=load_next(nodes,link-1); }
    if(previous)store_next(nodes,previous-1,n.next);
    else buckets[h]=n.next;
}
static void insert_node(unsigned* buckets,const NodePort* nodes,unsigned id,Key3 k,uint64_t order,ResidentNode& n) {
    unsigned h=hash(k),used=0;
    for(unsigned link=buckets[h];link && used<4;) {
        ResidentNode s=load_node(nodes,link-1);link=s.next;
        used+=same(k,s) && s.primary;
    }
    n.x=k.x;n.y=k.y;n.z=k.z;n.primary=used<4;n.insertion=order;
    n.next=buckets[h];buckets[h]=id+1;
}
// Sort by old stable ID, preserving input order for each ID and all unmatched.
#if defined(NATIONAL_GROUPED) || defined(NATIONAL_COMPACT)
struct Ordered {
    uint64_t word;
    Ordered()=default;
    Ordered(unsigned key,unsigned index):word((uint64_t(key)<<32)|index){}
    unsigned group_key()const{return unsigned(word>>32);}
    unsigned input_index()const{return unsigned(word);}
};
#else
struct Ordered {
    unsigned key,index;
    unsigned group_key()const{return key;}
    unsigned input_index()const{return index;}
};
#endif
#ifdef NATIONAL_CHAIN_WAVE
#include "observation_window.hpp"
#endif
static void radix(const Ordered* in,Ordered* out,unsigned n,unsigned shift) {
    unsigned offsets[256]={};
    for(unsigned i=0;i<n;++i) ++offsets[(in[i].group_key()>>shift)&255];
    unsigned sum=0;
    for(unsigned b=0;b<256;++b){unsigned v=offsets[b];offsets[b]=sum;sum+=v;}
    for(unsigned i=0;i<n;++i){Ordered p=in[i];out[offsets[(p.group_key()>>shift)&255]++]=p;}
}
#ifdef NATIONAL_GROUPED
#include "grouped_matches.hpp"
#endif
extern "C" void fusion_frame(const ObservationPort* observations,unsigned* buckets,
    NodePort* nodes,unsigned* touched,DeltaPort* delta,int* matches,
    unsigned size,unsigned initial_count,unsigned capacity,unsigned job,ResultPort* result
#ifdef NATIONAL_CYCLE_COUNTERS
    ,const volatile uint32_t& fabric_cycle
#endif
    ) {
#ifdef NATIONAL_CYCLE_COUNTERS
#pragma HLS INTERFACE ap_none port=fabric_cycle
    const uint32_t cycle_start=fabric_cycle;
#endif
#ifdef NATIONAL_NARROW
#ifdef NATIONAL_CHAIN_WAVE
#pragma HLS INTERFACE m_axi port=observations offset=slave bundle=mapbus max_widen_bitwidth=128 num_read_outstanding=32 max_read_burst_length=32
#else
#pragma HLS INTERFACE m_axi port=observations offset=slave bundle=mapbus max_widen_bitwidth=128
#endif
#pragma HLS INTERFACE m_axi port=buckets offset=slave bundle=iobus max_widen_bitwidth=32
#ifdef NATIONAL_CHAIN_WAVE
#pragma HLS INTERFACE m_axi port=nodes offset=slave bundle=mapbus max_widen_bitwidth=128 num_read_outstanding=32 max_read_burst_length=32
#else
#pragma HLS INTERFACE m_axi port=nodes offset=slave bundle=mapbus max_widen_bitwidth=128
#endif
#pragma HLS INTERFACE m_axi port=touched offset=slave bundle=iobus max_widen_bitwidth=32
#pragma HLS INTERFACE m_axi port=delta offset=slave bundle=mapbus max_widen_bitwidth=128
#pragma HLS INTERFACE m_axi port=matches offset=slave bundle=iobus max_widen_bitwidth=32
#pragma HLS INTERFACE m_axi port=result offset=slave bundle=mapbus max_widen_bitwidth=128
#elif defined(NATIONAL_TWO_PORT)
#pragma HLS INTERFACE m_axi port=observations offset=slave bundle=iobus
#pragma HLS INTERFACE m_axi port=buckets offset=slave bundle=iobus
#pragma HLS INTERFACE m_axi port=nodes offset=slave bundle=mapbus
#pragma HLS INTERFACE m_axi port=touched offset=slave bundle=iobus
#pragma HLS INTERFACE m_axi port=delta offset=slave bundle=iobus
#pragma HLS INTERFACE m_axi port=matches offset=slave bundle=iobus
#pragma HLS INTERFACE m_axi port=result offset=slave bundle=iobus
#else
#pragma HLS INTERFACE m_axi port=observations offset=slave bundle=inbus
#pragma HLS INTERFACE m_axi port=buckets offset=slave bundle=indexbus
#pragma HLS INTERFACE m_axi port=nodes offset=slave bundle=mapbus
#pragma HLS INTERFACE m_axi port=touched offset=slave bundle=outbus
#pragma HLS INTERFACE m_axi port=delta offset=slave bundle=outbus
#pragma HLS INTERFACE m_axi port=matches offset=slave bundle=outbus
#pragma HLS INTERFACE m_axi port=result offset=slave bundle=outbus
#endif
#pragma HLS INTERFACE s_axilite port=observations bundle=control
#pragma HLS INTERFACE s_axilite port=buckets bundle=control
#pragma HLS INTERFACE s_axilite port=nodes bundle=control
#pragma HLS INTERFACE s_axilite port=touched bundle=control
#pragma HLS INTERFACE s_axilite port=delta bundle=control
#pragma HLS INTERFACE s_axilite port=matches bundle=control
#pragma HLS INTERFACE s_axilite port=size bundle=control
#pragma HLS INTERFACE s_axilite port=initial_count bundle=control
#pragma HLS INTERFACE s_axilite port=capacity bundle=control
#pragma HLS INTERFACE s_axilite port=job bundle=control
#pragma HLS INTERFACE s_axilite port=result bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
#ifdef NATIONAL_COMPACT
#pragma HLS ALLOCATION operation instances=fdiv limit=1
#endif
    FusionResult r{};
    if(!job || size>64000 || capacity>2000000 || initial_count>capacity){store_result(result,r);return;}
    r.job=job;r.count=initial_count;
    Ordered first[64000],second[64000];
#pragma HLS BIND_STORAGE variable=first type=ram_2p impl=uram
#pragma HLS BIND_STORAGE variable=second type=ram_2p impl=uram
#ifdef NATIONAL_GROUPED
#ifdef NATIONAL_TILE_HEADS
    int local_matches[64000];
#pragma HLS BIND_STORAGE variable=local_matches type=ram_2p impl=uram
#else
    int* local_matches=matches;
#endif
    const unsigned prepared=grouped_matches(observations,size,initial_count,buckets,nodes,local_matches,first,second,r.visits);
#ifdef NATIONAL_CHAIN_WAVE
    if(prepared==UINT32_MAX){store_result(result,FusionResult{});return;}
#else
    (void)prepared;
#endif
    unsigned n=0;
    for(unsigned i=0;i<size;++i){
        int id=local_matches[i];if(id!=-2)first[n++]={id<0?initial_count:unsigned(id),i};
    }
#ifdef NATIONAL_TILE_HEADS
    for(unsigned i=0;i<size;++i){
#pragma HLS PIPELINE II=1
        matches[i]=local_matches[i];
    }
#endif
#else
    unsigned n=0;
    for(unsigned i=0;i<size;++i) {
        SurfaceWord p;
        if(!prepare(load_observation(observations,i),p)){matches[i]=-2;continue;}
        int id=initial_count?nearest(p,buckets,nodes,r.visits):-1;
        matches[i]=id;first[n++]={id<0?initial_count:unsigned(id),i};
    }
#endif
#ifdef NATIONAL_CYCLE_COUNTERS
    const uint32_t cycle_matched=fabric_cycle;
#endif
    radix(first,second,n,0);radix(second,first,n,8);radix(first,second,n,16);
#ifdef NATIONAL_CYCLE_COUNTERS
    const uint32_t cycle_ordered=fabric_cycle;
#endif
#ifdef NATIONAL_CACHED_COMMIT
    // Stable radix order makes each committed match ID a single run. Keep
    // it in registers across observations and publish it before any other ID
    // or unmatched lookup can traverse the chain.
    int cached_id=-1;
    ResidentNode cached_node{};
#endif
#ifdef NATIONAL_CHAIN_WAVE
    ObservationWindow commit_input;
#endif
    for(unsigned i=0;i<n;++i){
        Ordered query=second[i];
        SurfaceWord p;
#ifdef NATIONAL_CHAIN_WAVE
        if(i%ObservationWindow::length==0)commit_input.ordered(observations,second,i,ObservationWindow::remaining(i,n));
        prepare(commit_input.at(i),p);
#else
        prepare(load_observation(observations,query.input_index()),p);
#endif
#ifdef NATIONAL_CACHED_COMMIT
        if(cached_id>=0 && unsigned(cached_id)!=query.group_key()){
            store_node(nodes,cached_id,cached_node);
            cached_id=-1;
        }
#endif
        int id=query.group_key()==initial_count?nearest(p,buckets,nodes,r.visits):int(query.group_key());
        ResidentNode node{};
        uint64_t ordinal=(uint64_t(job)<<32)|i;
        if(id<0) {
            if(r.count==capacity){++r.dropped;continue;}
            id=r.count++;node.surface=p;
            insert_node(buckets,nodes,id,voxel(p),ordinal,node);++r.added;
        } else {
#ifdef NATIONAL_CACHED_COMMIT
            node=id==cached_id?cached_node:load_node(nodes,id);
#else
            node=load_node(nodes,id);
#endif
            node.surface=merge_surface(node.surface,p);
            Key3 k=voxel(node.surface);
            if(!same(k,node)) {
                unlink_node(buckets,nodes,id,node);
                insert_node(buckets,nodes,id,k,ordinal,node);
            }
            ++r.merged;
        }
        if(node.dirty_job!=job){touched[r.dirty++]=id;node.dirty_job=job;}
#ifdef NATIONAL_CACHED_COMMIT
        if(query.group_key()<initial_count){cached_id=id;cached_node=node;}
        else store_node(nodes,id,node);
#else
        store_node(nodes,id,node);
#endif
    }
#ifdef NATIONAL_CACHED_COMMIT
    if(cached_id>=0)store_node(nodes,cached_id,cached_node);
#endif
    // PS sorts the bounded dirty list by stable ID for LJCD publication.
#ifdef NATIONAL_CYCLE_COUNTERS
    const uint32_t cycle_committed=fabric_cycle;
#endif
    for(unsigned i=0;i<r.dirty;++i){
#ifdef NATIONAL_COMPACT
#pragma HLS PIPELINE off
#endif
        unsigned id=touched[i];
        store_delta(delta,i,{id,load_node(nodes,id).surface});
    }
    store_result(result,r);
#ifdef NATIONAL_CYCLE_COUNTERS
    const uint32_t cycle_delta=fabric_cycle;
    result[2]={{cycle_matched-cycle_start,cycle_ordered-cycle_matched,cycle_committed-cycle_ordered,cycle_delta-cycle_committed}};
    result[3]={{cycle_start,cycle_delta,job,0x4c4a504d}};
#endif
}
