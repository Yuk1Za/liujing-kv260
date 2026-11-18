#pragma once
#include "fusion_frame.hpp"
#include <cstring>

#ifdef NATIONAL_NARROW
// All wide data uses aligned 128-bit transfers. The index port carries only
// uint32_t words. Both match the HP bus width without a 512-bit crossbar.
static float as_float(uint32_t bits){float value;std::memcpy(&value,&bits,4);return value;}
static uint32_t as_word(float value){uint32_t bits;std::memcpy(&bits,&value,4);return bits;}
static SurfaceWord unpack_surface(FrameBeat a,FrameBeat b){
    return {as_float(a.words[0]),as_float(a.words[1]),as_float(a.words[2]),as_float(a.words[3]),
            as_float(b.words[0]),as_float(b.words[1]),b.words[2],as_float(b.words[3])};
}
#ifdef NATIONAL_RECORD_BURSTS
// Each record starts at a 32- or 64-byte aligned address. Explicit loop
// bounds give HLS one contiguous transfer before unpacking or after packing.
template<unsigned N>
static void read_beats(const FrameBeat* port,unsigned base,FrameBeat (&words)[N]){
    for(unsigned k=0;k<N;++k){
#pragma HLS PIPELINE II=1
        words[k]=port[base+k];
    }
}
template<unsigned N>
static void write_beats(FrameBeat* port,unsigned base,const FrameBeat (&words)[N]){
    for(unsigned k=0;k<N;++k){
#pragma HLS PIPELINE II=1
        port[base+k]=words[k];
    }
}
#endif
static Observation load_observation(const ObservationPort* p,unsigned i){
#ifdef NATIONAL_RECORD_BURSTS
    FrameBeat words[2];read_beats(p,2*i,words);
    auto a=words[0],b=words[1];
#else
    FrameBeat a=p[2*i],b=p[2*i+1];
#endif
    auto v=unpack_surface(a,b);
    return {v.x,v.y,v.z,v.nx,v.ny,v.nz,v.rgb,b.words[3]};
}
static ResidentNode load_node(const NodePort* p,unsigned i){
#ifdef NATIONAL_RECORD_BURSTS
    FrameBeat words[4];read_beats(p,4*i,words);
    auto a=words[0],b=words[1],c=words[2],d=words[3];
#else
    FrameBeat a=p[4*i],b=p[4*i+1],c=p[4*i+2],d=p[4*i+3];
#endif
    return {unpack_surface(a,b),int32_t(c.words[0]),int32_t(c.words[1]),int32_t(c.words[2]),
            c.words[3],d.words[0],d.words[1],uint64_t(d.words[2])|(uint64_t(d.words[3])<<32)};
}
static unsigned load_next(const NodePort* p,unsigned i){return p[4*i+2].words[3];}
static void store_next(NodePort* p,unsigned i,unsigned link){
    auto word=p[4*i+2];word.words[3]=link;p[4*i+2]=word;
}
static void store_node(NodePort* p,unsigned i,const ResidentNode& n){
    const auto s=n.surface;
#ifdef NATIONAL_RECORD_BURSTS
    const FrameBeat words[4]={
        {{as_word(s.x),as_word(s.y),as_word(s.z),as_word(s.nx)}},
        {{as_word(s.ny),as_word(s.nz),s.rgb,as_word(s.weight)}},
        {{uint32_t(n.x),uint32_t(n.y),uint32_t(n.z),n.next}},
        {{n.primary,n.dirty_job,uint32_t(n.insertion),uint32_t(n.insertion>>32)}}};
    write_beats(p,4*i,words);
#else
    p[4*i]={{as_word(s.x),as_word(s.y),as_word(s.z),as_word(s.nx)}};
    p[4*i+1]={{as_word(s.ny),as_word(s.nz),s.rgb,as_word(s.weight)}};
    p[4*i+2]={{uint32_t(n.x),uint32_t(n.y),uint32_t(n.z),n.next}};
    p[4*i+3]={{n.primary,n.dirty_job,uint32_t(n.insertion),uint32_t(n.insertion>>32)}};
#endif
}
static void store_delta(DeltaPort* p,unsigned i,const UpdateResult& d){
    const auto s=d.value;
#ifdef NATIONAL_RECORD_BURSTS
    const FrameBeat words[4]={
        {{d.id,as_word(s.x),as_word(s.y),as_word(s.z)}},
        {{as_word(s.nx),as_word(s.ny),as_word(s.nz),s.rgb}},
        {{as_word(s.weight),0,0,0}},{{0,0,0,0}}};
    write_beats(p,4*i,words);
#else
    p[4*i]={{d.id,as_word(s.x),as_word(s.y),as_word(s.z)}};
    p[4*i+1]={{as_word(s.nx),as_word(s.ny),as_word(s.nz),s.rgb}};
    p[4*i+2]={{as_word(s.weight),0,0,0}};
    p[4*i+3]={{0,0,0,0}};
#endif
}
static void store_result(ResultPort* p,const FusionResult& r){
    p[0]={{r.job,r.count,r.dirty,r.added}};
    p[1]={{r.merged,r.dropped,uint32_t(r.visits),uint32_t(r.visits>>32)}};
}
#else
static Observation load_observation(const ObservationPort* p,unsigned i){return p[i];}
static ResidentNode load_node(const NodePort* p,unsigned i){return p[i];}
static unsigned load_next(const NodePort* p,unsigned i){return p[i].next;}
static void store_next(NodePort* p,unsigned i,unsigned link){p[i].next=link;}
static void store_node(NodePort* p,unsigned i,const ResidentNode& n){p[i]=n;}
static void store_delta(DeltaPort* p,unsigned i,const UpdateResult& d){p[i]=d;}
static void store_result(ResultPort* p,const FusionResult& r){*p=r;}
#endif
