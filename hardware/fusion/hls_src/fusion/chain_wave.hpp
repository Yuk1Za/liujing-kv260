#pragma once

// Read the independent heads of the 27 chains together. No next pointer is
// consumed until this wave's AXI reads have completed. The map is immutable
// throughout association, including when multiple cells share one hash bucket.
static unsigned load_chain_wave(Key3 center,const unsigned* buckets,const FrameBeat* nodes,
                                BucketLine* bc,NodeLine* nc,LocalCandidate* candidates,uint64_t& visits){
    Key3 keys[27];unsigned links[27],active[27];
    prefetch_group_heads(center,buckets,bc,keys,links);
    unsigned live=0,count=0;
    for(unsigned cell=0;cell<27;++cell)if(links[cell])active[live++]=cell;
    while(live){
        unsigned missing[27],missing_id[27],misses=0;
        ResidentNode records[27];
        for(unsigned i=0;i<live;++i){
#pragma HLS PIPELINE II=1
            const unsigned id=links[active[i]]-1;
            const NodeLine line=nc[id&2047];
            if(line.tag==id+1)records[i]=line.value;
            else{missing[misses]=i;missing_id[misses++]=id;}
        }
        FrameBeat words[27][4];
#pragma HLS ARRAY_PARTITION variable=words complete dim=2
        for(unsigned t=0;t<4*misses;++t){
#pragma HLS PIPELINE II=1
            words[t>>2][t&3]=nodes[4*missing_id[t>>2]+(t&3)];
        }
        for(unsigned m=0;m<misses;++m){
#pragma HLS PIPELINE II=1
            const auto a=words[m][0],b=words[m][1],c=words[m][2],d=words[m][3];
            const ResidentNode node={unpack_surface(a,b),int32_t(c.words[0]),int32_t(c.words[1]),int32_t(c.words[2]),
                                     c.words[3],d.words[0],d.words[1],uint64_t(d.words[2])|(uint64_t(d.words[3])<<32)};
            const unsigned id=missing_id[m];
            nc[id&2047]={id+1,node};records[missing[m]]=node;
        }
#ifdef NATIONAL_TRACE
        frame_node_misses+=misses;
#endif
        visits+=live;
        unsigned next=0;
        for(unsigned i=0;i<live;++i){
            const unsigned cell=active[i],id=links[cell]-1;
            const auto node=records[i];
            links[cell]=node.next;
            if(node.next)active[next++]=cell;
            if(!same(keys[cell],node))continue;
            if(count==neighborhood_limit){
#ifdef NATIONAL_TRACE
                ++frame_dense_fallbacks;
#endif
                return neighborhood_limit+1;
            }
            candidates[count++]={node.surface,id,cell,node.primary,node.insertion};
        }
        live=next;
    }
    return count;
}
