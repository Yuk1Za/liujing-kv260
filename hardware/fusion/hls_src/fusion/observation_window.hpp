#pragma once
// A 128-observation window separates DDR reads from dependent fusion loops.
// Sequential input uses one contiguous range; sorted phases issue independent
// addresses from the on-chip order table. Only the last window can be short.
struct ObservationWindow {
    static constexpr unsigned length=128;
    FrameBeat words[2*length];
    void sequential(const FrameBeat* port,unsigned start,unsigned count){
        for(unsigned t=0;t<2*count;++t){
#pragma HLS PIPELINE II=1
            words[t]=port[2*start+t];
        }
    }
    void ordered(const FrameBeat* port,const Ordered* order,unsigned start,unsigned count){
        for(unsigned t=0;t<2*count;++t){
#pragma HLS PIPELINE II=1
            const unsigned index=order[start+(t>>1)].input_index();
            words[t]=port[2*index+(t&1)];
        }
    }
    Observation at(unsigned position)const{
        const unsigned base=2*(position&(length-1));
        const auto b=words[base+1];
        const auto p=unpack_surface(words[base],b);
        return {p.x,p.y,p.z,p.nx,p.ny,p.nz,p.rgb,b.words[3]};
    }
    static unsigned remaining(unsigned start,unsigned size){
        return size-start<length?size-start:length;
    }
};
