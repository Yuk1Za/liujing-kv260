#pragma once
// Included inside lj2, after FusionCell. All reads use the committed cache.
#if defined(__aarch64__) && !defined(LJ3_SCALAR)
inline void associate_query_lanes(std::vector<AssociationQuery>&q,
 const std::vector<AssociationGroup>&groups,const std::vector<const FusionCell*>&cells,
 int threads,uint64_t&candidates,double&cpu_ms){
 auto start=thread_cpu_millis();
 std::vector<uint32_t>offset(groups.size()+1,0);
 for(const auto&query:q)++offset[query.group+1];
 std::partial_sum(offset.begin(),offset.end(),offset.begin());
 auto cursor=offset;std::vector<uint32_t>indices(q.size());
 for(uint32_t i=0;i<q.size();++i)indices[cursor[q[i].group]++]=i;
 cpu_ms=thread_cpu_millis()-start;uint64_t visits=0;double workers=0;
 #pragma omp parallel num_threads(threads) reduction(+:visits,workers) if(q.size()>1000&&threads>1)
 {auto worker_start=thread_cpu_millis();
 #pragma omp for schedule(static)
 for(size_t gi=0;gi<groups.size();++gi){
  const auto&g=groups[gi];uint64_t count=0;
  for(uint32_t j=0;j<g.count;++j)count+=cells[g.offset+j]->size();
  visits+=count*(offset[gi+1]-offset[gi]);
  for(uint32_t first=offset[gi];first<offset[gi+1];first+=4){
   auto n=std::min(uint32_t(4),offset[gi+1]-first);
   float x[4]{},y[4]{},z[4]{},nx[4]{},ny[4]{},nz[4]{};
   for(uint32_t lane=0;lane<n;++lane){const auto&p=q[indices[first+lane]].value;
    x[lane]=p.x;y[lane]=p.y;z[lane]=p.z;nx[lane]=p.nx;ny[lane]=p.ny;nz[lane]=p.nz;}
   const auto px=vld1q_f32(x),py=vld1q_f32(y),pz=vld1q_f32(z);
   const auto pnx=vld1q_f32(nx),pny=vld1q_f32(ny),pnz=vld1q_f32(nz);
   auto distance=vdupq_n_f32(.025f*.025f);auto best=vdupq_n_s32(-1);
   auto compare=[&](const auto&b,size_t k,uint32_t id){
    auto dx=vsubq_f32(px,vdupq_n_f32(b.x[k])),dy=vsubq_f32(py,vdupq_n_f32(b.y[k])),dz=vsubq_f32(pz,vdupq_n_f32(b.z[k]));
    auto sx=vdupq_n_f32(b.nx[k]),sy=vdupq_n_f32(b.ny[k]),sz=vdupq_n_f32(b.nz[k]);
    auto d=vaddq_f32(vaddq_f32(vmulq_f32(dx,dx),vmulq_f32(dy,dy)),vmulq_f32(dz,dz));
    auto dot=vaddq_f32(vaddq_f32(vmulq_f32(sx,pnx),vmulq_f32(sy,pny)),vmulq_f32(sz,pnz));
    auto plane=vabsq_f32(vaddq_f32(vaddq_f32(vmulq_f32(dx,sx),vmulq_f32(dy,sy)),vmulq_f32(dz,sz)));
    auto valid=vandq_u32(vandq_u32(vcltq_f32(d,distance),vcgtq_f32(dot,vdupq_n_f32(.94f))),vcltq_f32(plane,vdupq_n_f32(.012f)));
    distance=vbslq_f32(valid,d,distance);best=vbslq_s32(valid,vdupq_n_s32(int32_t(id)),best);
   };
   // Strict order, including ties: z/y/x cells, inline slots, overflow IDs.
   for(uint32_t j=0;j<g.count;++j){const auto&b=*cells[g.offset+j];
    for(uint32_t k=0;k<b.n;++k)compare(b,k,b.ids[k]);
    for(size_t k=0;k<b.extra.size();++k)compare(b.extra_values[k/4],k%4,b.extra[k]);
   }
   int32_t ids[4];vst1q_s32(ids,best);
   for(uint32_t lane=0;lane<n;++lane)q[indices[first+lane]].match=ids[lane];
  }
 }
 workers+=thread_cpu_millis()-worker_start;
 }
 candidates=visits;cpu_ms+=workers;
}
#endif
