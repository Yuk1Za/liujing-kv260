export function backendLabel(value){return ({cpu_overflow_reference:'CPU · 可变长面元融合',cpu_batched_fusion_v3:'CPU · 批量面元融合',pl_association_v1:'PL · 邻域查找与表面匹配',pl_variable_association_v1:'PL · 可变长邻域查找与表面匹配',pl_cached_variable_v2:'PL · 可变长邻域查找与表面关联',pl_resident_fusion_v4:'PL · 邻域关联与索引维护'})[value]||value||'等待数据';}
export const plScope='PL 计算：邻域查找与表面关联；PS 计算：地图更新与版本提交。';

// A saved map and an unclassified connection must never count as live updates.
export function mapTelemetry({state,session,now,lastStatusAt,lastMapAt,events,lastReport,snapshot}){
 const alive=lastStatusAt>0&&now-lastStatusAt<2500;
 const known=!!state.session&&state.session===session;
 const live=known&&!snapshot;
 const rate=state.fusion_commit_hz_10s;
 return {
  status_mode:snapshot?'snapshot':!known?'unknown':alive?'connected':'disconnected',
  display_hz_10s:live?events.filter(e=>e.at>now-10000).length/10:0,
  new_versions:live?events.filter(e=>e.at>lastReport):[],
  fusion_commit_hz_10s:live&&alive&&now-lastMapAt<=1500&&typeof rate==='number'&&Number.isFinite(rate)&&rate>=0?rate:null,
  live_source_age:live,
 };
}
