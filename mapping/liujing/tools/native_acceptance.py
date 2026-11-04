"""Evaluate metrics only; never record or replay sensor data."""
import argparse,bisect,json,math
from pathlib import Path

def percentile(values,p):
    values=sorted(values)
    return values[min(len(values)-1,math.ceil(p*len(values))-1)] if values else None

def evaluate(metrics,frames,start,end,observations):
    failures=[];missing=[]
    def check(name,value,limit,minimum=False):
        if value is None:missing.append(name)
        elif not math.isfinite(value) or (value<limit if minimum else value>limit):failures.append(name)
        return value
    duration=end-start
    check('duration_600s',duration,600,True)
    # Each completed render sequence is counted once. HTTP polls/old JPEG sends are irrelevant.
    seen=set();fresh=[]
    for f in frames:
        seq=int(f['sequence']);t=float(f['ready_mono'])
        if start<=t<=end and seq not in seen:seen.add(seq);fresh.append(f)
    if any(str(f.get('validation',False)).lower()=='true' for f in fresh):failures.append('comparison_interrupts_mobile_acceptance')
    times=sorted(float(f['ready_mono']) for f in fresh)
    windows=[start]+[t+1e-7 for t in times if t+10+1e-7<=end]
    rates=[(bisect.bisect_left(times,t+10)-bisect.bisect_left(times,t))/10 for t in windows if t+10<=end]
    result={'duration_s':duration,'unique_new_frames':len(fresh),
        'minimum_10s_new_fps':check('new_fps',min(rates) if rates else None,10,True),
        'view_latency_p95_ms':check('view_latency',percentile([float(f['view_latency_ms']) for f in fresh if 'view_latency_ms'in f],.95),200),
        'pose_age_max_s':check('follow_pose_age',max([float(f['pose_age_s']) for f in fresh],default=None),.5)}
    rows=[r for r in metrics if start<=float(r['mono'])<=end]
    ages=[float(r['map_age_s']) for r in rows if float(r.get('map_age_s',-1))>=0]
    ages += [float(f['map_age_s']) for f in fresh if 'map_age_s' in f]
    if rows and (float(rows[0]['mono'])>start+1.5 or float(rows[-1]['mono'])<end-1.5 or any(float(b['mono'])-float(a['mono'])>1.5 for a,b in zip(rows,rows[1:]))):missing.append('continuous_metrics_coverage')
    result['map_age_p95_s']=check('map_age_p95',percentile(ages,.95),1)
    result['map_age_max_s']=check('map_age_max',max(ages,default=None),2)
    if len(rows)>1:
        dt=float(rows[-1]['mono'])-float(rows[0]['mono'])
        result['fusion_hz']=check('fusion_2hz',(float(rows[-1].get('fused',0))-float(rows[0].get('fused',0)))/dt,2,True)
        def delta(k):return float(rows[-1].get(k,0))-float(rows[0].get(k,0))
        result['pose_input_ratio']=check('localization_ratio',delta('raw_pose_count')/delta('lidar_input') if delta('lidar_input') else None,.9,True)
        if rows[-1].get('backend')!='pl_opaque_v1' or delta('pl_completed_jobs')<=0:failures.append('actual_PL_rendering')
        if any(r.get('failure') for r in rows):failures.append('runtime_fault')
    else:missing.append('metrics_coverage')
    for key,limit in [('static_drift_m',.05),('return_origin_error_m',.15)]:result[key]=check(key,observations.get(key),limit)
    for key in ['actual_mobile_600s','calibration_verified','lidar_input_accounting_verified','recognizable_surfaces','no_growing_ghosts']:
        if key not in observations:missing.append(key)
        elif observations[key] is not True:failures.append(key)
    dimensions=observations.get('independent_dimensions',[])
    if not dimensions:missing.append('independent_dimensions')
    for i,d in enumerate(dimensions):
        if not all(math.isfinite(d[k]) and d[k]>0 for k in ('estimated_m','measured_m')) or abs(d['estimated_m']-d['measured_m'])>max(.05,.03*d['measured_m']):failures.append('dimension_'+str(i))
    if not observations.get('evidence_note'):missing.append('observation_evidence_note')
    result.update(status='FAIL' if failures else 'INCOMPLETE' if missing else 'PASS',failures=failures,missing=missing)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--start-mono',type=float,required=True);p.add_argument('--end-mono',type=float,required=True);p.add_argument('--observations');a=p.parse_args()
    root=Path(a.run);read=lambda name:[json.loads(x) for x in (root/name).read_text().splitlines() if x]
    result=evaluate(read('metrics.jsonl'),read('frames.jsonl'),a.start_mono,a.end_mono,json.loads(Path(a.observations).read_text()) if a.observations else {})
    (root/'acceptance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));return 0 if result['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
