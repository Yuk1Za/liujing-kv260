"""Summarize measured RAM-only checks; never certifies live/mobile acceptance."""
import argparse,bisect,json,math,statistics
from pathlib import Path

def stats(values):
    v=sorted(values)
    if not v:return None
    def p(q):
        x=(len(v)-1)*q;i=int(x);return v[i]+(v[min(i+1,len(v)-1)]-v[i])*(x-i)
    return dict(count=len(v),median=statistics.median(v),p95=p(.95),p99=p(.99),maximum=v[-1])

def read_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]

def summarize(root):
    result={'scope':'synthetic RAM checks; no real sensors/localization or mobile acceptance','runs':{}}
    for path in sorted(root.glob('board_*/comparison.jsonl')):
        rows=read_lines(path);normal=[r for r in rows if r.get('output_mode')=='color_only']
        run={'comparisons':sum('mismatched_pixels' in r for r in rows),'mismatched_pixels':sum(r.get('mismatched_pixels',0) for r in rows),'cases':{}}
        for t in sorted({r['test'] for r in normal}):
            batch=[r for r in normal if r['test']==t]
            run['cases'][str(t)]={k:stats([r[k] for r in batch]) for k in ('cpu_ms','pl_call_ms','cpu_process_ms','pl_process_ms','pl_kernel_cycles')}
            run['cases'][str(t)]['candidates']=batch[0]['candidates']
            run['cases'][str(t)]['median_call_speed_ratio']=run['cases'][str(t)]['cpu_ms']['median']/run['cases'][str(t)]['pl_call_ms']['median']
            run['cases'][str(t)]['median_cpu_time_reduction']=1-run['cases'][str(t)]['pl_process_ms']['median']/run['cases'][str(t)]['cpu_process_ms']['median']
        c16=json.loads((path.parent/'c16_check.json').read_text());samples=c16.get('joint',[])
        run['c16']={'scope':c16['scope'],'failure':c16['failure'],'joint_samples':len(samples),'preflight_kernel_ms':stats([r['kernel_call_ms'] for r in c16['preflight']]),'joint_kernel_ms':stats([r['kernel_call_ms'] for r in samples if 'kernel_call_ms' in r]),'harness_call_gap_ms':stats([1000*(b['mono']-a['mono']) for a,b in zip(samples,samples[1:]) if 'mono' in a and 'mono' in b])}
        if len(samples)>1 and all('mono' in r for r in samples):run['c16']['harness_rate_hz']=(len(samples)-1)/(samples[-1]['mono']-samples[0]['mono'])
        counters=[r for r in rows if r.get('counter_sample')]
        if counters:run['axi_counters']={str(r['test']):dict(zip(r['metric_order'].split(','),r['values'])) for r in counters}
        pipeline=path.parent/'pipeline.jsonl'
        if pipeline.exists():
            entries=read_lines(pipeline);duration=entries[0]['seconds'];frames=[r for r in entries if 'frame' in r];steady=[r for r in frames if 5<=r['completed_s']<=duration]
            times=sorted(r['completed_s'] for r in frames if r['new_pixels'])
            starts={5.,duration-10}
            for t in times:
                starts.update(x for x in (t,t+1e-8,t-10,t-10+1e-8) if 5<=x<=duration-10)
            windows=[(bisect.bisect_right(times,s+10)-bisect.bisect_right(times,s))/10 for s in starts if 5<=s<=duration-10]
            run['pipeline']={'scope':entries[0]['scope'],'summary':entries[-1],'steady_interval_s':[5,duration],'min_rolling_10s_changed_image_fps':min(windows) if windows else None,'steady_frames':len(steady)}
            for k in ('latency_ms','map_age_ms','prepare_ms','render_ms','encode_ms','visible','candidates'):run['pipeline'][k]=stats([r[k] for r in steady])
            updates=[r for r in entries if 'fusion_ms' in r];run['pipeline']['fusion_ms']=stats([r['fusion_ms'] for r in updates if r['map_commit_s']>=5])
            if len(updates)>1:run['pipeline']['map_commit_rate_hz']=(len(updates)-1)/(updates[-1]['map_commit_s']-updates[0]['map_commit_s'])
        result['runs'][path.parent.name]=run
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('evidence');a=p.parse_args();root=Path(a.evidence)
    result=summarize(root);(root/'board_bench_summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
