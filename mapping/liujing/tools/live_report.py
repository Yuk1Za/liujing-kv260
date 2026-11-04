"""Summarize real native sessions; startup/shutdown excluded by an explicit interval.

No sensor recordings are read or written. A continuous-display result is not a
calibration or mobile-acceptance result.
"""
import argparse
import bisect
import json
import math
from pathlib import Path


def percentile(values, p):
    values = sorted(values)
    return values[min(len(values)-1, math.ceil(len(values)*p)-1)] if values else None


def evaluate(metrics, frames, warmup=30., duration=300.):
    if not metrics:
        return {'status': 'INCOMPLETE', 'reason': 'no metrics'}
    start = float(metrics[0]['mono']) + warmup
    end = start + duration
    rows = [m for m in metrics if start <= float(m['mono']) <= end]
    seen = set()
    fresh = []
    for f in frames:
        seq = int(f['sequence'])
        if start <= float(f['ready_mono']) < end and seq not in seen:
            seen.add(seq)
            fresh.append(f)
    times = sorted(float(f['ready_mono']) for f in fresh)
    tracked = [f for f in fresh if f.get('view_mode','follow') != 'free']
    windows = [start, end-10] + [t+1e-7 for t in times if t+10+1e-7 <= end]
    rates = [(bisect.bisect_left(times, t+10)-bisect.bisect_left(times, t))/10 for t in windows]
    supported = (rows and float(rows[0]['mono']) <= start+1.5 and
                 float(rows[-1]['mono']) >= end-1.5 and
                 all(float(b['mono'])-float(a['mono']) <= 1.5 for a,b in zip(rows,rows[1:])))
    result = {'scope': 'real-input continuous-display check; NOT mobile/calibration acceptance',
              'interval_start_mono': start, 'duration_s': duration,
              'metrics_coverage': bool(supported), 'unique_frames': len(fresh),
              'mean_new_fps': len(fresh)/duration, 'minimum_10s_new_fps': min(rates),
              'maximum_frame_gap_s': max((b-a for a,b in zip([start]+times,times+[end])),default=duration),
              'at_least_320_fraction': sum(int(f.get('width',0))>=320 for f in fresh)/len(fresh) if fresh else 0,
              'sizes': {str(w):sum(int(f.get('width',0))==w for f in fresh) for w in (640,320,256,160)},
              'timing_ms': {k:{'median':percentile([float(f[k]) for f in fresh if k in f],.5),
                              'p95':percentile([float(f[k]) for f in fresh if k in f],.95)}
                            for k in ('prepare_ms','render_call_ms','encode_overlay_ms','view_latency_ms')},
              'map_age_p95_s': percentile([float(f['map_age_s']) for f in fresh],.95),
              'view_modes': {mode:sum(f.get('view_mode','follow')==mode for f in fresh)
                             for mode in sorted({f.get('view_mode','follow') for f in fresh})},
              'pose_age_max_s': max([float(f['pose_age_s']) for f in tracked],default=None),
              'runtime_failures': list({str(m['failure']) for m in rows if m.get('failure')}),
              'comparison_in_measurement': any(str(f.get('validation','false')).lower()=='true' for f in fresh)}
    if len(rows)>1:
        dt=float(rows[-1]['mono'])-float(rows[0]['mono'])
        def delta(k):return float(rows[-1].get(k,0))-float(rows[0].get(k,0))
        result.update(fusion_hz=delta('fused')/dt,raw_pose_hz=delta('raw_pose_count')/dt,
                      lidar_hz=delta('lidar_input')/dt,pl_completed_delta=delta('pl_completed_jobs'),
                      rejected_tasks=delta('visibility_budget_rejected'),
                      final_surfels=int(rows[-1].get('map_surfels',0)))
    if fresh and all('task_reused' in f for f in fresh):
        newly_prepared = [f for f in fresh if str(f['task_reused']).lower() != 'true']
        result['task_reuse_fraction'] = 1-len(newly_prepared)/len(fresh)
        result['prepare_new_task_ms'] = {k:percentile([float(f['prepare_ms']) for f in newly_prepared],p)
                                         for k,p in [('median',.5),('p95',.95)]}
    passed=(min(rates)>=10 and result['at_least_320_fraction']>=.9 and
            (not tracked or (result['pose_age_max_s'] is not None and result['pose_age_max_s']<=.501)) and
            result.get('fusion_hz',0)>0 and result.get('raw_pose_hz',0)>0 and result.get('lidar_hz',0)>0 and
            not result['runtime_failures'] and not result['comparison_in_measurement'] and
            result.get('pl_completed_delta',0)>0)
    result['status']='INCOMPLETE' if not supported else 'PASS_DISPLAY_ONLY' if passed else 'FAIL_DISPLAY'
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run');parser.add_argument('--warmup',type=float,default=30);parser.add_argument('--duration',type=float,default=300)
    args=parser.parse_args();root=Path(args.run)
    if (root/'visual').is_dir():root=root/'visual'
    def read(name):return [json.loads(s) for s in (root/name).read_text().splitlines() if s.strip()]
    if args.duration<10 or args.warmup<0:raise ValueError('invalid interval')
    result=evaluate(read('metrics.jsonl'),read('frames.jsonl'),args.warmup,args.duration)
    print(json.dumps(result,indent=2));(root/'display_report.json').write_text(json.dumps(result,indent=2))


if __name__=='__main__':main()
