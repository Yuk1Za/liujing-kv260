"""Collect completed diagnostic runs; never turns calibration flags into passes."""
import argparse,json
from pathlib import Path
import numpy as np
from session_report import report


def main():
    p=argparse.ArgumentParser()
    for name in ('live','baseline','regression','match','faults','out'):p.add_argument('--'+name,required=True)
    a=p.parse_args();root=Path(__file__).resolve().parents[1];out=Path(a.out);out.mkdir(exist_ok=False,parents=True)
    def collect(path):
        path=Path(path);data=report(path);(path/'joint_report.json').write_text(json.dumps(data,indent=2))
        keys=('run','sampled_duration_s','completion','observed_fusion_hz','observed_new_image_hz','system_cpu_busy_percent','system_cpu_iowait_percent','mem_available_kib','temperature_c','fusion_ms','association_ms','render_ms','jpeg_ms','receive_to_image_ready_ms','map_age_s','recorded_counts','bag_integrity','camera_clock_audit','stationary_translation_diagnostic')
        result={k:data.get(k) for k in keys};result['map_age_s_scope']=data['map_age_s_scope'];result['recorded_status_map_age_s']=data['recorded_status_map_age_s'];result['localization']={k:v for k,v in data.get('losam',{}).items() if k in ('/point_cloud_raw','/odom_init/laser_odom','/mapping/odometry','/feature/fpga_output_cloud')}
        return result
    result={'scope':'static unverified diagnostic; moving reconstruction not accepted','live':collect(a.live),'baseline':collect(a.baseline)}
    regression=json.loads(Path(a.regression,'report.json').read_text());result['replay_rate']=regression['rate']
    result['replays']=[{'round':r['round'],'exit_code':r['exit_code'],**collect(r['run'])} for r in regression['rounds']]
    match=json.loads(Path(a.match,'report.json').read_text());result['matched']={'passed':match['passed'],'frames':match['frames'],'views_per_frame':match['views_per_frame'],'scope':match['scope'],'timing_scope':match['timing_scope']}
    for backend in ('cpu','pl'):result['matched'][backend]={key:float(np.mean([r[backend][key] for r in match['rows']])) for key in ('ms','process_cpu_ms','association_ms','fusion_ms')}
    faults=json.loads(Path(a.faults,'report.json').read_text());result['fault_tests']=[{k:r[k] for k in ('sensor','signal','run','exit_code','recognized_and_cleaned','remaining_children')} for r in faults['tests']]
    for row in faults['tests']:collect(row['run'])
    http=json.loads(Path(a.faults,'http/report.json').read_text());result['board_http']={k:http[k] for k in ('scope','request_count','all_204','jpeg_bytes')};result['board_http']['superseded_requests']=http['after'].get('counts',{}).get('view_superseded',0)
    temp=root/'runs/replay_temperature_01.jsonl'
    if temp.exists():
        rows=[json.loads(x) for x in temp.read_text().splitlines()];paths=sorted({s['path'] for r in rows for s in r['sensors']})
        result['replay_temperature_c']={path:{'min':min(s['celsius'] for r in rows for s in r['sensors'] if s['path']==path),'max':max(s['celsius'] for r in rows for s in r['sensors'] if s['path']==path)} for path in paths}
    (out/'report.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({'replays':[{'round':r['round'],'exit_code':r['exit_code'],'counts':r['completion']['counts'],'surfels':r['completion']['surfels']} for r in result['replays']],'matched':result['matched'],'fault_tests':result['fault_tests'],'board_http':result['board_http'],'live_pose_drift':result['live']['stationary_translation_diagnostic'],'baseline_localization':result['baseline']['localization'],'replay_temperature_c':result.get('replay_temperature_c')},indent=2))


if __name__=='__main__':main()
