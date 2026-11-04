"""Read-only compact snapshot of the owned online session."""
import json, os, sys
from pathlib import Path

root=Path(__file__).resolve().parents[1]
state=json.loads((root/'runs/online_session.json').read_text());run=Path(state['run'])
result={'state':state,'children':[]}
for path in Path('/proc').glob('[0-9]*/cmdline'):
    try:
        env=dict(s.split(b'=',1) for s in path.with_name('environ').read_bytes().split(b'\0') if b'=' in s)
        if env.get(b'LIUJING_READY_DIR',b'').decode()!=str(run):continue
        libraries=sorted({s.split()[-1] for s in path.with_name('maps').read_text().splitlines() if any(k in s for k in ('libfastrtps.so','libfastcdr.so','librmw_fastrtps_cpp.so'))})
        result['children'].append({'pid':int(path.parent.name),'command':path.read_bytes().replace(b'\0',b' ').decode(),'libraries':libraries})
    except (FileNotFoundError,PermissionError):pass
for key,file in [('visual','visual/status.json'),('completion','visual/completion.json'),('losam','losam/live.json')]:
    try:result[key]=json.loads((run/file).read_text())
    except (FileNotFoundError,json.JSONDecodeError):pass
if '--compact' in sys.argv:
    result={'run':str(run),'session_status':state['status'],'children':len(result['children']),
        'visual':{k:v for k,v in result.get('visual',{}).items() if k in ('map_version','surfels','counts','failure','pl_completed_jobs','map_age_s','pose_topic','pose_fault')},
        'completion':result.get('completion'),
        'losam':{k:v for k,v in result.get('losam',{}).items() if k in ('/point_cloud_raw','/odom_init/laser_odom','/mapping/odometry','trajectory_extent')}}
print(json.dumps(result,indent=2))
