"""Read a finished joint session, audit recordings and compute actual rates."""
import argparse, csv, json, sqlite3, struct
from pathlib import Path
import numpy as np


def metadata_cdr(blob):
    """Losslessly unpack rosbag2 message zstd before reading String CDR."""
    if not blob.startswith(b'\x28\xb5\x2f\xfd'):return blob
    import ctypes as C
    lib=C.CDLL('libzstd.so.1')
    lib.ZSTD_getFrameContentSize.argtypes=[C.c_void_p,C.c_size_t];lib.ZSTD_getFrameContentSize.restype=C.c_ulonglong
    lib.ZSTD_decompress.argtypes=[C.c_void_p,C.c_size_t,C.c_void_p,C.c_size_t];lib.ZSTD_decompress.restype=C.c_size_t
    size=lib.ZSTD_getFrameContentSize(blob,len(blob))
    if not 0<size<=1024*1024:raise ValueError('invalid metadata zstd frame size')
    output=C.create_string_buffer(size)
    if lib.ZSTD_decompress(output,size,blob,len(blob))!=size:raise ValueError('metadata decompression failed')
    return output.raw


def stats(x):
    x=np.asarray(x,dtype=float)
    return {'count':len(x),'mean':float(x.mean()),'p99':float(np.percentile(x,99)),'max':float(x.max())} if len(x) else None


def report(root):
    root=Path(root);out={'run':str(root),'scope':'observed joint session; not automatic calibration acceptance'}
    manifest=json.loads((root/'manifest.json').read_text());out['manifest']=manifest
    out['source_age_scope']='wall-versus-recorded source age from legacy monitor is not replay latency; use matched_input latency and /clock visual age' if manifest.get('replay') else 'live board host time'
    events=root/'visual/events.jsonl'
    rows=[json.loads(x) for x in events.read_text().splitlines()] if events.exists() else []
    rows=[x for x in rows if 'map_version' in x]
    out['visual_rows']=len(rows)
    resource_path=root/'resources.jsonl'
    resources=[json.loads(x) for x in resource_path.read_text().splitlines()] if resource_path.exists() else []
    if len(resources)>1:
        duration=resources[-1]['mono']-resources[0]['mono']
        out['sampled_duration_s']=duration
        completion_path=root/'visual/completion.json'
        if completion_path.exists():
            completion=json.loads(completion_path.read_text());out['completion']=completion
            out['observed_fusion_hz']=completion['counts'].get('fused',0)/duration
            out['observed_new_image_hz']=completion['counts'].get('rendered',0)/duration
        cpu=np.asarray([[int(x) for x in r['cpu'].split()[1:9]] for r in resources],dtype=float)
        delta=np.diff(cpu,axis=0);total=delta.sum(axis=1);valid=total>0
        out['system_cpu_busy_percent']=stats(100*(total[valid]-delta[valid,3]-delta[valid,4])/total[valid])
        out['system_cpu_iowait_percent']=stats(100*delta[valid,4]/total[valid])
        memory=[{line.split(':')[0]:int(line.split()[1]) for line in r['meminfo'].splitlines()} for r in resources]
        out['mem_available_kib']=stats([m['MemAvailable'] for m in memory])
        out['dirty_kib']=stats([m['Dirty'] for m in memory])
        out['temperature_c']=stats([float(t)/1000 for r in resources for t in r['temperature']])
    for key in ('fusion_ms','association_ms','render_ms','compose_ms','ros_image_publish_ms','jpeg_ms','receive_to_image_ready_ms','map_age_s'):
        selected=[r for r in rows if r.get('fusion_ms',0)>0] if key=='association_ms' else rows
        out[key]=stats([r[key] for r in selected if r.get(key) is not None and (r[key]>0 or key=='map_age_s')])
    if rows:
        out['latest_visual']=rows[-1]
        stamps=sorted({r['map_source_stamp'] for r in rows if r.get('map_source_stamp') is not None})
        out['fused_source_intervals_s']=stats(np.diff(stamps))
    counts={};times={};integrity=[];metadata=[];status_samples=[]
    for db in sorted((root/'bag').glob('*.db3')):
        with sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True) as c:
            integrity.append({'file':db.name,'result':c.execute('PRAGMA integrity_check').fetchone()[0]})
            topics=dict(c.execute('select id,name from topics'))
            for tid,name in topics.items():
                stamps=[r[0] for r in c.execute('select timestamp from messages where topic_id=? order by timestamp',(tid,))]
                counts[name]=counts.get(name,0)+len(stamps);times.setdefault(name,[]).extend(stamps)
                if name in ('/camera/frame_metadata','/liujing/status'):
                    for blob, in c.execute('select data from messages where topic_id=? order by timestamp',(tid,)):
                        blob=metadata_cdr(blob)
                        # std_msgs/String consists of one CDR length-prefixed UTF-8 string.
                        if len(blob)<9 or blob[:2] not in (b'\x00\x01',b'\x00\x00'):raise ValueError('unsupported metadata CDR')
                        n=struct.unpack_from('<I' if blob[1]==1 else '>I',blob,4)[0]
                        if n<1 or len(blob)<8+n or blob[7+n]!=0:raise ValueError('invalid metadata CDR string')
                        value=json.loads(blob[8:7+n].decode())
                        (metadata if name=='/camera/frame_metadata' else status_samples).append(value)
    out['bag_integrity']=integrity;out['recorded_counts']=counts
    out['map_age_s_scope']='fusion/render event snapshots only; not continuous freshness'
    out['recorded_status_map_age_s']=stats([r['map_age_s'] for r in status_samples if r.get('map_age_s') is not None])
    out['recorded_intervals_s']={k:stats(np.diff(sorted(v))/1e9) for k,v in times.items()}
    clock={}
    for stream in ('color','depth'):
        data=[m for m in metadata if m['stream']==stream]
        if len(data)<2:continue
        seq=np.array([m['sequence'] for m in data],dtype=np.int64)
        dev=np.array([m['device_timestamp_us'] for m in data],dtype=np.float64)
        host=np.array([m['host_monotonic_ns'] for m in data],dtype=np.float64)
        dd=np.diff(dev);dh=np.diff(host)/1000;ok=(dd>0)&(dh>0)
        clock[stream]={'metadata_messages':len(data),'sequence_gaps':int(np.maximum(np.diff(seq)-1,0).sum()),
            'sequence_regressions':int((np.diff(seq)<=0).sum()),'device_regressions':int((dd<=0).sum()),
            'device_to_host_increment_ratio':float(np.median(dd[ok]/dh[ok])) if ok.any() else None,
            'device_to_host_elapsed_ratio':float((dev[-1]-dev[0])/((host[-1]-host[0])/1000)) if host[-1]>host[0] else None,
            'timestamp_basis':data[0].get('ros_stamp_basis'),'calibrated':False}
    out['camera_clock_audit']=clock
    out['color_storage']='native_jpeg' if counts.get('/camera/color/image_raw/compressed',0) else 'decoded_bgr'
    out['raw_recording_nonempty']=(counts.get('/point_cloud_raw',0)>0 and counts.get('/camera/depth/image_raw',0)>0 and (counts.get('/camera/color/image_raw',0)>0 or counts.get('/camera/color/image_raw/compressed',0)>0)) if not manifest['lidar_only'] else counts.get('/point_cloud_raw',0)>0
    if manifest.get('replay'):out['raw_recording_nonempty']=None;out['recording_scope']='replay reads source bag; no duplicate raw recording'
    trajectory=root/'losam/trajectory.csv'
    if trajectory.exists():
        with trajectory.open() as f:poses=list(csv.DictReader(f))
        out['stationary_translation_diagnostic']={}
        for topic in ('/odom_init/laser_odom','/mapping/odometry'):
            selected=[r for r in poses if r['topic']==topic]
            if not selected:continue
            xyz=np.array([[float(r[k]) for k in ('x','y','z')] for r in selected]);stamp=np.array([int(r['stamp_ns']) for r in selected])
            warm=xyz[stamp>=stamp[0]+10_000_000_000]
            out['stationary_translation_diagnostic'][topic]={'count':len(xyz),'max_from_first_m':float(np.linalg.norm(xyz-xyz[0],axis=1).max()),'max_after_10s_from_first_retained_m':float(np.linalg.norm(warm-warm[0],axis=1).max()) if len(warm) else None}
    losam=root/'losam/summary.json'
    if losam.exists():out['losam']=json.loads(losam.read_text())
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args()
    data=report(a.run);Path(a.run,'joint_report.json').write_text(json.dumps(data,indent=2));print(json.dumps({k:v for k,v in data.items() if k not in ('manifest','latest_visual','losam')},indent=2))
