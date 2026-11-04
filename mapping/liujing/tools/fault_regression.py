"""Interrupt only the camera/LiDAR children of two newly owned diagnostic runs."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path


def owned(run):
    matches=[]
    for p in Path('/proc').glob('[0-9]*/environ'):
        try:
            env=dict(x.split(b'=',1) for x in p.read_bytes().split(b'\0') if b'=' in x)
            if env.get(b'LIUJING_READY_DIR',b'').decode()!=run:continue
            args=p.with_name('cmdline').read_bytes().split(b'\0')
            matches.append((int(p.parent.name),{Path(os.fsdecode(x)).name for x in args if x}))
        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    return matches


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args()
    root=Path(__file__).resolve().parents[1];out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=False);rows=[]
    for sensor,executable in [('camera','liujing_camera'),('lidar','lslidar_driver_node')]:
        if Path('/run/lock/liujing_raster_fault.json').exists():raise RuntimeError('PL fault latch; reboot before further tests')
        state_path=root/'runs/online_session.json';previous=json.loads(state_path.read_text()).get('run') if state_path.exists() else None
        with (out/(sensor+'.log')).open('w') as log:
            proc=subprocess.Popen(['bash',str(root/'tools/online.sh'),'run','--mode','static_diagnostic','--observe-static-drift','--seconds','90'],stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+100;ready=None
            while proc.poll() is None and time.monotonic()<deadline:
                try:
                    state=json.loads(state_path.read_text());run=state['run']
                    if run!=previous and state['status']=='running':
                        visual=json.loads(Path(run,'visual/status.json').read_text())
                        if visual['counts'].get('fused',0):ready=run;break
                except (FileNotFoundError,json.JSONDecodeError):pass
                time.sleep(.5)
            if ready is None:
                proc.wait(timeout=60);raise RuntimeError('no fresh fused diagnostic frame before interruption')
            if sensor=='camera':
                subprocess.run(['/usr/bin/python3',str(root/'tools/web_probe.py'),'--out',str(out/'http')],check=True,timeout=60)
            targets=[pid for pid,names in owned(ready) if executable in names]
            if len(targets)!=1:raise RuntimeError('expected exactly one owned sensor process')
            # pidfd prevents signalling a reused PID. The sensor was selected by this run's environment marker.
            fd=os.pidfd_open(targets[0]);signal.pidfd_send_signal(fd,signal.SIGINT);os.close(fd)
            code=proc.wait(timeout=60);manifest=json.loads(Path(ready,'manifest.json').read_text());remaining=owned(ready)
            passed=code!=0 and sensor in (manifest.get('error') or '') and not remaining
            rows.append({'sensor':sensor,'pid':targets[0],'signal':'SIGINT to owned driver; no physical cable removal','run':ready,'exit_code':code,'recognized_and_cleaned':passed,'remaining_children':remaining,'manifest':manifest})
            (out/'report.json').write_text(json.dumps({'tests':rows},indent=2));print(json.dumps({'sensor':sensor,'passed':passed,'run':ready}),flush=True)
    if not all(r['recognized_and_cleaned'] for r in rows):raise SystemExit(1)


if __name__=='__main__':main()
