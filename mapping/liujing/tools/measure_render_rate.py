"""Static A/B/A render-rate experiment, bounded and stopped after each phase."""
import argparse,json,subprocess,time,urllib.request
from pathlib import Path

ROOT=Path('/home/ubuntu/liujing')

def get(path,port=8090,data=None):
    request=urllib.request.Request('http://127.0.0.1:'+str(port)+path,
        data=json.dumps(data).encode() if data is not None else None,
        headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=2) as response:return json.load(response)

def session():
    return json.loads(subprocess.check_output(['python3',str(ROOT/'tools/online_session.py'),'status','--engine','native-v2']))

def sample():
    s=get('/status');b=get('/api/status',8091);b.pop('path',None)
    return {'mono':time.monotonic(),'native':s,'bridge':b}

def main():
    p=argparse.ArgumentParser();p.add_argument('--unit-prefix',required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    for label,hz,seconds in [('A1_2hz',2,60),('B_10hz',10,120),('A2_2hz',2,60)]:
        if session().get('active'):raise RuntimeError('existing session active')
        if Path('/run/lock/liujing_raster_fault.json').exists():raise RuntimeError('PL fault latch')
        folder=a.out/label;folder.mkdir();unit=a.unit_prefix+'-'+label.lower()
        subprocess.run(['systemd-run','--unit='+unit,'--collect','/bin/bash',str(ROOT/'tools/start_mobile_demo.sh'),'300','--render-hz',str(hz)],check=True)
        print(json.dumps({'phase':label,'event':'starting','target_hz':hz}),flush=True)
        try:
            deadline=time.monotonic()+100
            while time.monotonic()<deadline:
                try:
                    row=sample()
                    if row['native'].get('failure'):raise RuntimeError(row['native']['failure'])
                    if row['bridge']['live_ready'] and row['bridge']['layers'].get('geometry') and int(row['native'].get('pl_completed_jobs',0))>0:break
                except (OSError,ValueError):pass
                time.sleep(1)
            else:raise RuntimeError('startup readiness timeout')
            if float(row['native']['render_hz_limit'])!=hz:raise RuntimeError('rate not applied')
            manifest=session();(folder/'run.json').write_text(json.dumps(manifest,indent=2))
            print(json.dumps({'phase':label,'event':'warming','run':manifest['run']}),flush=True)
            warm_end=time.monotonic()+30
            while time.monotonic()<warm_end:
                row=sample()
                if row['native'].get('failure'):raise RuntimeError(row['native']['failure'])
                time.sleep(1)
            start=time.monotonic();end=start+seconds;next_request=start+5;next_log=start
            requests=[];last=None
            with (folder/'samples.jsonl').open('w',buffering=1) as log:
                while time.monotonic()<end:
                    tick=time.monotonic()
                    try:
                        row=sample();row['elapsed_s']=row['mono']-start;log.write(json.dumps(row)+'\n');last=row
                        if row['native'].get('failure'):raise RuntimeError(row['native']['failure'])
                    except OSError as exc:
                        log.write(json.dumps({'mono':time.monotonic(),'request_error':str(exc)})+'\n')
                    if tick>=next_request:
                        requested=time.monotonic();reply=get('/view',data={'mode':'free'})
                        requests.append({'requested_mono':requested,'reply':reply});next_request+=10
                    if tick>=next_log and last:
                        print(json.dumps({'phase':label,'elapsed_s':round(tick-start),'jobs':last['native'].get('pl_completed_jobs'),'frames':last['native'].get('new_frame'),'mapping_count':last['native'].get('mapping_pose_count'),'map_age':last['bridge']['layers'].get('geometry',{}).get('source_age_s')}),flush=True);next_log+=30
                    time.sleep(max(0,.5-(time.monotonic()-tick)))
                row=sample();row['elapsed_s']=row['mono']-start;log.write(json.dumps(row)+'\n')
            (folder/'window.json').write_text(json.dumps({'start_mono':start,'end_mono':row['mono'],'target_hz':hz,'warmup_s':30,'refresh_requests':requests},indent=2))
        finally:
            subprocess.run(['systemctl','kill','--kill-who=main','--signal=SIGINT',unit],check=False)
            for _ in range(65):
                state=session()
                if not state.get('active'):break
                time.sleep(1)
            (folder/'stopped.json').write_text(json.dumps(state,indent=2))
            print(json.dumps({'phase':label,'event':'stopped','active':state.get('active'),'status':state.get('status')}),flush=True)
            if state.get('active') or state.get('status')=='fault_pinned_until_reboot':raise RuntimeError('session did not stop')
        if state.get('status')!='stopped':raise RuntimeError('session failed')
    (a.out/'success.json').write_text(json.dumps({'complete':True,'phases':3,'no_cpu_compare':True,'view_refresh':'same free view once per ten seconds; no physical movement'}))

if __name__=='__main__':main()
