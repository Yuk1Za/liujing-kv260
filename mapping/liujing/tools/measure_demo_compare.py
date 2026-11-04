"""Bounded real-input CPU/PL paired check; always stop the owned session."""
import argparse,json,subprocess,time,urllib.request
from pathlib import Path

ROOT=Path('/home/ubuntu/liujing')

def http(path,post=False):
    req=urllib.request.Request('http://127.0.0.1:8090'+path,
        data=b'' if post else None,method='POST' if post else 'GET')
    with urllib.request.urlopen(req,timeout=3) as response:return json.load(response)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--unit',required=True)
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=False)
    state=ROOT/'runs/native_session.json'
    def status():
        return json.loads(subprocess.check_output(['python3',str(ROOT/'tools/online_session.py'),'status','--engine','native-v2']))
    if status().get('active'):raise RuntimeError('existing session active')
    if Path('/run/lock/liujing_raster_fault.json').exists():raise RuntimeError('PL fault latch')
    subprocess.run(['systemd-run','--unit='+args.unit,'--collect','/bin/bash',str(ROOT/'tools/start_mobile_demo.sh'),'300'],check=True)
    run=None
    try:
        deadline=time.monotonic()+100
        while time.monotonic()<deadline:
            try:
                s=http('/status')
                if s.get('failure'):raise RuntimeError(s['failure'])
                if int(s.get('pl_completed_jobs',0))>=20 and int(s.get('map_surfels',0))>=1000:break
            except (OSError,ValueError):pass
            time.sleep(1)
        else:raise RuntimeError('live data did not become ready')
        manifest=json.loads(state.read_text());run=Path(manifest['run'])
        (args.out/'run.json').write_text(json.dumps(manifest,indent=2))
        comparisons=run/'visual/comparisons.jsonl'
        print(json.dumps({'ready':True,'run':str(run),'map_surfels':s.get('map_surfels')}),flush=True)
        with (args.out/'paired_samples.jsonl').open('w',buffering=1) as log:
            for index in range(33):
                before=http('/status');request=http('/compare-next',True)
                until=time.monotonic()+12
                while time.monotonic()<until:
                    lines=comparisons.read_text().splitlines() if comparisons.exists() else []
                    if len(lines)>index:break
                    if http('/status').get('failure'):raise RuntimeError('renderer failure')
                    time.sleep(.1)
                else:raise RuntimeError('comparison completion timeout')
                pair=json.loads(lines[index]);after=http('/status')
                item={'index':index,'warmup':index<3,'pair':pair,'before':before,'after':after,'board_mono':time.monotonic()}
                log.write(json.dumps(item)+'\n')
                if int(pair['mismatched_pixels']):raise RuntimeError('CPU/PL mismatch')
                if index%5==0 or index==32:print(json.dumps({'index':index,**pair}),flush=True)
                time.sleep(.5)
        (args.out/'success.json').write_text(json.dumps({'pairs':33,'warmups':3,'measured':30,'scope':'real static sensor input; identical prepared task per pair; live pipeline concurrent; PL then 2-thread CPU; RGB/depth/ID full-output comparison'}))
    finally:
        subprocess.run(['systemctl','kill','--kill-who=main','--signal=SIGINT',args.unit],check=False)
        for _ in range(65):
            s=status()
            if not s.get('active'):break
            time.sleep(1)
        (args.out/'stopped.json').write_text(json.dumps(s,indent=2))
        print(json.dumps({'stopped':not s.get('active'),'status':s.get('status'),'run':str(run)}),flush=True)
        if s.get('active'):raise RuntimeError('owned session has not stopped')

if __name__=='__main__':main()
