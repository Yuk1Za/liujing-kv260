"""Run three sequential board replays from sensor inputs, never saved poses."""
import argparse,json,subprocess,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('--out',required=True)
    p.add_argument('--rate',type=float,default=.5);p.add_argument('--seconds',type=float,default=180)
    p.add_argument('--observe-static-drift',action='store_true');a=p.parse_args()
    root=Path(__file__).resolve().parents[1];bag=Path(a.bag).resolve();out=Path(a.out).resolve();out.mkdir(parents=True,exist_ok=False)
    if not (bag/'metadata.yaml').is_file():raise ValueError('recorded sensor bag missing')
    rows=[]
    for index in range(1,4):
        if Path('/run/lock/liujing_raster_fault.json').exists():raise RuntimeError('PL fault latch; stop regression and reboot')
        cmd=['bash',str(root/'tools/online.sh'),'run','--mode','static_diagnostic','--replay',str(bag),'--rate',str(a.rate),'--seconds',str(a.seconds)]
        if a.observe_static_drift:cmd.append('--observe-static-drift')
        start=time.monotonic()
        with (out/f'replay_{index}.log').open('w') as log:result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
        state=json.loads((root/'runs/online_session.json').read_text())
        row={'round':index,'exit_code':result.returncode,'elapsed_wall_s':time.monotonic()-start,'run':state['run'],'manifest':state}
        rows.append(row);(out/'report.json').write_text(json.dumps({'source_bag':str(bag),'rate':a.rate,'rounds':rows},indent=2))
        print(json.dumps({k:v for k,v in row.items() if k!='manifest'}),flush=True)
    if any(x['exit_code'] for x in rows):raise SystemExit(1)


if __name__=='__main__':main()
