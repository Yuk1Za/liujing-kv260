"""Bounded procfs sampler. Reports CPU execution independently of wall latency."""
import argparse,json,os,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--seconds',type=int,default=120);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
if not 1<=a.seconds<=3600:raise ValueError('duration')
names={'liujing_native','feature_extract','pldataprocess','laserOdometry','map_optimization','lslidar_driver_node','c16_attached.py'}
ticks=os.sysconf('SC_CLK_TCK');previous={};start=time.monotonic();last=start
with a.out.open('w') as out:
    while time.monotonic()-start<a.seconds:
        now=time.monotonic();rows=[];seen={}
        for proc in Path('/proc').glob('[0-9]*'):
            try:
                args=proc.joinpath('cmdline').read_bytes().split(b'\0');matched=names.intersection(Path(os.fsdecode(x)).name for x in args if x)
                if not matched:continue
                for task in proc.joinpath('task').iterdir():
                    raw=task.joinpath('stat').read_text();fields=raw.rsplit(')',1)[1].split();key=(int(proc.name),int(task.name),fields[19]);cpu=(int(fields[11])+int(fields[12]))/ticks;seen[key]=cpu
                    if key in previous:rows.append({'pid':key[0],'tid':key[1],'process':sorted(matched)[0],'thread':raw[raw.index('(')+1:raw.rindex(')')],'cpu_seconds':cpu-previous[key],'cpu_percent':100*(cpu-previous[key])/(now-last),'state':fields[0],'processor':int(fields[36])})
            except (FileNotFoundError,ProcessLookupError,PermissionError):continue
        row={'mono':now,'interval':now-last,'threads':rows,'sum_cpu_percent':sum(r['cpu_percent'] for r in rows),'loadavg':Path('/proc/loadavg').read_text().strip(),'cpu_frequency_khz':Path('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq').read_text().strip()}
        out.write(json.dumps(row)+'\n');out.flush();previous=seen;last=now;time.sleep(max(.01,1-(time.monotonic()-now)))
