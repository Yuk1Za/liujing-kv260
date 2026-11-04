"""Read labelled hwmon temperatures without changing board configuration."""
import argparse,json,time
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--seconds',type=float,default=480);a=p.parse_args()
end=time.monotonic()+a.seconds
with open(a.out,'x',buffering=1) as f:
    while time.monotonic()<end:
        sensors=[]
        for path in sorted(Path('/sys/class/hwmon').glob('hwmon*/temp*_input')):
            label=path.with_name(path.name.replace('_input','_label'))
            sensors.append({'path':str(path),'label':label.read_text().strip() if label.exists() else path.name,'celsius':float(path.read_text())/1000})
        f.write(json.dumps({'monotonic':time.monotonic(),'sensors':sensors})+'\n');time.sleep(2)
