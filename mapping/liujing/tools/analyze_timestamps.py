"""Diagnose timestamp domains. A fitted clock here is not exposure-time calibration."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('metadata');p.add_argument('--out',required=True);a=p.parse_args();groups={}
for line in Path(a.metadata).read_text().splitlines():
 row=json.loads(line);groups.setdefault(row['stream'],[]).append(row)
report={'scope':'clock-domain diagnostic, not exposure synchronization','streams':{}}
for kind,rows in groups.items():
 host=np.array([r['host_monotonic_ns'] for r in rows],np.int64);wall=np.array([r['host_wall_ns'] for r in rows],np.int64);device=np.array([r['device_timestamp_us'] for r in rows],np.int64);sdk=np.array([r['sdk_system_timestamp_ms'] for r in rows],np.int64);seq=np.array([r['sequence'] for r in rows],np.int64)
 dh=np.diff(host)*1e-9;dd=np.diff(device)*1e-6;good=dh>0
 report['streams'][kind]={'frames':len(rows),'host_observed_hz':float((len(rows)-1)/((host[-1]-host[0])*1e-9)),'host_interval_ms_median':float(np.median(dh)*1e3),'host_interval_ms_p99':float(np.percentile(dh,99)*1e3),'device_over_host_increment_ratio_median':float(np.median(dd[good]/dh[good])),'device_non_increasing':int(np.sum(dd<=0)),'sequence_non_increasing':int(np.sum(np.diff(seq)<=0)),'sdk_to_host_receipt_ms_median':float(np.median(wall*1e-6-sdk)),'sdk_to_host_receipt_ms_p99':float(np.percentile(wall*1e-6-sdk,99))}
Path(a.out).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
