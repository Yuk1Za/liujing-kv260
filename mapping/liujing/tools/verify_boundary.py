"""Check retained external burst-length sinks, independently of XML metadata."""
import argparse,json,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--tsv',required=True);p.add_argument('--burst',type=int,required=True);p.add_argument('--out',required=True);a=p.parse_args()
if a.burst not in (16,32,64,128,256):raise ValueError('supported AXI burst power of two required')
prefix='system_i/lj_raster_0/'
rows={r.split('\t')[0]:r.split('\t') for r in Path(a.tsv).read_text().splitlines()}
checks=[]
for i in range((a.burst-1).bit_length()):
 name=prefix+'m_axi_output_mem_AWLEN['+str(i)+']';row=rows.get(name,[])
 external=[x for x in row[4].split() if not x.startswith(prefix)] if len(row)>4 else []
 checks.append(dict(pin=name,external_sinks=external,retained=bool(external)))
report=dict(passed=all(x['retained'] for x in checks),scope='AWLEN external boundary retention; not complete functional acceptance',maximum_burst=a.burst,checks=checks,source_sha256=hashlib.sha256(Path(a.tsv).read_bytes()).hexdigest())
Path(a.out).write_text(json.dumps(report,indent=2));print(json.dumps(report))
if not report['passed']:raise SystemExit('Required AXI burst-length boundary signal removed')
