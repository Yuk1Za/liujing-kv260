"""Same-input capacity accounting. No sensor or PL session is started."""
import argparse,fcntl,hashlib,json,os,shutil,subprocess,zipfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
root=Path(__file__).resolve().parents[1]
lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
a.output.mkdir(parents=True,exist_ok=False)
binary=root/'native_fusion_v3_build/fusion_replay';shutil.copyfile(binary,a.output/'fusion_replay_tested')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sources=[x for x in (root/'native_fusion_v3').glob('*') if x.is_file() and x.suffix in ('.cpp','.hpp','.txt')]
with zipfile.ZipFile(a.output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
    for x in sources:z.write(x,x.relative_to(root))
(a.output/'source_sha256.json').write_text(json.dumps({x.relative_to(root).as_posix():sha(x) for x in sources},indent=2))
capture=root/'runs/fusion_v3_20261011_095743_cd7136/visual'
env=dict(os.environ,OMP_NUM_THREADS='2',OMP_THREAD_LIMIT='2',OMP_WAIT_POLICY='PASSIVE',GOMP_SPINCOUNT='0',LJ3_TILED_INDEX='1')
expected={'baseline':'ad33a89782594e591490bd834b6ee66131be8d5341d9125f5fd1fe147b6099b5','fast':'7210a5b2413b3ea69ec7e8e9681b55443dcc82bbd91cba508c8c1518399007a0'}
results={}
for mode in expected:
    prefix=a.output/mode
    with (a.output/(mode+'.log')).open('w') as log:subprocess.run([str(binary),mode,str(capture),str(prefix)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    rows=[json.loads(x) for x in prefix.with_suffix('.jsonl').read_text().splitlines()]
    digest=sha(prefix.with_suffix('.bin'));assert digest==expected[mode]
    results[mode]={'frames':len(rows),'input_observations':sum(x['points'] for x in rows),'final_surfels':rows[-1]['count'],'capacity_drops':rows[-1]['capacity_drops'],'voxel_capacity_rejections':rows[-1]['orientation_drops'],'map_sha256':digest}
    print(mode+': '+json.dumps(results[mode]),flush=True)
(a.output/'summary.json').write_text(json.dumps({'scope':'same captured input, original thresholds and four-surfel cell limit; counts are rejected observations, not unique surfaces','binary_sha256':sha(binary),'results':results},indent=2))
