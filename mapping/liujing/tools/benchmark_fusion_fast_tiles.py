"""Isolated, alternating cached versus direct tile neighborhood comparison on the same captured input."""
import argparse,fcntl,hashlib,json,os,shutil,statistics,subprocess,zipfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
root=Path(__file__).resolve().parents[1]
lock=open('/home/ubuntu/losam_live_20261021/pipeline.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
a.output.mkdir(parents=True,exist_ok=False)
binary=root/'native_fusion_overflow_build/fusion_replay';shutil.copyfile(binary,a.output/'fusion_replay_tested')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sources=[x for x in (root/'native_fusion_overflow').glob('*') if x.is_file()]
with zipfile.ZipFile(a.output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
    for x in sources:z.write(x,x.relative_to(root))
(a.output/'source_sha256.json').write_text(json.dumps({x.relative_to(root).as_posix():sha(x) for x in sources},indent=2))
capture=root/'runs/fusion_v3_20261011_095743_cd7136/visual';results=[];expected={0:'9166d90bcd444267b6068e7e2f1d9a7dc5300d84bae07660cdc32027ec50cae5',1:'9166d90bcd444267b6068e7e2f1d9a7dc5300d84bae07660cdc32027ec50cae5'}
for index,enabled in enumerate((0,1,1,0)):
    prefix=a.output/f'run_{index}_fast_tiles_{enabled}'
    env=dict(os.environ,OMP_NUM_THREADS='2',OMP_THREAD_LIMIT='2',OMP_WAIT_POLICY='PASSIVE',GOMP_SPINCOUNT='0',LJ3_TILED_INDEX='1',LJ3_CELL_OVERFLOW='1',LJ3_QUERY_SIMD='1',LJ3_FAST_TILES=str(enabled))
    with prefix.with_suffix('.log').open('w') as log:subprocess.run([str(binary),'fast',str(capture),str(prefix)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    rows=[json.loads(x) for x in prefix.with_suffix('.jsonl').read_text().splitlines()];digest=sha(prefix.with_suffix('.bin'))
    expected.setdefault(enabled,digest);assert digest==expected[enabled]
    assert rows[-1]['orientation_drops']==0 and rows[-1]['capacity_drops']==0
    result={'run':index,'enabled':enabled,'frames':len(rows),'final_surfels':rows[-1]['count'],'voxel_capacity_rejections':rows[-1]['orientation_drops'],'overflow_surfels':rows[-1]['overflow_surfels'],'max_cell_occupancy':rows[-1]['max_cell_occupancy'],'map_sha256':digest,'median_ms':{key:statistics.median(x[key] for x in rows) for key in ('ms','grouping_ms','associate_ms','commit_ms')}}
    results.append(result);print(json.dumps(result),flush=True)
(a.output/'summary.json').write_text(json.dumps({'scope':'same captured input CPU reference; no live or PL acceptance','binary_sha256':sha(binary),'runs':results},indent=2))
