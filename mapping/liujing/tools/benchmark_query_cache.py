"""Same-binary alternating cache A/B, only retained real inputs, no sensors."""
import argparse, hashlib, json, os, statistics, subprocess, shutil, zipfile, time, resource
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('binary',type=Path);p.add_argument('capture',type=Path);p.add_argument('output',type=Path)
p.add_argument('--tiled',action='store_true',help='compare tiled index instead of candidate cache')
p.add_argument('--threads',action='store_true',help='compare 1 vs 2 association workers with tiled index enabled')
p.add_argument('--commit',action='store_true',help='compare coalesced writes with tiled index and two association workers')
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
if sum((a.tiled,a.threads,a.commit))>1:raise ValueError('choose one experiment')
shutil.copyfile(a.binary,a.output/'fusion_replay_tested')
source_root=Path(__file__).resolve().parents[1]/'native_fusion_v3'
sources={}
with zipfile.ZipFile(a.output/'sources.zip','w',zipfile.ZIP_DEFLATED) as archive:
    for source in sorted(source_root.iterdir()):
        if source.is_file():
            archive.write(source,source.name);sources[source.name]=hashlib.sha256(source.read_bytes()).hexdigest()
(a.output/'source_sha256.json').write_text(json.dumps(sources,indent=2))
runs=[]
for index,setting in enumerate((0,1,1,0,0,1)):
    capacity=0 if a.tiled or a.threads or a.commit else setting*65536
    tiled=1 if a.threads or a.commit else setting if a.tiled else 0
    threads=setting+1 if a.threads else 2
    coalesced=setting if a.commit else 0
    prefix=a.output/f'run_{index}_cache_{capacity}_tiled_{tiled}_threads_{threads}_commit_{coalesced}'
    env=dict(os.environ,LJ3_QUERY_CACHE_CAPACITY=str(capacity),LJ3_TILED_INDEX=str(tiled),LJ3_ASSOC_THREADS=str(threads),LJ3_COALESCED_WRITES=str(coalesced),OMP_WAIT_POLICY='PASSIVE')
    before=resource.getrusage(resource.RUSAGE_CHILDREN);started=time.monotonic()
    subprocess.run([str(a.binary),'fast',str(a.capture),str(prefix)],env=env,check=True)
    elapsed=time.monotonic()-started;after=resource.getrusage(resource.RUSAGE_CHILDREN)
    rows=[json.loads(line) for line in prefix.with_suffix('.jsonl').read_text().splitlines()]
    if len(rows)!=120:raise ValueError('expected fixed 120-frame input')
    medians={key:statistics.median(row[key] for row in rows) for key in
        ('ms','grouping_ms','associate_ms','commit_ms','cache_update_ms','query_cache_hits','query_cache_rebuilds','query_cache_invalidations')}
    result={'index':index,'capacity':capacity,'tiled_index':bool(tiled),'association_threads':threads,'coalesced_writes':bool(coalesced),'frames':len(rows),'medians':medians,
            'process_wall_s_including_io':elapsed,'process_cpu_s_including_io':after.ru_utime+after.ru_stime-before.ru_utime-before.ru_stime,
            'map_sha256':hashlib.sha256(prefix.with_suffix('.bin').read_bytes()).hexdigest()}
    runs.append(result);print(json.dumps(result),flush=True)
expected='7210a5b2413b3ea69ec7e8e9681b55443dcc82bbd91cba508c8c1518399007a0'
if any(run['map_sha256']!=expected for run in runs):raise ValueError('map differs from frozen CPU reference')
(a.output/'summary.json').write_text(json.dumps({'runs':runs,'same_map':True,
    'binary_sha256':hashlib.sha256(a.binary.read_bytes()).hexdigest(),
    'scope':'120-frame replay only; excludes sensors, localization, browser and moving acceptance'},indent=2))
