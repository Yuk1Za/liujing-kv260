"""Small evidence archive; large raw bags/maps stay on board with hashes."""
import argparse,hashlib,io,json,tarfile
from pathlib import Path


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--live',required=True);a=p.parse_args()
    root=Path(__file__).resolve().parents[1];live=Path(a.live).resolve();live.relative_to(root)
    files=[];hashes={}
    for pattern in ['bag/*.db3','visual/keyframes/*.npz','visual/surfels.npy','losam/map.pcd','losam/map_xyz.npy']:
        for path in sorted(live.glob(pattern)):hashes[str(path.relative_to(root))]={'bytes':path.stat().st_size,'sha256':digest(path)}
    (live/'retained_data_hashes.json').write_text(json.dumps(hashes,indent=2))
    for run in root.joinpath('runs').glob('online_*'):
        if not run.is_dir():continue
        for pattern in ['*.json','resources.jsonl','bag/metadata.yaml','visual/*.json','visual/events.jsonl','losam/summary.json','losam/trajectory.csv','losam/map_metadata.json','http/report.json']:
            files.extend(run.glob(pattern))
    files.extend([live/'visual/last_frame.jpg',root/'runs/replay_temperature_01.jsonl'])
    with tarfile.open(a.out,'w:gz') as archive:
        for path in sorted(set(files)):
            if path.is_file():archive.add(path,arcname=str(path.relative_to(root)),recursive=False)
        for run in root.joinpath('runs').glob('online_*'):
            if not run.is_dir():continue
            for path in run.glob('*.log'):
                with path.open('rb') as f:
                    f.seek(max(0,path.stat().st_size-65536));data=f.read()
                member=tarfile.TarInfo(str(path.relative_to(root))+'.tail.txt');member.size=len(data);archive.addfile(member,io.BytesIO(data))
    path=Path(a.out);print(json.dumps({'archive':str(path),'bytes':path.stat().st_size,'sha256':digest(path),'retained_large_files':len(hashes)}))


if __name__=='__main__':main()
