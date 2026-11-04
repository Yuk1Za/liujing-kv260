from pathlib import Path
import tarfile,hashlib,json
root=Path(__file__).resolve().parents[1]
dest=root/'evidence/deploy.tgz'
def stable(info):info.mtime=0;info.uid=info.gid=0;info.uname=info.gname='';return info
with tarfile.open(dest,'w:gz') as tar:
    for p in sorted(root.rglob('*')):
        rel=p.relative_to(root)
        if not p.is_file() or any(x in ('build','sensors_build','artifacts','runs','evidence','__pycache__','.git') for x in rel.parts):continue
        tar.add(p,arcname=str(Path('liujing')/rel),filter=stable)
manifest={'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'bytes':dest.stat().st_size,'client_date':'2026-10-24'}
dest.with_suffix('.json').write_text(json.dumps(manifest,indent=2));print(manifest)
