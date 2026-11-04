"""Copy exact small run evidence to a new archive without mutating the runs."""
import argparse,hashlib,tarfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('paths',nargs='+');a=p.parse_args()
root=Path.cwd().resolve();dest=Path(a.out)
with tarfile.open(dest,'x:gz') as tar:
 for name in a.paths:
  source=root/name
  if not source.exists():raise FileNotFoundError(source)
  for f in sorted(source.rglob('*')) if source.is_dir() else [source]:
   if f.is_file() and (not source.is_dir() or f.suffix in ('.json','.log','.txt','.png')):
    tar.add(f,arcname=f.relative_to(root).as_posix())
print(dest.stat().st_size,hashlib.sha256(dest.read_bytes()).hexdigest(),flush=True)
