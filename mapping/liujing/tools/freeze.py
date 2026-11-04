"""Record existing inputs without touching original projects or storing credentials."""
from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[2]
paths=['sensor_bringup_20261021/camera_ros2.cpp','sensor_bringup_20261021/acceptance.json',
 'losam_live_20261021/c16.yaml','losam_live_20261021/hardware_implementation.json',
 'losam_live_20261021/c16_acceptance.json','tmp/vision_ab_20261023/REPORT.md']
paths += [str(p.relative_to(root)).replace('\\','/') for p in (root/'losam_live_20261021/hardware_source').glob('*.cpp')]
out={p:{'sha256':hashlib.sha256((root/p).read_bytes()).hexdigest(),'bytes':(root/p).stat().st_size} for p in paths if (root/p).is_file()}
target=root/'liujing/evidence/baseline_manifest.json';target.parent.mkdir(exist_ok=True)
if target.exists(): raise SystemExit('Existing baseline preserved; do not overwrite')
target.write_text(json.dumps({'client_date':'2026-10-24','files':out,'scope':'local source and existing acceptance records; not a fresh live acceptance'},indent=2),encoding='utf8')
print(target)
