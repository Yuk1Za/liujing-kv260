"""Boot-scoped poison latch: PL reload does not reset outstanding PS HP traffic."""
import json,os,time
from pathlib import Path
FAULT_PATH=Path('/run/lock/liujing_raster_fault.json')

def require_healthy():
    if FAULT_PATH.exists():
        raise RuntimeError('Previous PL timeout in this boot; reboot required before further renderer acceptance: '+str(FAULT_PATH))

def latch_timeout(job):
    record=dict(reason='PL AP_DONE timeout; PS HP outstanding transactions may survive overlay reload',recovery='reboot required; do not delete this latch to continue acceptance',monotonic=time.monotonic(),job=job)
    try:record['boot_id']=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:record['boot_id']='unavailable'
    try:
        with FAULT_PATH.open('x') as f:
            json.dump(record,f,indent=2);f.flush();os.fsync(f.fileno())
    except FileExistsError:pass
