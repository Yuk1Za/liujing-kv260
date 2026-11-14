import time, os, sys
HZ = int(os.environ.get("CLK_TCK", "100"))
NAMES = ["feature_extract", "pynq_feature_extraction", "pldataprocess", "laserOdometry",
         "map_optimization", "cpu_feature_extract", "render_pl", "bench_repeat"]
def rd(pid):
    try:
        p = open("/proc/%d/stat" % pid).read().split()
        return int(p[13]) + int(p[14])
    except Exception:
        return None
def scan():
    out = {}
    for d in os.listdir("/proc"):
        if not d.isdigit(): continue
        try:
            cmd = open("/proc/%s/cmdline" % d, "rb").read().replace(b"\0", b" ").decode("utf-8", "ignore")
        except Exception:
            continue
        if "cpuprof" in cmd: continue
        for n in NAMES:
            if n in cmd:
                out.setdefault(n, []).append(int(d))
    return out
pids = scan()
cores = os.cpu_count() or 4
print("cores=%d pids=%s" % (cores, {k: len(v) for k, v in pids.items()}), flush=True)
prev = {k: [rd(p) for p in v] for k, v in pids.items()}
pt = time.time()
secs = int(sys.argv[1]) if len(sys.argv) > 1 else 30
for i in range(secs):
    time.sleep(1.0)
    nt = time.time(); dt = nt - pt; pt = nt
    row = []; tot = 0.0
    for k, v in pids.items():
        cur = [rd(p) for p in v]
        d = sum((c - q) for c, q in zip(cur, prev[k]) if c is not None and q is not None)
        pct = d / HZ / dt * 100.0
        row.append((k, pct)); tot += pct
        prev[k] = cur
    print("%5.1f%% | " % (tot / cores) + "  ".join("%s=%.0f" % (k, v) for k, v in sorted(row)), flush=True)
