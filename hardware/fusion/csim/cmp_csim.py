import numpy as np, os
P="/home/ubuntu/fusion_probe_pl/ab_pl"
C="/home/ubuntu/fusion_probe_pl/csim"
pairs=[("flat","_feature_fpga_output_flat.npy","flat.bin"),
       ("sharp","_feature_fpga_output_sharp.npy","sharp.bin"),
       ("lesssharp","_feature_fpga_output_lesssharp.npy","lesssharp.bin")]
for name,pf,cf in pairs:
    pl=np.load(os.path.join(P,pf)).astype(np.int16)
    cs=np.frombuffer(open(os.path.join(C,cf),"rb").read(),dtype="<i2").reshape(-1,2)
    csn=cs[(cs!=0).any(axis=1)]
    pls=set(map(tuple,pl.tolist())); css=set(map(tuple,csn.tolist()))
    inter=len(pls&css)
    print("%-9s PL=%4d csim=%4d  identical_sets=%s  |PL\\csim|=%d  |csim\\PL|=%d  first_mismatch=%s" % (
        name, len(pl), len(csn), pls==css, len(pls-css), len(css-pls),
        sorted(pls-css)[:3] if pls!=css else "-"))
