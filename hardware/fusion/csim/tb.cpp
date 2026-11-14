#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ap_int.h>
#include "registration_fpga.h"

#define MAX_IN 117142
#define MAX_CNT 115200
#define FLAT_D 1536
#define LS_D 7680
#define SH_D 768

static My_PointXYZI_Port in[MAX_IN];
static My_PointXYZI_Port pt[MAX_CNT];
static float rng[MAX_CNT];
static ap_uint<1> lessflat[MAX_CNT];
static ap_uint<2> ground[MAX_CNT];
static My_PointOutFeatureLocation flatb[FLAT_D];
static My_PointOutFeatureLocation lsb[LS_D];
static My_PointOutFeatureLocation shb[SH_D];
static unsigned char lf8[MAX_CNT];

int main(int argc, char** argv) {
    if (argc < 5) { printf("usage: tb in.bin outdir ori0 ori1\n"); return 1; }
    FILE* f = fopen(argv[1], "rb");
    if (!f) { printf("ERR cannot open %s\n", argv[1]); return 1; }
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    int n = (int)(sz / (long)sizeof(My_PointXYZI_Port));
    size_t rd = fread(in, sizeof(My_PointXYZI_Port), n, f);
    fclose(f);
    printf("in n=%d read=%zu sizeofP=%zu\n", n, rd, sizeof(My_PointXYZI_Port)); fflush(stdout);
    memset(pt, 0, sizeof(pt)); memset(flatb, 0, sizeof(flatb));
    memset(lsb, 0, sizeof(lsb)); memset(shb, 0, sizeof(shb));
    fpga_feature_extract_hw(in, n, (float)atof(argv[3]), (float)atof(argv[4]),
        pt, rng, lessflat, ground, flatb, lsb, shb);
    printf("hw done\n"); fflush(stdout);
    char p[512];
    sprintf(p, "%s/flat.bin", argv[2]); FILE* o = fopen(p, "wb");
    fwrite(flatb, sizeof(My_PointOutFeatureLocation), FLAT_D, o); fclose(o);
    sprintf(p, "%s/lesssharp.bin", argv[2]); o = fopen(p, "wb");
    fwrite(lsb, sizeof(My_PointOutFeatureLocation), LS_D, o); fclose(o);
    sprintf(p, "%s/sharp.bin", argv[2]); o = fopen(p, "wb");
    fwrite(shb, sizeof(My_PointOutFeatureLocation), SH_D, o); fclose(o);
    for (int i = 0; i < MAX_CNT; i++) lf8[i] = (unsigned char)lessflat[i].to_uint();
    sprintf(p, "%s/lessflat.bin", argv[2]); o = fopen(p, "wb");
    fwrite(lf8, 1, MAX_CNT, o); fclose(o);
    printf("done\n");
    return 0;
}
