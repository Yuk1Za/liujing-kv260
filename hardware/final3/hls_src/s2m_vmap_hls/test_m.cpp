// test_m.cpp —— 纯数学验证：calc_M 生成的 J 是否等于原码 arx/ary/arz 的表达式
// 编译：g++ -O2 -o test_m test_m.cpp
// 判据：三个 J 分量与基准的误差应 <= 1e-6（float 舍入）
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include "pl_s2m.hpp"

// 原码（map_optimization.cpp）的 arx/ary/arz，逐字符照抄，只把 coeff.x/y/z 换成 cx/cy/cz
static void ref_ar(const float srx, const float crx, const float sry, const float cry,
                   const float srz, const float crz,
                   const float x, const float y, const float z,
                   const float cx, const float cy, const float cz,
                   float &arx, float &ary, float &arz)
{
    arx = (crx * sry * srz * x + crx * crz * sry * y - srx * sry * z) * cx
        + (-srx * srz * x - crz * srx * y - crx * z) * cy
        + (crx * cry * srz * x + crx * cry * crz * y - cry * srx * z) * cz;

    ary = ((cry * srx * srz - crz * sry) * x + (sry * srz + cry * crz * srx) * y + crx * cry * z) * cx
        + ((-cry * crz - srx * sry * srz) * x + (cry * srz - crz * srx * sry) * y - crx * sry * z) * cz;

    arz = ((crz * srx * sry - cry * srz) * x + (-cry * crz - srx * sry * srz) * y) * cx
        + (crx * crz * x - crx * srz * y) * cy
        + ((sry * srz + cry * crz * srx) * x + (crz * sry - cry * srx * srz) * y) * cz;
}

int main()
{
    const int N = 1000;
    srand(7);
    double worst = 0.0;
    int    worst_i = -1;
    for (int t = 0; t < N; ++t)
    {
        // 随机位姿（±60°）与随机点/法向
        const float srx = std::sin((rand() / (float)RAND_MAX - 0.5f) * 2.0f);
        const float crx = std::sqrt(1.f - srx * srx);
        const float sry = std::sin((rand() / (float)RAND_MAX - 0.5f) * 2.0f);
        const float cry = std::sqrt(1.f - sry * sry);
        const float srz = std::sin((rand() / (float)RAND_MAX - 0.5f) * 2.0f);
        const float crz = std::sqrt(1.f - srz * srz);

        const float x = (rand() / (float)RAND_MAX - 0.5f) * 80.f;
        const float y = (rand() / (float)RAND_MAX - 0.5f) * 80.f;
        const float z = (rand() / (float)RAND_MAX - 0.5f) * 20.f;
        const float cx = rand() / (float)RAND_MAX - 0.5f;
        const float cy = rand() / (float)RAND_MAX - 0.5f;
        const float cz = rand() / (float)RAND_MAX - 0.5f;

        float arx0, ary0, arz0;
        ref_ar(srx, crx, sry, cry, srz, crz, x, y, z, cx, cy, cz, arx0, ary0, arz0);

        float S6[6] = {srx, crx, sry, cry, srz, crz};
        float M[27];
        pls2m::calc_M(S6, M);

        // 内核式子：pc = (y, z, x)；cc0/cc1/cc2 = 原码表达式里的 coeff.x/y/z
        const float pc0 = y, pc1 = z, pc2 = x;
        const float cc0 = cx, cc1 = cy, cc2 = cz;
        const float J0 = (M[0] * pc0 + M[1] * pc1 + M[2] * pc2) * cc0
                       + (M[3] * pc0 + M[4] * pc1 + M[5] * pc2) * cc1
                       + (M[6] * pc0 + M[7] * pc1 + M[8] * pc2) * cc2;
        const float J1 = (M[9] * pc0 + M[10] * pc1 + M[11] * pc2) * cc0
                       + (M[12] * pc0 + M[13] * pc1 + M[14] * pc2) * cc1
                       + (M[15] * pc0 + M[16] * pc1 + M[17] * pc2) * cc2;
        const float J2 = (M[18] * pc0 + M[19] * pc1 + M[20] * pc2) * cc0
                       + (M[21] * pc0 + M[22] * pc1 + M[23] * pc2) * cc1
                       + (M[24] * pc0 + M[25] * pc1 + M[26] * pc2) * cc2;

        const double e0 = std::fabs(J0 - arz0);
        const double e1 = std::fabs(J1 - arx0);
        const double e2 = std::fabs(J2 - ary0);
        const double e = e0 > e1 ? (e0 > e2 ? e0 : e2) : (e1 > e2 ? e1 : e2);
        if (e > worst) { worst = e; worst_i = t; }
    }
    printf("max |dJ| over %d random cases = %.3e\n", N, worst);
    if (worst_i >= 0 && worst > 1e-5)
    {
        printf("FAIL（>1e-5）—— calc_M 的行列映射与源码不符\n");
        return 1;
    }
    printf("PASS：calc_M 与源码 arz/arx/ary 表达式逐项一致\n");
    return 0;
}
