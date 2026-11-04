#ifndef _XF_SHITOMASI_CONFIG_H_
#define _XF_SHITOMASI_CONFIG_H_

#include "hls_stream.h"
#include "ap_int.h"
#include "common/xf_common.hpp"
#include "common/xf_utility.hpp"
#include "features/xf_harris.hpp"

#define WIDTH  640
#define HEIGHT 480
#define IMGSIZE WIDTH * HEIGHT
#define XF_CV_DEPTH_IN  2
#define XF_CV_DEPTH_OUT 2
#define MAXCORNERS 1024

#define FILTER_WIDTH 3
#define BLOCK_WIDTH  3
#define NMS_RADIUS   1

/* 8-bit response compression at the NMS output: resp = lam >> XF_RESP_SHIFT, clamped to 255.
   lam_max is ~1e3 for 8-bit imagery, so shift 2 nearly fills the 8-bit range. */
#ifndef XF_RESP_SHIFT
#define XF_RESP_SHIFT 2
#endif
#define XF_USE_URAM  0

#define NPPCX XF_NPPC1

#define IN_TYPE  XF_8UC1
#define OUT_TYPE XF_8UC1
#define CV_IN_TYPE  CV_8UC1
#define CV_OUT_TYPE CV_8UC1

/* 32-bit 指针 = 与现有 FAST IP 同接口（每字 4 个 8-bit 像素），Python 侧可零改动替换 */
#define INPUT_PTR_WIDTH  32
#define OUTPUT_PTR_WIDTH 32

void shitomasi_accel(ap_uint<INPUT_PTR_WIDTH>* img_in, ap_uint<OUTPUT_PTR_WIDTH>* img_out,
                     int rows, int cols, int threshold, int k);
#endif
