// Shi-Tomasi (min eigenvalue) 角点检测 —— 原型
// 骨架复用 Vitis Vision 官方 Harris 示例；response 部分待换成 min eigenvalue（见 utils 补丁）
// 接口按合并 bit 的约定：m_axi 一读一写【同 bundle gmem0】+ s_axilite
#include "xf_shitomasi_config.h"

static constexpr int __XF_DEPTH = (HEIGHT * WIDTH * (XF_PIXELWIDTH(IN_TYPE, NPPCX)) / 8) / (INPUT_PTR_WIDTH / 8);

void shitomasi_accel(ap_uint<INPUT_PTR_WIDTH>* img_in, ap_uint<OUTPUT_PTR_WIDTH>* img_out,
                     int rows, int cols, int threshold, int k) {
#pragma HLS INTERFACE m_axi     port=img_in  offset=slave bundle=gmem0 depth=__XF_DEPTH
#pragma HLS INTERFACE m_axi     port=img_out offset=slave bundle=gmem0 depth=__XF_DEPTH
#pragma HLS INTERFACE s_axilite port=rows
#pragma HLS INTERFACE s_axilite port=cols
#pragma HLS INTERFACE s_axilite port=threshold
#pragma HLS INTERFACE s_axilite port=k
#pragma HLS INTERFACE s_axilite port=return

    xf::cv::Mat<IN_TYPE, HEIGHT, WIDTH, NPPCX, XF_CV_DEPTH_IN> in_mat(rows, cols);
    xf::cv::Mat<OUT_TYPE, HEIGHT, WIDTH, NPPCX, XF_CV_DEPTH_OUT> out_mat(rows, cols);

#pragma HLS DATAFLOW
    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, IN_TYPE, HEIGHT, WIDTH, NPPCX, XF_CV_DEPTH_IN>(img_in, in_mat);
    xf::cv::cornerHarris<FILTER_WIDTH, BLOCK_WIDTH, NMS_RADIUS, IN_TYPE, HEIGHT, WIDTH, NPPCX, XF_USE_URAM,
                         XF_CV_DEPTH_IN, XF_CV_DEPTH_OUT>(in_mat, out_mat, threshold, k);
    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, OUT_TYPE, HEIGHT, WIDTH, NPPCX, XF_CV_DEPTH_OUT>(out_mat, img_out);
}
