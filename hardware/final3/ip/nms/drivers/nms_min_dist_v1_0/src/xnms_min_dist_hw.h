// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
// control
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read/COR)
//        bit 7  - auto_restart (Read/Write)
//        bit 9  - interrupt (Read)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0 - enable ap_done interrupt (Read/Write)
//        bit 1 - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0 - ap_done (Read/TOW)
//        bit 1 - ap_ready (Read/TOW)
//        others - reserved
// 0x10 : Data signal of resp
//        bit 31~0 - resp[31:0] (Read/Write)
// 0x14 : Data signal of resp
//        bit 31~0 - resp[63:32] (Read/Write)
// 0x18 : reserved
// 0x1c : Data signal of out_xy
//        bit 31~0 - out_xy[31:0] (Read/Write)
// 0x20 : Data signal of out_xy
//        bit 31~0 - out_xy[63:32] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of out_n
//        bit 31~0 - out_n[31:0] (Read/Write)
// 0x2c : Data signal of out_n
//        bit 31~0 - out_n[63:32] (Read/Write)
// 0x30 : reserved
// 0x34 : Data signal of rows
//        bit 31~0 - rows[31:0] (Read/Write)
// 0x38 : reserved
// 0x3c : Data signal of cols
//        bit 31~0 - cols[31:0] (Read/Write)
// 0x40 : reserved
// 0x44 : Data signal of thr
//        bit 31~0 - thr[31:0] (Read/Write)
// 0x48 : reserved
// 0x4c : Data signal of min_dist
//        bit 31~0 - min_dist[31:0] (Read/Write)
// 0x50 : reserved
// 0x54 : Data signal of border
//        bit 31~0 - border[31:0] (Read/Write)
// 0x58 : reserved
// 0x5c : Data signal of max_n
//        bit 31~0 - max_n[31:0] (Read/Write)
// 0x60 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XNMS_MIN_DIST_CONTROL_ADDR_AP_CTRL       0x00
#define XNMS_MIN_DIST_CONTROL_ADDR_GIE           0x04
#define XNMS_MIN_DIST_CONTROL_ADDR_IER           0x08
#define XNMS_MIN_DIST_CONTROL_ADDR_ISR           0x0c
#define XNMS_MIN_DIST_CONTROL_ADDR_RESP_DATA     0x10
#define XNMS_MIN_DIST_CONTROL_BITS_RESP_DATA     64
#define XNMS_MIN_DIST_CONTROL_ADDR_OUT_XY_DATA   0x1c
#define XNMS_MIN_DIST_CONTROL_BITS_OUT_XY_DATA   64
#define XNMS_MIN_DIST_CONTROL_ADDR_OUT_N_DATA    0x28
#define XNMS_MIN_DIST_CONTROL_BITS_OUT_N_DATA    64
#define XNMS_MIN_DIST_CONTROL_ADDR_ROWS_DATA     0x34
#define XNMS_MIN_DIST_CONTROL_BITS_ROWS_DATA     32
#define XNMS_MIN_DIST_CONTROL_ADDR_COLS_DATA     0x3c
#define XNMS_MIN_DIST_CONTROL_BITS_COLS_DATA     32
#define XNMS_MIN_DIST_CONTROL_ADDR_THR_DATA      0x44
#define XNMS_MIN_DIST_CONTROL_BITS_THR_DATA      32
#define XNMS_MIN_DIST_CONTROL_ADDR_MIN_DIST_DATA 0x4c
#define XNMS_MIN_DIST_CONTROL_BITS_MIN_DIST_DATA 32
#define XNMS_MIN_DIST_CONTROL_ADDR_BORDER_DATA   0x54
#define XNMS_MIN_DIST_CONTROL_BITS_BORDER_DATA   32
#define XNMS_MIN_DIST_CONTROL_ADDR_MAX_N_DATA    0x5c
#define XNMS_MIN_DIST_CONTROL_BITS_MAX_N_DATA    32

