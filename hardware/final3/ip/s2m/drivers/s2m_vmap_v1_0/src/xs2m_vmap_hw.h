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
// 0x10 : Data signal of q
//        bit 31~0 - q[31:0] (Read/Write)
// 0x14 : Data signal of q
//        bit 31~0 - q[63:32] (Read/Write)
// 0x18 : reserved
// 0x1c : Data signal of vbw
//        bit 31~0 - vbw[31:0] (Read/Write)
// 0x20 : Data signal of vbw
//        bit 31~0 - vbw[63:32] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of tab
//        bit 31~0 - tab[31:0] (Read/Write)
// 0x2c : Data signal of tab
//        bit 31~0 - tab[63:32] (Read/Write)
// 0x30 : reserved
// 0x34 : Data signal of parq
//        bit 31~0 - parq[31:0] (Read/Write)
// 0x38 : Data signal of parq
//        bit 31~0 - parq[63:32] (Read/Write)
// 0x3c : reserved
// 0x40 : Data signal of parm
//        bit 31~0 - parm[31:0] (Read/Write)
// 0x44 : Data signal of parm
//        bit 31~0 - parm[63:32] (Read/Write)
// 0x48 : reserved
// 0x4c : Data signal of Hb
//        bit 31~0 - Hb[31:0] (Read/Write)
// 0x50 : Data signal of Hb
//        bit 31~0 - Hb[63:32] (Read/Write)
// 0x54 : reserved
// 0x58 : Data signal of nq
//        bit 31~0 - nq[31:0] (Read/Write)
// 0x5c : reserved
// 0x60 : Data signal of mode
//        bit 31~0 - mode[31:0] (Read/Write)
// 0x64 : reserved
// 0x68 : Data signal of L
//        bit 31~0 - L[31:0] (Read/Write)
// 0x6c : reserved
// 0x70 : Data signal of invL
//        bit 31~0 - invL[31:0] (Read/Write)
// 0x74 : reserved
// 0x78 : Data signal of rmax
//        bit 31~0 - rmax[31:0] (Read/Write)
// 0x7c : reserved
// 0x80 : Data signal of orgx
//        bit 31~0 - orgx[31:0] (Read/Write)
// 0x84 : reserved
// 0x88 : Data signal of orgy
//        bit 31~0 - orgy[31:0] (Read/Write)
// 0x8c : reserved
// 0x90 : Data signal of orgz
//        bit 31~0 - orgz[31:0] (Read/Write)
// 0x94 : reserved
// 0x98 : Data signal of dimx
//        bit 31~0 - dimx[31:0] (Read/Write)
// 0x9c : reserved
// 0xa0 : Data signal of dimy
//        bit 31~0 - dimy[31:0] (Read/Write)
// 0xa4 : reserved
// 0xa8 : Data signal of dimz
//        bit 31~0 - dimz[31:0] (Read/Write)
// 0xac : reserved
// 0xb0 : Data signal of rescan
//        bit 31~0 - rescan[31:0] (Read/Write)
// 0xb4 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XS2M_VMAP_CONTROL_ADDR_AP_CTRL     0x00
#define XS2M_VMAP_CONTROL_ADDR_GIE         0x04
#define XS2M_VMAP_CONTROL_ADDR_IER         0x08
#define XS2M_VMAP_CONTROL_ADDR_ISR         0x0c
#define XS2M_VMAP_CONTROL_ADDR_Q_DATA      0x10
#define XS2M_VMAP_CONTROL_BITS_Q_DATA      64
#define XS2M_VMAP_CONTROL_ADDR_VBW_DATA    0x1c
#define XS2M_VMAP_CONTROL_BITS_VBW_DATA    64
#define XS2M_VMAP_CONTROL_ADDR_TAB_DATA    0x28
#define XS2M_VMAP_CONTROL_BITS_TAB_DATA    64
#define XS2M_VMAP_CONTROL_ADDR_PARQ_DATA   0x34
#define XS2M_VMAP_CONTROL_BITS_PARQ_DATA   64
#define XS2M_VMAP_CONTROL_ADDR_PARM_DATA   0x40
#define XS2M_VMAP_CONTROL_BITS_PARM_DATA   64
#define XS2M_VMAP_CONTROL_ADDR_HB_DATA     0x4c
#define XS2M_VMAP_CONTROL_BITS_HB_DATA     64
#define XS2M_VMAP_CONTROL_ADDR_NQ_DATA     0x58
#define XS2M_VMAP_CONTROL_BITS_NQ_DATA     32
#define XS2M_VMAP_CONTROL_ADDR_MODE_DATA   0x60
#define XS2M_VMAP_CONTROL_BITS_MODE_DATA   32
#define XS2M_VMAP_CONTROL_ADDR_L_DATA      0x68
#define XS2M_VMAP_CONTROL_BITS_L_DATA      32
#define XS2M_VMAP_CONTROL_ADDR_INVL_DATA   0x70
#define XS2M_VMAP_CONTROL_BITS_INVL_DATA   32
#define XS2M_VMAP_CONTROL_ADDR_RMAX_DATA   0x78
#define XS2M_VMAP_CONTROL_BITS_RMAX_DATA   32
#define XS2M_VMAP_CONTROL_ADDR_ORGX_DATA   0x80
#define XS2M_VMAP_CONTROL_BITS_ORGX_DATA   32
#define XS2M_VMAP_CONTROL_ADDR_ORGY_DATA   0x88
#define XS2M_VMAP_CONTROL_BITS_ORGY_DATA   32
#define XS2M_VMAP_CONTROL_ADDR_ORGZ_DATA   0x90
#define XS2M_VMAP_CONTROL_BITS_ORGZ_DATA   32
#define XS2M_VMAP_CONTROL_ADDR_DIMX_DATA   0x98
#define XS2M_VMAP_CONTROL_BITS_DIMX_DATA   32
#define XS2M_VMAP_CONTROL_ADDR_DIMY_DATA   0xa0
#define XS2M_VMAP_CONTROL_BITS_DIMY_DATA   32
#define XS2M_VMAP_CONTROL_ADDR_DIMZ_DATA   0xa8
#define XS2M_VMAP_CONTROL_BITS_DIMZ_DATA   32
#define XS2M_VMAP_CONTROL_ADDR_RESCAN_DATA 0xb0
#define XS2M_VMAP_CONTROL_BITS_RESCAN_DATA 32

