// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
// control_bus
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
// 0x10 : Data signal of laserCloudInArray
//        bit 31~0 - laserCloudInArray[31:0] (Read/Write)
// 0x14 : Data signal of laserCloudInArray
//        bit 31~0 - laserCloudInArray[63:32] (Read/Write)
// 0x18 : reserved
// 0x1c : Data signal of cloudSize
//        bit 31~0 - cloudSize[31:0] (Read/Write)
// 0x20 : reserved
// 0x24 : Data signal of start_point_ori
//        bit 31~0 - start_point_ori[31:0] (Read/Write)
// 0x28 : reserved
// 0x2c : Data signal of end_point_ori
//        bit 31~0 - end_point_ori[31:0] (Read/Write)
// 0x30 : reserved
// 0x34 : Data signal of rangeimage_point
//        bit 31~0 - rangeimage_point[31:0] (Read/Write)
// 0x38 : Data signal of rangeimage_point
//        bit 31~0 - rangeimage_point[63:32] (Read/Write)
// 0x3c : reserved
// 0x40 : Data signal of rangeimage_range
//        bit 31~0 - rangeimage_range[31:0] (Read/Write)
// 0x44 : Data signal of rangeimage_range
//        bit 31~0 - rangeimage_range[63:32] (Read/Write)
// 0x48 : reserved
// 0x4c : Data signal of rangeimage_lessflat
//        bit 31~0 - rangeimage_lessflat[31:0] (Read/Write)
// 0x50 : Data signal of rangeimage_lessflat
//        bit 31~0 - rangeimage_lessflat[63:32] (Read/Write)
// 0x54 : reserved
// 0x58 : Data signal of rangeimage_ground
//        bit 31~0 - rangeimage_ground[31:0] (Read/Write)
// 0x5c : Data signal of rangeimage_ground
//        bit 31~0 - rangeimage_ground[63:32] (Read/Write)
// 0x60 : reserved
// 0x64 : Data signal of rangeimage_flat
//        bit 31~0 - rangeimage_flat[31:0] (Read/Write)
// 0x68 : Data signal of rangeimage_flat
//        bit 31~0 - rangeimage_flat[63:32] (Read/Write)
// 0x6c : reserved
// 0x70 : Data signal of rangeimage_lesssharp
//        bit 31~0 - rangeimage_lesssharp[31:0] (Read/Write)
// 0x74 : Data signal of rangeimage_lesssharp
//        bit 31~0 - rangeimage_lesssharp[63:32] (Read/Write)
// 0x78 : reserved
// 0x7c : Data signal of rangeimage_sharp
//        bit 31~0 - rangeimage_sharp[31:0] (Read/Write)
// 0x80 : Data signal of rangeimage_sharp
//        bit 31~0 - rangeimage_sharp[63:32] (Read/Write)
// 0x84 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_AP_CTRL                   0x00
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_GIE                       0x04
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_IER                       0x08
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_ISR                       0x0c
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_LASERCLOUDINARRAY_DATA    0x10
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_LASERCLOUDINARRAY_DATA    64
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_CLOUDSIZE_DATA            0x1c
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_CLOUDSIZE_DATA            32
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_START_POINT_ORI_DATA      0x24
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_START_POINT_ORI_DATA      32
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_END_POINT_ORI_DATA        0x2c
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_END_POINT_ORI_DATA        32
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_POINT_DATA     0x34
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_RANGEIMAGE_POINT_DATA     64
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_RANGE_DATA     0x40
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_RANGEIMAGE_RANGE_DATA     64
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSFLAT_DATA  0x4c
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_RANGEIMAGE_LESSFLAT_DATA  64
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_GROUND_DATA    0x58
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_RANGEIMAGE_GROUND_DATA    64
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_FLAT_DATA      0x64
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_RANGEIMAGE_FLAT_DATA      64
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSSHARP_DATA 0x70
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_RANGEIMAGE_LESSSHARP_DATA 64
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_SHARP_DATA     0x7c
#define XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_BITS_RANGEIMAGE_SHARP_DATA     64

