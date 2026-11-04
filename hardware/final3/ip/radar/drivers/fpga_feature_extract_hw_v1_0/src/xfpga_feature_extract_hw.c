// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xfpga_feature_extract_hw.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XFpga_feature_extract_hw_CfgInitialize(XFpga_feature_extract_hw *InstancePtr, XFpga_feature_extract_hw_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_bus_BaseAddress = ConfigPtr->Control_bus_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XFpga_feature_extract_hw_Start(XFpga_feature_extract_hw *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_AP_CTRL) & 0x80;
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_AP_CTRL, Data | 0x01);
}

u32 XFpga_feature_extract_hw_IsDone(XFpga_feature_extract_hw *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XFpga_feature_extract_hw_IsIdle(XFpga_feature_extract_hw *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XFpga_feature_extract_hw_IsReady(XFpga_feature_extract_hw *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XFpga_feature_extract_hw_EnableAutoRestart(XFpga_feature_extract_hw *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_AP_CTRL, 0x80);
}

void XFpga_feature_extract_hw_DisableAutoRestart(XFpga_feature_extract_hw *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_AP_CTRL, 0);
}

void XFpga_feature_extract_hw_Set_laserCloudInArray(XFpga_feature_extract_hw *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_LASERCLOUDINARRAY_DATA, (u32)(Data));
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_LASERCLOUDINARRAY_DATA + 4, (u32)(Data >> 32));
}

u64 XFpga_feature_extract_hw_Get_laserCloudInArray(XFpga_feature_extract_hw *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_LASERCLOUDINARRAY_DATA);
    Data += (u64)XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_LASERCLOUDINARRAY_DATA + 4) << 32;
    return Data;
}

void XFpga_feature_extract_hw_Set_cloudSize(XFpga_feature_extract_hw *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_CLOUDSIZE_DATA, Data);
}

u32 XFpga_feature_extract_hw_Get_cloudSize(XFpga_feature_extract_hw *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_CLOUDSIZE_DATA);
    return Data;
}

void XFpga_feature_extract_hw_Set_start_point_ori(XFpga_feature_extract_hw *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_START_POINT_ORI_DATA, Data);
}

u32 XFpga_feature_extract_hw_Get_start_point_ori(XFpga_feature_extract_hw *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_START_POINT_ORI_DATA);
    return Data;
}

void XFpga_feature_extract_hw_Set_end_point_ori(XFpga_feature_extract_hw *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_END_POINT_ORI_DATA, Data);
}

u32 XFpga_feature_extract_hw_Get_end_point_ori(XFpga_feature_extract_hw *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_END_POINT_ORI_DATA);
    return Data;
}

void XFpga_feature_extract_hw_Set_rangeimage_point(XFpga_feature_extract_hw *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_POINT_DATA, (u32)(Data));
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_POINT_DATA + 4, (u32)(Data >> 32));
}

u64 XFpga_feature_extract_hw_Get_rangeimage_point(XFpga_feature_extract_hw *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_POINT_DATA);
    Data += (u64)XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_POINT_DATA + 4) << 32;
    return Data;
}

void XFpga_feature_extract_hw_Set_rangeimage_range(XFpga_feature_extract_hw *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_RANGE_DATA, (u32)(Data));
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_RANGE_DATA + 4, (u32)(Data >> 32));
}

u64 XFpga_feature_extract_hw_Get_rangeimage_range(XFpga_feature_extract_hw *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_RANGE_DATA);
    Data += (u64)XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_RANGE_DATA + 4) << 32;
    return Data;
}

void XFpga_feature_extract_hw_Set_rangeimage_lessflat(XFpga_feature_extract_hw *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSFLAT_DATA, (u32)(Data));
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSFLAT_DATA + 4, (u32)(Data >> 32));
}

u64 XFpga_feature_extract_hw_Get_rangeimage_lessflat(XFpga_feature_extract_hw *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSFLAT_DATA);
    Data += (u64)XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSFLAT_DATA + 4) << 32;
    return Data;
}

void XFpga_feature_extract_hw_Set_rangeimage_ground(XFpga_feature_extract_hw *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_GROUND_DATA, (u32)(Data));
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_GROUND_DATA + 4, (u32)(Data >> 32));
}

u64 XFpga_feature_extract_hw_Get_rangeimage_ground(XFpga_feature_extract_hw *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_GROUND_DATA);
    Data += (u64)XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_GROUND_DATA + 4) << 32;
    return Data;
}

void XFpga_feature_extract_hw_Set_rangeimage_flat(XFpga_feature_extract_hw *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_FLAT_DATA, (u32)(Data));
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_FLAT_DATA + 4, (u32)(Data >> 32));
}

u64 XFpga_feature_extract_hw_Get_rangeimage_flat(XFpga_feature_extract_hw *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_FLAT_DATA);
    Data += (u64)XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_FLAT_DATA + 4) << 32;
    return Data;
}

void XFpga_feature_extract_hw_Set_rangeimage_lesssharp(XFpga_feature_extract_hw *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSSHARP_DATA, (u32)(Data));
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSSHARP_DATA + 4, (u32)(Data >> 32));
}

u64 XFpga_feature_extract_hw_Get_rangeimage_lesssharp(XFpga_feature_extract_hw *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSSHARP_DATA);
    Data += (u64)XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_LESSSHARP_DATA + 4) << 32;
    return Data;
}

void XFpga_feature_extract_hw_Set_rangeimage_sharp(XFpga_feature_extract_hw *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_SHARP_DATA, (u32)(Data));
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_SHARP_DATA + 4, (u32)(Data >> 32));
}

u64 XFpga_feature_extract_hw_Get_rangeimage_sharp(XFpga_feature_extract_hw *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_SHARP_DATA);
    Data += (u64)XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_RANGEIMAGE_SHARP_DATA + 4) << 32;
    return Data;
}

void XFpga_feature_extract_hw_InterruptGlobalEnable(XFpga_feature_extract_hw *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_GIE, 1);
}

void XFpga_feature_extract_hw_InterruptGlobalDisable(XFpga_feature_extract_hw *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_GIE, 0);
}

void XFpga_feature_extract_hw_InterruptEnable(XFpga_feature_extract_hw *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_IER);
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_IER, Register | Mask);
}

void XFpga_feature_extract_hw_InterruptDisable(XFpga_feature_extract_hw *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_IER);
    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_IER, Register & (~Mask));
}

void XFpga_feature_extract_hw_InterruptClear(XFpga_feature_extract_hw *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XFpga_feature_extract_hw_WriteReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_ISR, Mask);
}

u32 XFpga_feature_extract_hw_InterruptGetEnabled(XFpga_feature_extract_hw *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_IER);
}

u32 XFpga_feature_extract_hw_InterruptGetStatus(XFpga_feature_extract_hw *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XFpga_feature_extract_hw_ReadReg(InstancePtr->Control_bus_BaseAddress, XFPGA_FEATURE_EXTRACT_HW_CONTROL_BUS_ADDR_ISR);
}

