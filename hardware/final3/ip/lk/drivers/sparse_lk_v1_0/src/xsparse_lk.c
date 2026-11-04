// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xsparse_lk.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XSparse_lk_CfgInitialize(XSparse_lk *InstancePtr, XSparse_lk_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XSparse_lk_Start(XSparse_lk *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_AP_CTRL) & 0x80;
    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XSparse_lk_IsDone(XSparse_lk *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XSparse_lk_IsIdle(XSparse_lk *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XSparse_lk_IsReady(XSparse_lk *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XSparse_lk_EnableAutoRestart(XSparse_lk *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XSparse_lk_DisableAutoRestart(XSparse_lk *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_AP_CTRL, 0);
}

void XSparse_lk_Set_img_prev(XSparse_lk *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IMG_PREV_DATA, (u32)(Data));
    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IMG_PREV_DATA + 4, (u32)(Data >> 32));
}

u64 XSparse_lk_Get_img_prev(XSparse_lk *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IMG_PREV_DATA);
    Data += (u64)XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IMG_PREV_DATA + 4) << 32;
    return Data;
}

void XSparse_lk_Set_img_cur(XSparse_lk *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IMG_CUR_DATA, (u32)(Data));
    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IMG_CUR_DATA + 4, (u32)(Data >> 32));
}

u64 XSparse_lk_Get_img_cur(XSparse_lk *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IMG_CUR_DATA);
    Data += (u64)XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IMG_CUR_DATA + 4) << 32;
    return Data;
}

void XSparse_lk_Set_corners(XSparse_lk *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_CORNERS_DATA, (u32)(Data));
    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_CORNERS_DATA + 4, (u32)(Data >> 32));
}

u64 XSparse_lk_Get_corners(XSparse_lk *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_CORNERS_DATA);
    Data += (u64)XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_CORNERS_DATA + 4) << 32;
    return Data;
}

void XSparse_lk_Set_flow_io(XSparse_lk *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_FLOW_IO_DATA, (u32)(Data));
    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_FLOW_IO_DATA + 4, (u32)(Data >> 32));
}

u64 XSparse_lk_Get_flow_io(XSparse_lk *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_FLOW_IO_DATA);
    Data += (u64)XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_FLOW_IO_DATA + 4) << 32;
    return Data;
}

void XSparse_lk_Set_status(XSparse_lk *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_STATUS_DATA, (u32)(Data));
    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_STATUS_DATA + 4, (u32)(Data >> 32));
}

u64 XSparse_lk_Get_status(XSparse_lk *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_STATUS_DATA);
    Data += (u64)XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_STATUS_DATA + 4) << 32;
    return Data;
}

void XSparse_lk_Set_N(XSparse_lk *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_N_DATA, Data);
}

u32 XSparse_lk_Get_N(XSparse_lk *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_N_DATA);
    return Data;
}

void XSparse_lk_Set_rows(XSparse_lk *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_ROWS_DATA, Data);
}

u32 XSparse_lk_Get_rows(XSparse_lk *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_ROWS_DATA);
    return Data;
}

void XSparse_lk_Set_cols(XSparse_lk *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_COLS_DATA, Data);
}

u32 XSparse_lk_Get_cols(XSparse_lk *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_COLS_DATA);
    return Data;
}

void XSparse_lk_Set_max_iter(XSparse_lk *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_MAX_ITER_DATA, Data);
}

u32 XSparse_lk_Get_max_iter(XSparse_lk *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_MAX_ITER_DATA);
    return Data;
}

void XSparse_lk_InterruptGlobalEnable(XSparse_lk *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_GIE, 1);
}

void XSparse_lk_InterruptGlobalDisable(XSparse_lk *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_GIE, 0);
}

void XSparse_lk_InterruptEnable(XSparse_lk *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IER);
    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IER, Register | Mask);
}

void XSparse_lk_InterruptDisable(XSparse_lk *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IER);
    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IER, Register & (~Mask));
}

void XSparse_lk_InterruptClear(XSparse_lk *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XSparse_lk_WriteReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_ISR, Mask);
}

u32 XSparse_lk_InterruptGetEnabled(XSparse_lk *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_IER);
}

u32 XSparse_lk_InterruptGetStatus(XSparse_lk *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XSparse_lk_ReadReg(InstancePtr->Control_BaseAddress, XSPARSE_LK_CONTROL_ADDR_ISR);
}

