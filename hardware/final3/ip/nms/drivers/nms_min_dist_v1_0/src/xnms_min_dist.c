// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xnms_min_dist.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XNms_min_dist_CfgInitialize(XNms_min_dist *InstancePtr, XNms_min_dist_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XNms_min_dist_Start(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_AP_CTRL) & 0x80;
    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XNms_min_dist_IsDone(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XNms_min_dist_IsIdle(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XNms_min_dist_IsReady(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XNms_min_dist_EnableAutoRestart(XNms_min_dist *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XNms_min_dist_DisableAutoRestart(XNms_min_dist *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_AP_CTRL, 0);
}

void XNms_min_dist_Set_resp(XNms_min_dist *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_RESP_DATA, (u32)(Data));
    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_RESP_DATA + 4, (u32)(Data >> 32));
}

u64 XNms_min_dist_Get_resp(XNms_min_dist *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_RESP_DATA);
    Data += (u64)XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_RESP_DATA + 4) << 32;
    return Data;
}

void XNms_min_dist_Set_out_xy(XNms_min_dist *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_OUT_XY_DATA, (u32)(Data));
    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_OUT_XY_DATA + 4, (u32)(Data >> 32));
}

u64 XNms_min_dist_Get_out_xy(XNms_min_dist *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_OUT_XY_DATA);
    Data += (u64)XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_OUT_XY_DATA + 4) << 32;
    return Data;
}

void XNms_min_dist_Set_out_n(XNms_min_dist *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_OUT_N_DATA, (u32)(Data));
    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_OUT_N_DATA + 4, (u32)(Data >> 32));
}

u64 XNms_min_dist_Get_out_n(XNms_min_dist *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_OUT_N_DATA);
    Data += (u64)XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_OUT_N_DATA + 4) << 32;
    return Data;
}

void XNms_min_dist_Set_rows(XNms_min_dist *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_ROWS_DATA, Data);
}

u32 XNms_min_dist_Get_rows(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_ROWS_DATA);
    return Data;
}

void XNms_min_dist_Set_cols(XNms_min_dist *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_COLS_DATA, Data);
}

u32 XNms_min_dist_Get_cols(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_COLS_DATA);
    return Data;
}

void XNms_min_dist_Set_thr(XNms_min_dist *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_THR_DATA, Data);
}

u32 XNms_min_dist_Get_thr(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_THR_DATA);
    return Data;
}

void XNms_min_dist_Set_min_dist(XNms_min_dist *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_MIN_DIST_DATA, Data);
}

u32 XNms_min_dist_Get_min_dist(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_MIN_DIST_DATA);
    return Data;
}

void XNms_min_dist_Set_border(XNms_min_dist *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_BORDER_DATA, Data);
}

u32 XNms_min_dist_Get_border(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_BORDER_DATA);
    return Data;
}

void XNms_min_dist_Set_max_n(XNms_min_dist *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_MAX_N_DATA, Data);
}

u32 XNms_min_dist_Get_max_n(XNms_min_dist *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_MAX_N_DATA);
    return Data;
}

void XNms_min_dist_InterruptGlobalEnable(XNms_min_dist *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_GIE, 1);
}

void XNms_min_dist_InterruptGlobalDisable(XNms_min_dist *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_GIE, 0);
}

void XNms_min_dist_InterruptEnable(XNms_min_dist *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_IER);
    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_IER, Register | Mask);
}

void XNms_min_dist_InterruptDisable(XNms_min_dist *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_IER);
    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_IER, Register & (~Mask));
}

void XNms_min_dist_InterruptClear(XNms_min_dist *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XNms_min_dist_WriteReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_ISR, Mask);
}

u32 XNms_min_dist_InterruptGetEnabled(XNms_min_dist *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_IER);
}

u32 XNms_min_dist_InterruptGetStatus(XNms_min_dist *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XNms_min_dist_ReadReg(InstancePtr->Control_BaseAddress, XNMS_MIN_DIST_CONTROL_ADDR_ISR);
}

