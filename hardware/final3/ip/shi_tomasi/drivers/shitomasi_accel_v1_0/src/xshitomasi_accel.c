// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xshitomasi_accel.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XShitomasi_accel_CfgInitialize(XShitomasi_accel *InstancePtr, XShitomasi_accel_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XShitomasi_accel_Start(XShitomasi_accel *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_AP_CTRL) & 0x80;
    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XShitomasi_accel_IsDone(XShitomasi_accel *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XShitomasi_accel_IsIdle(XShitomasi_accel *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XShitomasi_accel_IsReady(XShitomasi_accel *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XShitomasi_accel_EnableAutoRestart(XShitomasi_accel *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XShitomasi_accel_DisableAutoRestart(XShitomasi_accel *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_AP_CTRL, 0);
}

void XShitomasi_accel_Set_img_in(XShitomasi_accel *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IMG_IN_DATA, (u32)(Data));
    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IMG_IN_DATA + 4, (u32)(Data >> 32));
}

u64 XShitomasi_accel_Get_img_in(XShitomasi_accel *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IMG_IN_DATA);
    Data += (u64)XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IMG_IN_DATA + 4) << 32;
    return Data;
}

void XShitomasi_accel_Set_img_out(XShitomasi_accel *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IMG_OUT_DATA, (u32)(Data));
    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IMG_OUT_DATA + 4, (u32)(Data >> 32));
}

u64 XShitomasi_accel_Get_img_out(XShitomasi_accel *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IMG_OUT_DATA);
    Data += (u64)XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IMG_OUT_DATA + 4) << 32;
    return Data;
}

void XShitomasi_accel_Set_rows(XShitomasi_accel *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_ROWS_DATA, Data);
}

u32 XShitomasi_accel_Get_rows(XShitomasi_accel *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_ROWS_DATA);
    return Data;
}

void XShitomasi_accel_Set_cols(XShitomasi_accel *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_COLS_DATA, Data);
}

u32 XShitomasi_accel_Get_cols(XShitomasi_accel *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_COLS_DATA);
    return Data;
}

void XShitomasi_accel_Set_threshold(XShitomasi_accel *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_THRESHOLD_DATA, Data);
}

u32 XShitomasi_accel_Get_threshold(XShitomasi_accel *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_THRESHOLD_DATA);
    return Data;
}

void XShitomasi_accel_Set_k(XShitomasi_accel *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_K_DATA, Data);
}

u32 XShitomasi_accel_Get_k(XShitomasi_accel *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_K_DATA);
    return Data;
}

void XShitomasi_accel_InterruptGlobalEnable(XShitomasi_accel *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_GIE, 1);
}

void XShitomasi_accel_InterruptGlobalDisable(XShitomasi_accel *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_GIE, 0);
}

void XShitomasi_accel_InterruptEnable(XShitomasi_accel *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IER);
    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IER, Register | Mask);
}

void XShitomasi_accel_InterruptDisable(XShitomasi_accel *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IER);
    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IER, Register & (~Mask));
}

void XShitomasi_accel_InterruptClear(XShitomasi_accel *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XShitomasi_accel_WriteReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_ISR, Mask);
}

u32 XShitomasi_accel_InterruptGetEnabled(XShitomasi_accel *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_IER);
}

u32 XShitomasi_accel_InterruptGetStatus(XShitomasi_accel *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XShitomasi_accel_ReadReg(InstancePtr->Control_BaseAddress, XSHITOMASI_ACCEL_CONTROL_ADDR_ISR);
}

