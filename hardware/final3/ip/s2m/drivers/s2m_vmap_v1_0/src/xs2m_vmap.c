// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
/***************************** Include Files *********************************/
#include "xs2m_vmap.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XS2m_vmap_CfgInitialize(XS2m_vmap *InstancePtr, XS2m_vmap_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XS2m_vmap_Start(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_AP_CTRL) & 0x80;
    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XS2m_vmap_IsDone(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XS2m_vmap_IsIdle(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XS2m_vmap_IsReady(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XS2m_vmap_EnableAutoRestart(XS2m_vmap *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XS2m_vmap_DisableAutoRestart(XS2m_vmap *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_AP_CTRL, 0);
}

void XS2m_vmap_Set_q(XS2m_vmap *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_Q_DATA, (u32)(Data));
    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_Q_DATA + 4, (u32)(Data >> 32));
}

u64 XS2m_vmap_Get_q(XS2m_vmap *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_Q_DATA);
    Data += (u64)XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_Q_DATA + 4) << 32;
    return Data;
}

void XS2m_vmap_Set_vbw(XS2m_vmap *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_VBW_DATA, (u32)(Data));
    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_VBW_DATA + 4, (u32)(Data >> 32));
}

u64 XS2m_vmap_Get_vbw(XS2m_vmap *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_VBW_DATA);
    Data += (u64)XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_VBW_DATA + 4) << 32;
    return Data;
}

void XS2m_vmap_Set_tab(XS2m_vmap *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_TAB_DATA, (u32)(Data));
    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_TAB_DATA + 4, (u32)(Data >> 32));
}

u64 XS2m_vmap_Get_tab(XS2m_vmap *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_TAB_DATA);
    Data += (u64)XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_TAB_DATA + 4) << 32;
    return Data;
}

void XS2m_vmap_Set_parq(XS2m_vmap *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_PARQ_DATA, (u32)(Data));
    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_PARQ_DATA + 4, (u32)(Data >> 32));
}

u64 XS2m_vmap_Get_parq(XS2m_vmap *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_PARQ_DATA);
    Data += (u64)XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_PARQ_DATA + 4) << 32;
    return Data;
}

void XS2m_vmap_Set_parm(XS2m_vmap *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_PARM_DATA, (u32)(Data));
    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_PARM_DATA + 4, (u32)(Data >> 32));
}

u64 XS2m_vmap_Get_parm(XS2m_vmap *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_PARM_DATA);
    Data += (u64)XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_PARM_DATA + 4) << 32;
    return Data;
}

void XS2m_vmap_Set_Hb(XS2m_vmap *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_HB_DATA, (u32)(Data));
    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_HB_DATA + 4, (u32)(Data >> 32));
}

u64 XS2m_vmap_Get_Hb(XS2m_vmap *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_HB_DATA);
    Data += (u64)XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_HB_DATA + 4) << 32;
    return Data;
}

void XS2m_vmap_Set_nq(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_NQ_DATA, Data);
}

u32 XS2m_vmap_Get_nq(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_NQ_DATA);
    return Data;
}

void XS2m_vmap_Set_mode(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_MODE_DATA, Data);
}

u32 XS2m_vmap_Get_mode(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_MODE_DATA);
    return Data;
}

void XS2m_vmap_Set_L(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_L_DATA, Data);
}

u32 XS2m_vmap_Get_L(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_L_DATA);
    return Data;
}

void XS2m_vmap_Set_invL(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_INVL_DATA, Data);
}

u32 XS2m_vmap_Get_invL(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_INVL_DATA);
    return Data;
}

void XS2m_vmap_Set_rmax(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_RMAX_DATA, Data);
}

u32 XS2m_vmap_Get_rmax(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_RMAX_DATA);
    return Data;
}

void XS2m_vmap_Set_orgx(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_ORGX_DATA, Data);
}

u32 XS2m_vmap_Get_orgx(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_ORGX_DATA);
    return Data;
}

void XS2m_vmap_Set_orgy(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_ORGY_DATA, Data);
}

u32 XS2m_vmap_Get_orgy(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_ORGY_DATA);
    return Data;
}

void XS2m_vmap_Set_orgz(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_ORGZ_DATA, Data);
}

u32 XS2m_vmap_Get_orgz(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_ORGZ_DATA);
    return Data;
}

void XS2m_vmap_Set_dimx(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_DIMX_DATA, Data);
}

u32 XS2m_vmap_Get_dimx(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_DIMX_DATA);
    return Data;
}

void XS2m_vmap_Set_dimy(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_DIMY_DATA, Data);
}

u32 XS2m_vmap_Get_dimy(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_DIMY_DATA);
    return Data;
}

void XS2m_vmap_Set_dimz(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_DIMZ_DATA, Data);
}

u32 XS2m_vmap_Get_dimz(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_DIMZ_DATA);
    return Data;
}

void XS2m_vmap_Set_rescan(XS2m_vmap *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_RESCAN_DATA, Data);
}

u32 XS2m_vmap_Get_rescan(XS2m_vmap *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_RESCAN_DATA);
    return Data;
}

void XS2m_vmap_InterruptGlobalEnable(XS2m_vmap *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_GIE, 1);
}

void XS2m_vmap_InterruptGlobalDisable(XS2m_vmap *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_GIE, 0);
}

void XS2m_vmap_InterruptEnable(XS2m_vmap *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_IER);
    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_IER, Register | Mask);
}

void XS2m_vmap_InterruptDisable(XS2m_vmap *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_IER);
    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_IER, Register & (~Mask));
}

void XS2m_vmap_InterruptClear(XS2m_vmap *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XS2m_vmap_WriteReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_ISR, Mask);
}

u32 XS2m_vmap_InterruptGetEnabled(XS2m_vmap *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_IER);
}

u32 XS2m_vmap_InterruptGetStatus(XS2m_vmap *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XS2m_vmap_ReadReg(InstancePtr->Control_BaseAddress, XS2M_VMAP_CONTROL_ADDR_ISR);
}

