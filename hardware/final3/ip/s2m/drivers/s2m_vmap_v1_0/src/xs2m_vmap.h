// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XS2M_VMAP_H
#define XS2M_VMAP_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xs2m_vmap_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
#ifdef SDT
    char *Name;
#else
    u16 DeviceId;
#endif
    u64 Control_BaseAddress;
} XS2m_vmap_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XS2m_vmap;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XS2m_vmap_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XS2m_vmap_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XS2m_vmap_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XS2m_vmap_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
#ifdef SDT
int XS2m_vmap_Initialize(XS2m_vmap *InstancePtr, UINTPTR BaseAddress);
XS2m_vmap_Config* XS2m_vmap_LookupConfig(UINTPTR BaseAddress);
#else
int XS2m_vmap_Initialize(XS2m_vmap *InstancePtr, u16 DeviceId);
XS2m_vmap_Config* XS2m_vmap_LookupConfig(u16 DeviceId);
#endif
int XS2m_vmap_CfgInitialize(XS2m_vmap *InstancePtr, XS2m_vmap_Config *ConfigPtr);
#else
int XS2m_vmap_Initialize(XS2m_vmap *InstancePtr, const char* InstanceName);
int XS2m_vmap_Release(XS2m_vmap *InstancePtr);
#endif

void XS2m_vmap_Start(XS2m_vmap *InstancePtr);
u32 XS2m_vmap_IsDone(XS2m_vmap *InstancePtr);
u32 XS2m_vmap_IsIdle(XS2m_vmap *InstancePtr);
u32 XS2m_vmap_IsReady(XS2m_vmap *InstancePtr);
void XS2m_vmap_EnableAutoRestart(XS2m_vmap *InstancePtr);
void XS2m_vmap_DisableAutoRestart(XS2m_vmap *InstancePtr);

void XS2m_vmap_Set_q(XS2m_vmap *InstancePtr, u64 Data);
u64 XS2m_vmap_Get_q(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_vbw(XS2m_vmap *InstancePtr, u64 Data);
u64 XS2m_vmap_Get_vbw(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_tab(XS2m_vmap *InstancePtr, u64 Data);
u64 XS2m_vmap_Get_tab(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_parq(XS2m_vmap *InstancePtr, u64 Data);
u64 XS2m_vmap_Get_parq(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_parm(XS2m_vmap *InstancePtr, u64 Data);
u64 XS2m_vmap_Get_parm(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_Hb(XS2m_vmap *InstancePtr, u64 Data);
u64 XS2m_vmap_Get_Hb(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_nq(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_nq(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_mode(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_mode(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_L(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_L(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_invL(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_invL(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_rmax(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_rmax(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_orgx(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_orgx(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_orgy(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_orgy(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_orgz(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_orgz(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_dimx(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_dimx(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_dimy(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_dimy(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_dimz(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_dimz(XS2m_vmap *InstancePtr);
void XS2m_vmap_Set_rescan(XS2m_vmap *InstancePtr, u32 Data);
u32 XS2m_vmap_Get_rescan(XS2m_vmap *InstancePtr);

void XS2m_vmap_InterruptGlobalEnable(XS2m_vmap *InstancePtr);
void XS2m_vmap_InterruptGlobalDisable(XS2m_vmap *InstancePtr);
void XS2m_vmap_InterruptEnable(XS2m_vmap *InstancePtr, u32 Mask);
void XS2m_vmap_InterruptDisable(XS2m_vmap *InstancePtr, u32 Mask);
void XS2m_vmap_InterruptClear(XS2m_vmap *InstancePtr, u32 Mask);
u32 XS2m_vmap_InterruptGetEnabled(XS2m_vmap *InstancePtr);
u32 XS2m_vmap_InterruptGetStatus(XS2m_vmap *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
