// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XNMS_MIN_DIST_H
#define XNMS_MIN_DIST_H

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
#include "xnms_min_dist_hw.h"

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
} XNms_min_dist_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XNms_min_dist;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XNms_min_dist_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XNms_min_dist_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XNms_min_dist_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XNms_min_dist_ReadReg(BaseAddress, RegOffset) \
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
int XNms_min_dist_Initialize(XNms_min_dist *InstancePtr, UINTPTR BaseAddress);
XNms_min_dist_Config* XNms_min_dist_LookupConfig(UINTPTR BaseAddress);
#else
int XNms_min_dist_Initialize(XNms_min_dist *InstancePtr, u16 DeviceId);
XNms_min_dist_Config* XNms_min_dist_LookupConfig(u16 DeviceId);
#endif
int XNms_min_dist_CfgInitialize(XNms_min_dist *InstancePtr, XNms_min_dist_Config *ConfigPtr);
#else
int XNms_min_dist_Initialize(XNms_min_dist *InstancePtr, const char* InstanceName);
int XNms_min_dist_Release(XNms_min_dist *InstancePtr);
#endif

void XNms_min_dist_Start(XNms_min_dist *InstancePtr);
u32 XNms_min_dist_IsDone(XNms_min_dist *InstancePtr);
u32 XNms_min_dist_IsIdle(XNms_min_dist *InstancePtr);
u32 XNms_min_dist_IsReady(XNms_min_dist *InstancePtr);
void XNms_min_dist_EnableAutoRestart(XNms_min_dist *InstancePtr);
void XNms_min_dist_DisableAutoRestart(XNms_min_dist *InstancePtr);

void XNms_min_dist_Set_resp(XNms_min_dist *InstancePtr, u64 Data);
u64 XNms_min_dist_Get_resp(XNms_min_dist *InstancePtr);
void XNms_min_dist_Set_out_xy(XNms_min_dist *InstancePtr, u64 Data);
u64 XNms_min_dist_Get_out_xy(XNms_min_dist *InstancePtr);
void XNms_min_dist_Set_out_n(XNms_min_dist *InstancePtr, u64 Data);
u64 XNms_min_dist_Get_out_n(XNms_min_dist *InstancePtr);
void XNms_min_dist_Set_rows(XNms_min_dist *InstancePtr, u32 Data);
u32 XNms_min_dist_Get_rows(XNms_min_dist *InstancePtr);
void XNms_min_dist_Set_cols(XNms_min_dist *InstancePtr, u32 Data);
u32 XNms_min_dist_Get_cols(XNms_min_dist *InstancePtr);
void XNms_min_dist_Set_thr(XNms_min_dist *InstancePtr, u32 Data);
u32 XNms_min_dist_Get_thr(XNms_min_dist *InstancePtr);
void XNms_min_dist_Set_min_dist(XNms_min_dist *InstancePtr, u32 Data);
u32 XNms_min_dist_Get_min_dist(XNms_min_dist *InstancePtr);
void XNms_min_dist_Set_border(XNms_min_dist *InstancePtr, u32 Data);
u32 XNms_min_dist_Get_border(XNms_min_dist *InstancePtr);
void XNms_min_dist_Set_max_n(XNms_min_dist *InstancePtr, u32 Data);
u32 XNms_min_dist_Get_max_n(XNms_min_dist *InstancePtr);

void XNms_min_dist_InterruptGlobalEnable(XNms_min_dist *InstancePtr);
void XNms_min_dist_InterruptGlobalDisable(XNms_min_dist *InstancePtr);
void XNms_min_dist_InterruptEnable(XNms_min_dist *InstancePtr, u32 Mask);
void XNms_min_dist_InterruptDisable(XNms_min_dist *InstancePtr, u32 Mask);
void XNms_min_dist_InterruptClear(XNms_min_dist *InstancePtr, u32 Mask);
u32 XNms_min_dist_InterruptGetEnabled(XNms_min_dist *InstancePtr);
u32 XNms_min_dist_InterruptGetStatus(XNms_min_dist *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
