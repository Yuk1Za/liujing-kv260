// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XSPARSE_LK_H
#define XSPARSE_LK_H

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
#include "xsparse_lk_hw.h"

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
} XSparse_lk_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XSparse_lk;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XSparse_lk_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XSparse_lk_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XSparse_lk_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XSparse_lk_ReadReg(BaseAddress, RegOffset) \
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
int XSparse_lk_Initialize(XSparse_lk *InstancePtr, UINTPTR BaseAddress);
XSparse_lk_Config* XSparse_lk_LookupConfig(UINTPTR BaseAddress);
#else
int XSparse_lk_Initialize(XSparse_lk *InstancePtr, u16 DeviceId);
XSparse_lk_Config* XSparse_lk_LookupConfig(u16 DeviceId);
#endif
int XSparse_lk_CfgInitialize(XSparse_lk *InstancePtr, XSparse_lk_Config *ConfigPtr);
#else
int XSparse_lk_Initialize(XSparse_lk *InstancePtr, const char* InstanceName);
int XSparse_lk_Release(XSparse_lk *InstancePtr);
#endif

void XSparse_lk_Start(XSparse_lk *InstancePtr);
u32 XSparse_lk_IsDone(XSparse_lk *InstancePtr);
u32 XSparse_lk_IsIdle(XSparse_lk *InstancePtr);
u32 XSparse_lk_IsReady(XSparse_lk *InstancePtr);
void XSparse_lk_EnableAutoRestart(XSparse_lk *InstancePtr);
void XSparse_lk_DisableAutoRestart(XSparse_lk *InstancePtr);

void XSparse_lk_Set_img_prev(XSparse_lk *InstancePtr, u64 Data);
u64 XSparse_lk_Get_img_prev(XSparse_lk *InstancePtr);
void XSparse_lk_Set_img_cur(XSparse_lk *InstancePtr, u64 Data);
u64 XSparse_lk_Get_img_cur(XSparse_lk *InstancePtr);
void XSparse_lk_Set_corners(XSparse_lk *InstancePtr, u64 Data);
u64 XSparse_lk_Get_corners(XSparse_lk *InstancePtr);
void XSparse_lk_Set_flow_io(XSparse_lk *InstancePtr, u64 Data);
u64 XSparse_lk_Get_flow_io(XSparse_lk *InstancePtr);
void XSparse_lk_Set_status(XSparse_lk *InstancePtr, u64 Data);
u64 XSparse_lk_Get_status(XSparse_lk *InstancePtr);
void XSparse_lk_Set_N(XSparse_lk *InstancePtr, u32 Data);
u32 XSparse_lk_Get_N(XSparse_lk *InstancePtr);
void XSparse_lk_Set_rows(XSparse_lk *InstancePtr, u32 Data);
u32 XSparse_lk_Get_rows(XSparse_lk *InstancePtr);
void XSparse_lk_Set_cols(XSparse_lk *InstancePtr, u32 Data);
u32 XSparse_lk_Get_cols(XSparse_lk *InstancePtr);
void XSparse_lk_Set_max_iter(XSparse_lk *InstancePtr, u32 Data);
u32 XSparse_lk_Get_max_iter(XSparse_lk *InstancePtr);

void XSparse_lk_InterruptGlobalEnable(XSparse_lk *InstancePtr);
void XSparse_lk_InterruptGlobalDisable(XSparse_lk *InstancePtr);
void XSparse_lk_InterruptEnable(XSparse_lk *InstancePtr, u32 Mask);
void XSparse_lk_InterruptDisable(XSparse_lk *InstancePtr, u32 Mask);
void XSparse_lk_InterruptClear(XSparse_lk *InstancePtr, u32 Mask);
u32 XSparse_lk_InterruptGetEnabled(XSparse_lk *InstancePtr);
u32 XSparse_lk_InterruptGetStatus(XSparse_lk *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
