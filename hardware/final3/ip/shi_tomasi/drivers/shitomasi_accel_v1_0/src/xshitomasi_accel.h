// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2023.2 (64-bit)
// Tool Version Limit: 2023.10
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XSHITOMASI_ACCEL_H
#define XSHITOMASI_ACCEL_H

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
#include "xshitomasi_accel_hw.h"

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
} XShitomasi_accel_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XShitomasi_accel;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XShitomasi_accel_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XShitomasi_accel_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XShitomasi_accel_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XShitomasi_accel_ReadReg(BaseAddress, RegOffset) \
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
int XShitomasi_accel_Initialize(XShitomasi_accel *InstancePtr, UINTPTR BaseAddress);
XShitomasi_accel_Config* XShitomasi_accel_LookupConfig(UINTPTR BaseAddress);
#else
int XShitomasi_accel_Initialize(XShitomasi_accel *InstancePtr, u16 DeviceId);
XShitomasi_accel_Config* XShitomasi_accel_LookupConfig(u16 DeviceId);
#endif
int XShitomasi_accel_CfgInitialize(XShitomasi_accel *InstancePtr, XShitomasi_accel_Config *ConfigPtr);
#else
int XShitomasi_accel_Initialize(XShitomasi_accel *InstancePtr, const char* InstanceName);
int XShitomasi_accel_Release(XShitomasi_accel *InstancePtr);
#endif

void XShitomasi_accel_Start(XShitomasi_accel *InstancePtr);
u32 XShitomasi_accel_IsDone(XShitomasi_accel *InstancePtr);
u32 XShitomasi_accel_IsIdle(XShitomasi_accel *InstancePtr);
u32 XShitomasi_accel_IsReady(XShitomasi_accel *InstancePtr);
void XShitomasi_accel_EnableAutoRestart(XShitomasi_accel *InstancePtr);
void XShitomasi_accel_DisableAutoRestart(XShitomasi_accel *InstancePtr);

void XShitomasi_accel_Set_img_in(XShitomasi_accel *InstancePtr, u64 Data);
u64 XShitomasi_accel_Get_img_in(XShitomasi_accel *InstancePtr);
void XShitomasi_accel_Set_img_out(XShitomasi_accel *InstancePtr, u64 Data);
u64 XShitomasi_accel_Get_img_out(XShitomasi_accel *InstancePtr);
void XShitomasi_accel_Set_rows(XShitomasi_accel *InstancePtr, u32 Data);
u32 XShitomasi_accel_Get_rows(XShitomasi_accel *InstancePtr);
void XShitomasi_accel_Set_cols(XShitomasi_accel *InstancePtr, u32 Data);
u32 XShitomasi_accel_Get_cols(XShitomasi_accel *InstancePtr);
void XShitomasi_accel_Set_threshold(XShitomasi_accel *InstancePtr, u32 Data);
u32 XShitomasi_accel_Get_threshold(XShitomasi_accel *InstancePtr);
void XShitomasi_accel_Set_k(XShitomasi_accel *InstancePtr, u32 Data);
u32 XShitomasi_accel_Get_k(XShitomasi_accel *InstancePtr);

void XShitomasi_accel_InterruptGlobalEnable(XShitomasi_accel *InstancePtr);
void XShitomasi_accel_InterruptGlobalDisable(XShitomasi_accel *InstancePtr);
void XShitomasi_accel_InterruptEnable(XShitomasi_accel *InstancePtr, u32 Mask);
void XShitomasi_accel_InterruptDisable(XShitomasi_accel *InstancePtr, u32 Mask);
void XShitomasi_accel_InterruptClear(XShitomasi_accel *InstancePtr, u32 Mask);
u32 XShitomasi_accel_InterruptGetEnabled(XShitomasi_accel *InstancePtr);
u32 XShitomasi_accel_InterruptGetStatus(XShitomasi_accel *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
