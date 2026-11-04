// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef XFPGA_FEATURE_EXTRACT_HW_H
#define XFPGA_FEATURE_EXTRACT_HW_H

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
#include "xfpga_feature_extract_hw_hw.h"

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
    u64 Control_bus_BaseAddress;
} XFpga_feature_extract_hw_Config;
#endif

typedef struct {
    u64 Control_bus_BaseAddress;
    u32 IsReady;
} XFpga_feature_extract_hw;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XFpga_feature_extract_hw_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XFpga_feature_extract_hw_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XFpga_feature_extract_hw_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XFpga_feature_extract_hw_ReadReg(BaseAddress, RegOffset) \
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
int XFpga_feature_extract_hw_Initialize(XFpga_feature_extract_hw *InstancePtr, UINTPTR BaseAddress);
XFpga_feature_extract_hw_Config* XFpga_feature_extract_hw_LookupConfig(UINTPTR BaseAddress);
#else
int XFpga_feature_extract_hw_Initialize(XFpga_feature_extract_hw *InstancePtr, u16 DeviceId);
XFpga_feature_extract_hw_Config* XFpga_feature_extract_hw_LookupConfig(u16 DeviceId);
#endif
int XFpga_feature_extract_hw_CfgInitialize(XFpga_feature_extract_hw *InstancePtr, XFpga_feature_extract_hw_Config *ConfigPtr);
#else
int XFpga_feature_extract_hw_Initialize(XFpga_feature_extract_hw *InstancePtr, const char* InstanceName);
int XFpga_feature_extract_hw_Release(XFpga_feature_extract_hw *InstancePtr);
#endif

void XFpga_feature_extract_hw_Start(XFpga_feature_extract_hw *InstancePtr);
u32 XFpga_feature_extract_hw_IsDone(XFpga_feature_extract_hw *InstancePtr);
u32 XFpga_feature_extract_hw_IsIdle(XFpga_feature_extract_hw *InstancePtr);
u32 XFpga_feature_extract_hw_IsReady(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_EnableAutoRestart(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_DisableAutoRestart(XFpga_feature_extract_hw *InstancePtr);

void XFpga_feature_extract_hw_Set_laserCloudInArray(XFpga_feature_extract_hw *InstancePtr, u64 Data);
u64 XFpga_feature_extract_hw_Get_laserCloudInArray(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_cloudSize(XFpga_feature_extract_hw *InstancePtr, u32 Data);
u32 XFpga_feature_extract_hw_Get_cloudSize(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_start_point_ori(XFpga_feature_extract_hw *InstancePtr, u32 Data);
u32 XFpga_feature_extract_hw_Get_start_point_ori(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_end_point_ori(XFpga_feature_extract_hw *InstancePtr, u32 Data);
u32 XFpga_feature_extract_hw_Get_end_point_ori(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_rangeimage_point(XFpga_feature_extract_hw *InstancePtr, u64 Data);
u64 XFpga_feature_extract_hw_Get_rangeimage_point(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_rangeimage_range(XFpga_feature_extract_hw *InstancePtr, u64 Data);
u64 XFpga_feature_extract_hw_Get_rangeimage_range(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_rangeimage_lessflat(XFpga_feature_extract_hw *InstancePtr, u64 Data);
u64 XFpga_feature_extract_hw_Get_rangeimage_lessflat(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_rangeimage_ground(XFpga_feature_extract_hw *InstancePtr, u64 Data);
u64 XFpga_feature_extract_hw_Get_rangeimage_ground(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_rangeimage_flat(XFpga_feature_extract_hw *InstancePtr, u64 Data);
u64 XFpga_feature_extract_hw_Get_rangeimage_flat(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_rangeimage_lesssharp(XFpga_feature_extract_hw *InstancePtr, u64 Data);
u64 XFpga_feature_extract_hw_Get_rangeimage_lesssharp(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_Set_rangeimage_sharp(XFpga_feature_extract_hw *InstancePtr, u64 Data);
u64 XFpga_feature_extract_hw_Get_rangeimage_sharp(XFpga_feature_extract_hw *InstancePtr);

void XFpga_feature_extract_hw_InterruptGlobalEnable(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_InterruptGlobalDisable(XFpga_feature_extract_hw *InstancePtr);
void XFpga_feature_extract_hw_InterruptEnable(XFpga_feature_extract_hw *InstancePtr, u32 Mask);
void XFpga_feature_extract_hw_InterruptDisable(XFpga_feature_extract_hw *InstancePtr, u32 Mask);
void XFpga_feature_extract_hw_InterruptClear(XFpga_feature_extract_hw *InstancePtr, u32 Mask);
u32 XFpga_feature_extract_hw_InterruptGetEnabled(XFpga_feature_extract_hw *InstancePtr);
u32 XFpga_feature_extract_hw_InterruptGetStatus(XFpga_feature_extract_hw *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
