// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
// 
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#ifdef SDT
#include "xparameters.h"
#endif
#include "xnms_min_dist.h"

extern XNms_min_dist_Config XNms_min_dist_ConfigTable[];

#ifdef SDT
XNms_min_dist_Config *XNms_min_dist_LookupConfig(UINTPTR BaseAddress) {
	XNms_min_dist_Config *ConfigPtr = NULL;

	int Index;

	for (Index = (u32)0x0; XNms_min_dist_ConfigTable[Index].Name != NULL; Index++) {
		if (!BaseAddress || XNms_min_dist_ConfigTable[Index].Control_BaseAddress == BaseAddress) {
			ConfigPtr = &XNms_min_dist_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XNms_min_dist_Initialize(XNms_min_dist *InstancePtr, UINTPTR BaseAddress) {
	XNms_min_dist_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XNms_min_dist_LookupConfig(BaseAddress);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XNms_min_dist_CfgInitialize(InstancePtr, ConfigPtr);
}
#else
XNms_min_dist_Config *XNms_min_dist_LookupConfig(u16 DeviceId) {
	XNms_min_dist_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XNMS_MIN_DIST_NUM_INSTANCES; Index++) {
		if (XNms_min_dist_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XNms_min_dist_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XNms_min_dist_Initialize(XNms_min_dist *InstancePtr, u16 DeviceId) {
	XNms_min_dist_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XNms_min_dist_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XNms_min_dist_CfgInitialize(InstancePtr, ConfigPtr);
}
#endif

#endif

