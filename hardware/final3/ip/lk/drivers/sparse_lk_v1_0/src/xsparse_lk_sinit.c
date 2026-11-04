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
#include "xsparse_lk.h"

extern XSparse_lk_Config XSparse_lk_ConfigTable[];

#ifdef SDT
XSparse_lk_Config *XSparse_lk_LookupConfig(UINTPTR BaseAddress) {
	XSparse_lk_Config *ConfigPtr = NULL;

	int Index;

	for (Index = (u32)0x0; XSparse_lk_ConfigTable[Index].Name != NULL; Index++) {
		if (!BaseAddress || XSparse_lk_ConfigTable[Index].Control_BaseAddress == BaseAddress) {
			ConfigPtr = &XSparse_lk_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XSparse_lk_Initialize(XSparse_lk *InstancePtr, UINTPTR BaseAddress) {
	XSparse_lk_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XSparse_lk_LookupConfig(BaseAddress);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XSparse_lk_CfgInitialize(InstancePtr, ConfigPtr);
}
#else
XSparse_lk_Config *XSparse_lk_LookupConfig(u16 DeviceId) {
	XSparse_lk_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XSPARSE_LK_NUM_INSTANCES; Index++) {
		if (XSparse_lk_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XSparse_lk_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XSparse_lk_Initialize(XSparse_lk *InstancePtr, u16 DeviceId) {
	XSparse_lk_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XSparse_lk_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XSparse_lk_CfgInitialize(InstancePtr, ConfigPtr);
}
#endif

#endif

