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
#include "xfpga_feature_extract_hw.h"

extern XFpga_feature_extract_hw_Config XFpga_feature_extract_hw_ConfigTable[];

#ifdef SDT
XFpga_feature_extract_hw_Config *XFpga_feature_extract_hw_LookupConfig(UINTPTR BaseAddress) {
	XFpga_feature_extract_hw_Config *ConfigPtr = NULL;

	int Index;

	for (Index = (u32)0x0; XFpga_feature_extract_hw_ConfigTable[Index].Name != NULL; Index++) {
		if (!BaseAddress || XFpga_feature_extract_hw_ConfigTable[Index].Control_bus_BaseAddress == BaseAddress) {
			ConfigPtr = &XFpga_feature_extract_hw_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XFpga_feature_extract_hw_Initialize(XFpga_feature_extract_hw *InstancePtr, UINTPTR BaseAddress) {
	XFpga_feature_extract_hw_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XFpga_feature_extract_hw_LookupConfig(BaseAddress);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XFpga_feature_extract_hw_CfgInitialize(InstancePtr, ConfigPtr);
}
#else
XFpga_feature_extract_hw_Config *XFpga_feature_extract_hw_LookupConfig(u16 DeviceId) {
	XFpga_feature_extract_hw_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XFPGA_FEATURE_EXTRACT_HW_NUM_INSTANCES; Index++) {
		if (XFpga_feature_extract_hw_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XFpga_feature_extract_hw_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XFpga_feature_extract_hw_Initialize(XFpga_feature_extract_hw *InstancePtr, u16 DeviceId) {
	XFpga_feature_extract_hw_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XFpga_feature_extract_hw_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XFpga_feature_extract_hw_CfgInitialize(InstancePtr, ConfigPtr);
}
#endif

#endif

