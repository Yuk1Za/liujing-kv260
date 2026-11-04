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
#include "xs2m_vmap.h"

extern XS2m_vmap_Config XS2m_vmap_ConfigTable[];

#ifdef SDT
XS2m_vmap_Config *XS2m_vmap_LookupConfig(UINTPTR BaseAddress) {
	XS2m_vmap_Config *ConfigPtr = NULL;

	int Index;

	for (Index = (u32)0x0; XS2m_vmap_ConfigTable[Index].Name != NULL; Index++) {
		if (!BaseAddress || XS2m_vmap_ConfigTable[Index].Control_BaseAddress == BaseAddress) {
			ConfigPtr = &XS2m_vmap_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XS2m_vmap_Initialize(XS2m_vmap *InstancePtr, UINTPTR BaseAddress) {
	XS2m_vmap_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XS2m_vmap_LookupConfig(BaseAddress);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XS2m_vmap_CfgInitialize(InstancePtr, ConfigPtr);
}
#else
XS2m_vmap_Config *XS2m_vmap_LookupConfig(u16 DeviceId) {
	XS2m_vmap_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XS2M_VMAP_NUM_INSTANCES; Index++) {
		if (XS2m_vmap_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XS2m_vmap_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XS2m_vmap_Initialize(XS2m_vmap *InstancePtr, u16 DeviceId) {
	XS2m_vmap_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XS2m_vmap_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XS2m_vmap_CfgInitialize(InstancePtr, ConfigPtr);
}
#endif

#endif

