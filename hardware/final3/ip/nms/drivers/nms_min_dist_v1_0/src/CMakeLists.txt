# ==============================================================
# Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
# Tool Version Limit: 2024.11
# Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
# Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
# 
# ==============================================================
cmake_minimum_required(VERSION 3.15)
project(nms_min_dist)
find_package(common)
collector_create (PROJECT_LIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}")
collect (PROJECT_LIB_SOURCES xnms_min_dist.c)
collect (PROJECT_LIB_SOURCES xnms_min_dist_sinit.c)
collect (PROJECT_LIB_SOURCES xnms_min_dist_linux.c)
collect (PROJECT_LIB_SOURCES xnms_min_dist_g.c)
collector_list (_sources PROJECT_LIB_SOURCES)
collector_create (PROJECT_LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}")
collect (PROJECT_LIB_HEADERS xnms_min_dist.h)
collect (PROJECT_LIB_HEADERS xnms_min_dist_hw.h)
collector_list (_headers PROJECT_LIB_HEADERS)
file(COPY ${_headers} DESTINATION ${CMAKE_BINARY_DIR}/include)
include_directories(${CMAKE_BINARY_DIR}/include)
add_library(nms_min_dist STATIC ${_sources})
set_target_properties(nms_min_dist PROPERTIES LINKER_LANGUAGE C)

