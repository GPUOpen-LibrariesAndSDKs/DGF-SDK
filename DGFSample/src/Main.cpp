//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "stdafx.h"
#include "D3D12MeshletRender.h"

extern "C" {
__declspec(dllexport) extern const UINT D3D12SDKVersion = 614;
}  // or later
extern "C" {
__declspec(dllexport) extern const char* D3D12SDKPath = ".\\D3D12\\";
}

_Use_decl_annotations_ int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow)
{
    D3D12MeshletRender sample(1280, 720, L"D3D12 DGF Mesh Shader Decompression Sample");
    return Win32Application::Run(&sample, hInstance, nCmdShow);
}
