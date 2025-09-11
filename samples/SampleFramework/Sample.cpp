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


#include "Sample.h"


Sample::Sample(UINT width, UINT height, std::wstring name)
    : m_width(width), m_height(height), m_title(name)
{
    m_aspectRatio = static_cast<float>(width) / static_cast<float>(height);
}

Sample::~Sample() {}

void Sample::Init(api::Device* device)
{
    m_device = device;
    m_api = L"(" + m_device->GetAPIName() + L")";
    OnInit(device);
}

// Helper function for setting the window's title text.
void Sample::SetCustomWindowText(LPCWSTR text)
{
    std::wstring windowText = m_title + m_api + L": " + text;
    SetWindowText(Win32Application::GetHwnd(), windowText.c_str());
}

