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

#pragma once

#include "api.h"
#include <string>
#include "Win32Application.h"

class Sample {
public:
    Sample(UINT width, UINT height, std::wstring name);
    virtual ~Sample();

    virtual void OnUpdate()  = 0;
    virtual void OnRender()  = 0;
    virtual void OnDestroy() = 0;

    // Samples override the event handlers to handle specific messages.
    virtual void OnKeyDown(UINT8 /*key*/) {}
    virtual void OnKeyUp(UINT8 /*key*/) {}

    // Accessors.
    UINT GetWidth() const
    {
        return m_width;
    }
    UINT GetHeight() const
    {
        return m_height;
    }
    const WCHAR* GetTitle() const
    {
        return m_title.c_str();
    }

    virtual void ParseCommandLineArgs(_In_reads_(argc) WCHAR* argv[], int argc) {};
    void Init(api::Device* device);

protected:
    virtual void OnInit(api::Device* device) = 0;

    void SetCustomWindowText(LPCWSTR text);

    // Viewport dimensions.
    UINT  m_width;
    UINT  m_height;
    float m_aspectRatio;
    ComPtr<api::Device> m_device;

private:
    
    // Window title.
    std::wstring m_title;
    std::wstring m_api;
};
