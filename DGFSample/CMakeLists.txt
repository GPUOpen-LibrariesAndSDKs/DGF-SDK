# This file is part of the the DGF-SDK.
#
# Copyright (C) 2025 Advanced Micro Devices, Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files(the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions :
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

project( DGFSample )

set( APP_NAME DGFSample )
set( SHADER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/shaders )

set( SHADERS 
    ${SHADER_ROOT}/DGFBitScanLaneMeshShader.hlsl
    ${SHADER_ROOT}/DGFBitScanWaveMeshShader.hlsl
    ${SHADER_ROOT}/DGFBVHBuildDecode.hlsl
    ${SHADER_ROOT}/DGFDecompression.hlsl
    ${SHADER_ROOT}/DGFSerialMeshShader.hlsl
    ${SHADER_ROOT}/PixelShader.hlsl
    ${SHADER_ROOT}/MeshShaderUtil.hlsl
    ${SHADER_ROOT}/RaytracingComputeShader.hlsl
)
set_source_files_properties(${SHADERS} PROPERTIES VS_TOOL_OVERRIDE "None")

set(SOURCE_FILES 

    include/D3D12MeshletRender.h
    include/d3dx12.h
    include/DXSample.h
    include/DXSampleHelper.h
    include/DXFramework.h
    include/SimpleCamera.h
    include/Span.h
    include/stdafx.h
    include/StepTimer.h
    include/Win32Application.h
    include/Util.h
    include/DGFBVHBuilder.h

    src/D3D12MeshletRender.cpp
    src/DXSample.cpp
    src/Main.cpp
    src/DXFramework.cpp
    src/SimpleCamera.cpp
    src/stdafx.cpp
    src/Win32Application.cpp
    src/Util.cpp
    src/DGFBVHBuilder.cpp
)
     
add_executable( ${APP_NAME} WIN32 ${SHADERS} ${SOURCE_FILES} )


target_compile_definitions(${APP_NAME} PRIVATE UNICODE _UNICODE)
target_compile_definitions(${APP_NAME} PUBLIC SHADER_ROOT=L"${SHADER_ROOT}/")
target_compile_definitions(${APP_NAME} PUBLIC DATA_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/data/")
target_include_directories(${APP_NAME} PUBLIC ${SHADER_ROOT} ${DXC_INCLUDE_PATH} include)

target_link_libraries( ${APP_NAME} PUBLIC 
    DGFLib 
    DGFBaker    
    tinyobj
    miniply 
    imgui
    agilitySDK 
    dxgi 
    d3d12 
    d3dcompiler 
    dxcompiler
) 


add_custom_command(TARGET ${APP_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different 
    ${CMAKE_BINARY_DIR}/bin/dxcompiler.dll
    ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/      

    COMMAND ${CMAKE_COMMAND} -E copy_if_different 
    ${CMAKE_BINARY_DIR}/bin/dxil.dll
    ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/
    
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/D3D12/ 
    
    COMMAND ${CMAKE_COMMAND} -E copy_if_different 
    ${CMAKE_BINARY_DIR}/bin/D3D12Core.dll
    ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/D3D12/ 
    
    COMMAND ${CMAKE_COMMAND} -E copy_if_different 
    ${CMAKE_BINARY_DIR}/bin/d3d12SDKLayers.dll
    ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/D3D12/
    
)

    
add_custom_command(TARGET ${APP_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/shaders/" "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/shaders/"
)

    
foreach(SHADER ${SHADERS})
    add_custom_command(
      TARGET ${APP_NAME}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SHADER} "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/shaders/"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SHADER} "${CMAKE_CURRENT_BINARY_DIR}/shaders/"
      COMMENT "Copying ${SHADER}"
    )
endforeach()

set_property(TARGET ${APP_NAME} PROPERTY CXX_STANDARD 20)
