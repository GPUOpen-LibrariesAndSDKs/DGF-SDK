
set( LIB_NAME SampleFramework )

set( SHADERS 
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DGFDecompression.hlsl
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders/DGFBVHBuildDecode.hlsl
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders/MeshShaderUtil.hlsl
)

set_source_files_properties(${SHADERS} PROPERTIES VS_TOOL_OVERRIDE "None")

add_library(${LIB_NAME} STATIC
            SimpleCamera.h
            SimpleCamera.cpp
            StepTimer.h
            Util.cpp
            Util.h 
            Sample.h
            DXSampleHelper.h
            Sample.cpp
            Win32Application.h
            Win32Application.cpp
            d3dx12.h

            api.h
            dx.h
            vk.h
            ShaderCompiler.h 
            ShaderCompiler.cpp

            vk.cpp
            dx.cpp

            DGFBVHBuilder.h 
            DGFBVHBuilder.cpp

            ${SHADERS}
)

set_property(TARGET ${LIB_NAME} PROPERTY CXX_STANDARD 20)

target_include_directories( ${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
target_compile_definitions( ${LIB_NAME} PRIVATE UNICODE _UNICODE)

target_link_libraries( ${LIB_NAME} PUBLIC 
    tinyobj
    miniply 
    imgui
    DGFLib
    DGFBaker
    agilitySDK 
    dxgi 
    d3d12 
    d3dcompiler 
    dxcompiler
    volk_headers
    vma
)

set( SAMPLE_FRAMEWORK_SHADERS ${SHADERS} PARENT_SCOPE )