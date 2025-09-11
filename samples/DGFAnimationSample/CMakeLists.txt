project( DGFAnimationSample )

set( APP_NAME DGFAnimationSample )
set( SHADER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/shaders )

set( SHADERS   
    ${SHADER_ROOT}/KeyFrameInterpolationCompute.hlsl
    ${SHADER_ROOT}/Requantization.hlsl
    ${SHADER_ROOT}/RaytracingComputeShader.hlsl
)
set_source_files_properties(${SHADERS} PROPERTIES VS_TOOL_OVERRIDE "None")

set(SOURCE_FILES 

    include/DGFAnimationSample.h
    include/Animation.h
    
    src/DGFAnimationSample.cpp
    src/Main.cpp
    src/Animation.cpp
    src/RayTracing.cpp
)
     
add_executable( ${APP_NAME} WIN32 ${SHADERS} ${SOURCE_FILES} )


target_compile_definitions(${APP_NAME} PRIVATE UNICODE _UNICODE)
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
    SampleFramework
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
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/shaders/" "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/shaders/" "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/data/"
)

set( ALL_SHADERS ${SHADERS} ${SAMPLE_FRAMEWORK_SHADERS})

    
foreach(SHADER ${ALL_SHADERS})
    add_custom_command(
      TARGET ${APP_NAME}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SHADER} "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/shaders/"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SHADER} "${CMAKE_CURRENT_BINARY_DIR}/shaders/"
      COMMENT "Copying ${SHADER}"
    )
endforeach()

set_property(TARGET ${APP_NAME} PROPERTY CXX_STANDARD 20)
add_custom_command(
    TARGET ${APP_NAME}
    COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_CURRENT_SOURCE_DIR}/data/wood-doll.obj.tgz"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/data/"        
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/data/wood-doll.obj.tgz"
)