
set(DXC_URL "https://www.nuget.org/api/v2/package/Microsoft.Direct3D.DXC/1.8.2407.12")

# Check if DXC NuGet package was already downloaded
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/dxc.zip)
    message(STATUS "Downloading DirectX Shader Compiler from ${DXC_URL}")

    file(DOWNLOAD ${DXC_URL} ${CMAKE_CURRENT_BINARY_DIR}/dxc.zip STATUS DOWNLOAD_RESULT)

    list(GET DOWNLOAD_RESULT 0 DOWNLOAD_RESULT_CODE)
    if(NOT DOWNLOAD_RESULT_CODE EQUAL 0)
        message(FATAL_ERROR "Failed to download DirectX Shader Compiler! Error: ${DOWNLOAD_RESULT}.")
    endif()

    message(STATUS "Successfully downloaded DirectX Shader Compiler")
else()
    message(STATUS "Found local copy of ${DXC_URL} in ${CMAKE_CURRENT_BINARY_DIR}/dxc.zip. Skipping download.")
endif()

message(STATUS "Extracting DirectX Shader Compiler")

# extract dxc zip
file(ARCHIVE_EXTRACT
    INPUT ${CMAKE_CURRENT_BINARY_DIR}/dxc.zip
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/dxc)

# validate DXC binaries
if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/dxc/build/native/bin/x64/dxcompiler.dll)
    message(FATAL_ERROR "Failed to extract DirectX Shader Compiler!")
endif()

message(STATUS "Successfully extracted DirectX Shader Compiler")

configure_file("${CMAKE_CURRENT_BINARY_DIR}/dxc/build/native/bin/x64/dxcompiler.dll" "${CMAKE_BINARY_DIR}/bin/dxcompiler.dll" COPYONLY)
configure_file("${CMAKE_CURRENT_BINARY_DIR}/dxc/build/native/bin/x64/dxil.dll" "${CMAKE_BINARY_DIR}/bin/dxil.dll" COPYONLY)