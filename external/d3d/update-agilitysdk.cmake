
set(AGILITY_SDK_URL "https://www.nuget.org/api/v2/package/Microsoft.Direct3D.D3D12/1.614.1")

# Check if Agility SDK NuGet package was already downloaded
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/agilitySDK.zip)
    message(STATUS "Downloading Agility SDK from ${AGILITY_SDK_URL}")

    file(DOWNLOAD ${AGILITY_SDK_URL} ${CMAKE_CURRENT_BINARY_DIR}/agilitySDK.zip STATUS DOWNLOAD_RESULT)

    list(GET DOWNLOAD_RESULT 0 DOWNLOAD_RESULT_CODE)
    if(NOT DOWNLOAD_RESULT_CODE EQUAL 0)
        message(FATAL_ERROR "Failed to download Agility SDK! Error: ${DOWNLOAD_RESULT}.")
    endif()

    message(STATUS "Successfully downloaded Agility SDK")
else()
    message(STATUS "Found local copy of ${AGILITY_SDK_URL} in ${CMAKE_CURRENT_BINARY_DIR}/agilitySDK.zip. Skipping download.")
endif()

message(STATUS "Extracting Agility SDK")

# extract agility SDK zip
file(ARCHIVE_EXTRACT
    INPUT ${CMAKE_CURRENT_BINARY_DIR}/agilitySDK.zip
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/agilitySDK)

# validate agility SDK binaries
if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/agilitySDK/build/native/bin/x64/D3D12Core.dll OR
   NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/agilitySDK/build/native/bin/x64/d3d12SDKLayers.dll)
    message(FATAL_ERROR "Failed to extract Agility SDK!")
endif()

message(STATUS "Successfully extracted Agility SDK")

configure_file("${CMAKE_CURRENT_BINARY_DIR}/agilitySDK/build/native/bin/x64/D3D12Core.dll" "${CMAKE_BINARY_DIR}/bin/D3D12Core.dll" COPYONLY)
configure_file("${CMAKE_CURRENT_BINARY_DIR}/agilitySDK/build/native/bin/x64/d3d12SDKLayers.dll" "${CMAKE_BINARY_DIR}/bin/d3d12SDKLayers.dll" COPYONLY)