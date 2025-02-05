project(agilitySDK)
include(update-agilitysdk.cmake)

add_library(agilitySDK INTERFACE )

target_include_directories(agilitySDK INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/agilitySDK/build/native/include)
target_compile_definitions(agilitySDK INTERFACE AGILITY_SDK_VERSION=614)
target_link_libraries(agilitySDK INTERFACE d3d12.lib)

