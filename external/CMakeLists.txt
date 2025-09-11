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

add_subdirectory(miniply)
add_subdirectory(tinyobjloader)

if ( DGF_BUILD_SAMPLES )
    add_subdirectory(d3d)
    add_subdirectory(imgui) # exclude imgui unless building samples
    
	set( VULKAN_HEADERS_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Vulkan-Headers" )
    add_subdirectory(volk)
    add_subdirectory(vma)

endif()


set_target_properties(miniply PROPERTIES FOLDER "external")
set_target_properties(tinyobj PROPERTIES FOLDER "external")

if ( DGF_BUILD_SAMPLES )
    set_target_properties(imgui PROPERTIES FOLDER "external")    # exclude imgui unless building samples
    set_target_properties(imgui PROPERTIES EXCLUDE_FROM_ALL 1)

    set_target_properties(volk PROPERTIES FOLDER "external"	)
    set_target_properties(vma PROPERTIES FOLDER "external")
	set_target_properties(agilitySDK PROPERTIES FOLDER "external")

    set_target_properties(volk PROPERTIES EXCLUDE_FROM_ALL 1)
    set_target_properties(vma PROPERTIES EXCLUDE_FROM_ALL 1)
    set_target_properties(agilitySDK PROPERTIES EXCLUDE_FROM_ALL 1)

endif()

set_target_properties(miniply PROPERTIES EXCLUDE_FROM_ALL 1)
set_target_properties(tinyobj PROPERTIES EXCLUDE_FROM_ALL 1)
