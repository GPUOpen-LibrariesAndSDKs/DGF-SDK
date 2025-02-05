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

cmake_minimum_required(VERSION 3.5)
project(DGFSDK)

# Check if DGF is used as main project or just consumed as libary with add_subdirectory
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(DGF_IS_TOP_LEVEL ON)
else()
    set(DGF_IS_TOP_LEVEL OFF)
endif()


if( DGF_IS_TOP_LEVEL )
    message( STATUS "DGFSDK:  Top-level build detected" )
    
    # We're the root project, build everything
    set( DGF_BUILD_DGFLIB 1 )
    set( DGF_BUILD_DGFBAKER 1 )
    set( DGF_BUILD_DGFTESTER 1 )
	set( DGF_BUILD_SAMPLES 1 )
else()
        
    # we're a client of somebody else's project, let them control what they want
    # by default, build only 'DGFLib'
    message( STATUS "DGFSDK:  Library build detected" )

	if( NOT DEFINED DGF_BUILD_DGFTESTER  )
		set( DGF_BUILD_DGFTESTER 0 )
        message( STATUS "DGF_BUILD_DGFLIB not set, setting to ${DGF_BUILD_DGFTESTER}")	
    endif()
	
	if( NOT DEFINED DGF_BUILD_SAMPLES )
		set( DGF_BUILD_SAMPLES 0 )
		message( STATUS "DGF_BUILD_SAMPLES not seat, setting to ${DGF_BUILD_SAMPLES}")
	endif()
	
    if( NOT DEFINED DGF_BUILD_DGFLIB )
	    set( DGF_BUILD_DGFLIB 1 )
        message( STATUS "DGF_BUILD_DGFLIB not set, setting to ${DGF_BUILD_DGFLIB}")
    endif()

    if( NOT DEFINED DGF_BUILD_DGFBAKER )        
        set( DGF_BUILD_DGFBAKER 0 )
	    message( STATUS "DGF_BUILD_DGFBAKER not set, setting to ${DGF_BUILD_DGFBAKER}")
    endif()
endif()

# DGFTester and Samples depend on DGFBaker
if( DGF_BUILD_SAMPLES OR DGF_BUILD_DGFTESTER )
	set( DGF_BUILD_DGFBAKER 1 )
	set( DGF_BUILD_DGFLIB 1 )
endif()

# DGFBaker depends on DGFLib
if( DGF_BUILD_DGFBAKER )
    set( DGF_BUILD_DGFLIB 1 )
endif()

add_subdirectory(external)

if( DGF_BUILD_DGFLIB )
    set( DGF_LIB_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/DGFLib )
    add_subdirectory(DGFLib)
endif()
if( DGF_BUILD_DGFBAKER )
    add_subdirectory(DGFBaker)
endif()
if( DGF_BUILD_DGFTESTER )
    add_subdirectory(DGFTester)
endif()
if( DGF_BUILD_SAMPLES )
	add_subdirectory(DGFSample)
endif()
