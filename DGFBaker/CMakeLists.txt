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

set( DGF_BAKER_DIR ${CMAKE_CURRENT_SOURCE_DIR} )

set( DGF_BAKER_API ${DGF_BAKER_DIR}/include/DGFBaker.h )

set( DGF_BAKER_SOURCES
     ${DGF_BAKER_DIR}/source/Utils.h
     ${DGF_BAKER_DIR}/source/ClusterBuilder.h
     ${DGF_BAKER_DIR}/source/ClusterBuilder.cpp
     ${DGF_BAKER_DIR}/source/Cluster.h
     ${DGF_BAKER_DIR}/source/Quantization.h 
     ${DGF_BAKER_DIR}/source/Quantization.cpp    
     ${DGF_BAKER_DIR}/source/HPG24BlockPacker.h 
     ${DGF_BAKER_DIR}/source/HPG24BlockPacker.cpp
     ${DGF_BAKER_DIR}/source/DebugOutput.h 
     ${DGF_BAKER_DIR}/source/DebugOutput.cpp
     ${DGF_BAKER_DIR}/source/AdjacencyGraph.h 
     ${DGF_BAKER_DIR}/source/StripGeneration.h 
     ${DGF_BAKER_DIR}/source/StripGeneration.cpp     
     ${DGF_BAKER_DIR}/source/MaterialCompression.h
     ${DGF_BAKER_DIR}/source/DGFBaker.cpp
     ${DGF_BAKER_DIR}/source/SAHBlockPacker.h 
     ${DGF_BAKER_DIR}/source/SAHBlockPacker.cpp
     ${DGF_BAKER_DIR}/source/SIMD.h     
)

add_library( DGFBaker STATIC ${DGF_BAKER_SOURCES} ${DGF_BAKER_API} )

target_include_directories( DGFBaker 
    PUBLIC ${DGF_LIB_ROOT} 
    PUBLIC ${DGF_BAKER_DIR}/include )



source_group( api ${DGF_BAKER_API} )
source_group( src ${DGF_BAKER_SOURCES} )
source_group( autogen "cxx|hxx" )

set_property(TARGET DGFBaker PROPERTY CXX_STANDARD 20)
