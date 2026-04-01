// This file is part of the DGF-SDK.
//
// Copyright (C) 2025 Advanced Micro Devices, Inc.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// clang-format off

#include "DebugOutput.h"
#include "HPG24BlockPacker.h"
#include "DGFBaker.h"

namespace DGFBaker
{    
    void PrintPerfData( std::ostream& str, PerformanceData& perfData, const DGFBaker::Config& config )
    {
        std::chrono::duration<double> totalTime = std::chrono::duration<double>(perfData.TotalTime);

        str << "Packed in " << totalTime << std::endl;

        auto PrintTiming = [&](const std::string& label, const auto time) {
            auto timed = std::chrono::duration<double>(time);
            str << "\t";
            str << std::left << std::setw(32) << (label + ":");
            std::stringstream ss;
            ss << std::setprecision(3) << (100*timed.count() / totalTime.count()) << "%";
            str << std::left << std::setw(10) << ss.str();
            str << std::left << std::setw(10) << timed << std::endl;
        };

        PrintTiming("Clustering",                perfData.Clustering);
        PrintTiming("ExponentSelect", perfData.ExponentSelect);
        if (config.packer == DGFBaker::PackerMode::HPG24)
        {
            PrintTiming("PackBlocksHPG24", perfData.TrianglePacking);
        }
        else
        {
            PrintTiming("PackBlocksSAH",       perfData.TrianglePacking);
        }
 
        PrintTiming("Cleanup",       perfData.Cleanup);

        auto categorizedTime = perfData.Clustering +
            perfData.TrianglePacking +
            perfData.ExponentSelect +
            perfData.Cleanup;

        PrintTiming("Other",                     perfData.TotalTime - categorizedTime);

    }

}