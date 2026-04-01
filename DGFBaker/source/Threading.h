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
#pragma once

#include <execution>
#include <algorithm>
#include <iterator>
#include <thread>

namespace Threading
{
    namespace _details
    {
        struct CountingIterator
        {
            CountingIterator() : _n(0) {};
            CountingIterator(size_t n) : _n(n) {};
            CountingIterator operator++() { return CountingIterator(++_n); }
            CountingIterator operator++(int) { return CountingIterator(_n++); }

            bool operator!=(const CountingIterator rhs)
            {
                return _n != rhs._n;
            }
            size_t operator*() { return _n; }

            size_t _n;
        };

    }
}

namespace std
{
    template<>
    class iterator_traits<Threading::_details::CountingIterator>
    {
    public:
        typedef std::forward_iterator_tag iterator_category;
        typedef size_t difference_type;
    };
}


namespace Threading
{
    namespace _details
    {

        struct Range
        {
            size_t begin;
            size_t end;
        };

        static inline Range AllocateWork(size_t numThreads, size_t threadIndex, size_t n)
        {
            size_t itemsPerThread = n / numThreads;
            size_t remainder = n % numThreads;

            size_t numPrior, numItems;
            if (threadIndex < remainder)
            {
                numPrior = (threadIndex * itemsPerThread) + threadIndex;
                numItems = itemsPerThread + 1;
            }
            else
            {
                numPrior = (threadIndex * itemsPerThread) + remainder;
                numItems = itemsPerThread;
            }

            return { numPrior, numPrior + numItems };
        }

     

        template< class TFunc >
        static void ForEach_StaticPartition( size_t n, TFunc fn )
        {
            size_t maxThreads = std::thread::hardware_concurrency();

            std::for_each_n(std::execution::par_unseq,
                CountingIterator(0), maxThreads,
                [maxThreads, n, fn](size_t threadIndex)
                {                    
                    _details::Range r = _details::AllocateWork(maxThreads, threadIndex, n);
                    fn(r.begin,r.end,threadIndex);
                }
            );
        }
        template< class TFunc >
        static void ForEach(size_t n, TFunc fn)
        {
            size_t maxThreads = std::thread::hardware_concurrency();

            std::for_each_n(std::execution::par_unseq,
                CountingIterator(0), n,
                [fn](size_t idx)
                {
                    fn(idx);
                }
            );
        }
    }


    template<class TFunc>
    void ParallelFor(size_t n, TFunc fn)
    {
        _details::ForEach(n, fn);
    }

    template< class TMapReduceFN, class TPairReduceFN >
    auto ParallelReduce(size_t n, TMapReduceFN mapReduce, TPairReduceFN pairReduce )
    {
        typedef decltype(mapReduce(0, n)) TReduceTY;
        size_t maxThreads = std::thread::hardware_concurrency();

        std::vector<TReduceTY> partials(maxThreads);
        
        _details::ForEach_StaticPartition(maxThreads,
            [n,mapReduce, &partials](size_t begin, size_t end, size_t threadIndex)
            {
                size_t maxThreads = std::thread::hardware_concurrency();

                _details::Range r = _details::AllocateWork(maxThreads, threadIndex, n);
                partials[threadIndex] = mapReduce(r.begin, r.end);
            }
        );
        
        if (maxThreads == 1)
        {
            return partials[0];
        }
        else
        {
            TReduceTY result = pairReduce(partials[0], partials[1]);
            for (size_t i = 2; i < maxThreads; i++)
                result = pairReduce(result, partials[i]);
            return result;
        }        
    }

}
