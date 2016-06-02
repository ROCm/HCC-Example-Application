/*
Copyright (c) 2015-2016 Advanced Micro Devices, Inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include <hsa/hsa.h>

typedef std::chrono::high_resolution_clock myclock;

class BitonicSort 
{
public:
    BitonicSort(hc::accelerator acc);

    void setup();
    myclock::duration  run();
    void verifyResults();
    void cleanup();

private:
    void bitonicSortGPU(myclock::duration *d);
    void bitonicSortGPU_opt(myclock::duration *d);
    void bitonicSortCPUReference( uint32_t * input, const uint32_t length, const bool sortIncreasing);
private:
    uint32_t        _seed;
    uint32_t        _sortIncreasing;
    uint32_t        *_input;
    hc::array<uint32_t>  *_inputArray;
    uint32_t        *_inputAccPtr;

    uint32_t        *_verificationInput;
    int             _length;
    int             _numStages;

    int             _iterations;

    // Which accelerator to launch the kernel on:
    hc::accelerator _acc;


    // HSA info for launching kernel:
    uint64_t                     _codeHandle;
    hsa_signal_t                 _signal;
    char *                       _kernargPointer; // base of kernarg region.

    friend void init_kernel(BitonicSort *bs, hc::accelerator acc);
};
void init_kernel(BitonicSort *bs, hc::accelerator acc);


BitonicSort::BitonicSort(hc::accelerator acc)
    :_inputArray(0), _acc(acc)
{
    _seed = 123;
    _sortIncreasing = 0;
    _input = NULL;
    _verificationInput = NULL;
    _length = 32768;
    _numStages = 0;
    _iterations = 1; // TODO
    _signal.handle = -1;
    _kernargPointer = NULL;
}
