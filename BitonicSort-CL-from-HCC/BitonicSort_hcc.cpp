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

#include <iostream>
#include <iomanip>
#include <chrono>

#include <hsa/hsa_ext_amd.h>
#include <hcc/hc.hpp>
#include <hcc/hc_am.hpp>

#include "hsa_utils.hpp"
#include "BitonicSort_hcc.hpp"


bool p_verify = true;       // Run a CPU version and verify results.
bool p_printArray = false;  // Print array.
bool p_printProgress = false; // print progress message at each kernel launch.

// This program uses the "hc" C++ runtime to manage memory.
// Use hc::array<> if true or use am_alloc pointer if false.
bool p_useHcArray = false;   


int  p_iterations = 10; // How many iterations to run.


// The kernel is stored in HSACO (HSA Code Object) form.
// The sample demonstrates two ways to load the hsaco file:
//   1. from an exernal file using load_hsa_code_object_from_file.  In this case, 
//   the makefile runs cloc to generate the hsaco file, and the app loads this 
//   file into memory using file I/O commands.
//
//   2. from a serialized internal string.  In this case the Makefile serializes 
//   the hsaco into a global string which is then linked into the executable, and
//   can be accessed via a global string.
//
//   Most applications use the second approach since it eliminates the need to locate the
//   external hsaco file at runtime.
bool p_loadKernelFromFile = false;  


/* 
 * Optimizations :
 */
bool p_optPreallocSignal  = false; // pre-allocate the signal in setup.
bool p_optPreallocKernarg = false; // pre-allocate the kernarg in setup.
bool p_optAvoidHostSync   = false; // Don't synchronize to host after each kernel launch
bool p_optFence           = false; // Don't fence/flush after each kernel submission.
bool p_optPinnedHost      = false; // Use pinned host memory for allocations.


// Filename for .hsaco file, only used if p_loadKernelFromFile=true.
#define HSACO_FILENAME "BitonicSort_Kernels.hsaco"

// Global symbols for embedded BitonicSort - only used if p_loadKernelFromFile=false
extern char _BitonicSort_Kernels_HSA_CodeObjMem[];
extern size_t _BitonicSort_Kernels_HSA_CodeObjMemSz;

// Name of kernel stored in the HSACO file:
#define HSACO_KERNELNAME "bitonicSort"


template<typename T>
void printArray(
    const std::string header,
    const T * data,
    const int width,
    const int height)
{
    std::cout<<"\n"<<header<<"\n";
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            std::cout<<data[i*width+j]<<" ";
        }
        std::cout<<"\n";
    }
    std::cout<<"\n";
}

const int CACHE_LINE_SIZE=64;

/*
 * This is the host-side representation of the kernel arguments expected by the 
 * BitonicSort kernel.
 * The format and alignment for this structure must exactly match the kernel 
 * signature defined in the kernel
 */   
struct BitonicSort_args_t {
    uint32_t * theArray;
    uint32_t stage;
    uint32_t passOfStage;
    uint32_t direction;
} ;


void 
BitonicSort::setup()
{

    // allocate and init memory used by host
    size_t sizeBytes = _length * sizeof(uint32_t);

    if (p_optPinnedHost) {
        _input = (uint32_t *) hc::am_alloc(sizeBytes, _acc,  amHostPinned);
    } else {
        _input = (uint32_t *) malloc(sizeBytes);
    }
    assert(_input);

    // random initialisation of input
    std::srand(_seed);
    std::generate_n(_input, _length, std::rand);


    if (p_useHcArray) {
        // Allocate the array to sort using hc::array<>,
        // which is a typed class representing device memory.

        _inputArray = new hc::array<uint32_t> (_length);
        _inputAccPtr = _inputArray->accelerator_pointer();


        if (p_printProgress) {
            printf ("info: allocated hc::array<>, size=%zu, accelerator_pointer=%p\n", sizeBytes, _inputAccPtr);
        }
    } else {
        // Allocate the array to sort using am_alloc, which returns a pointer.
        _inputAccPtr = hc::am_alloc(sizeBytes, _acc, 0);

        if (p_printProgress) {
            printf ("info: allocated hc::am_alloc, size=%zu, accelerator_pointer=%p\n", sizeBytes, _inputAccPtr);
        }
    }


    if(p_verify) {
        _verificationInput = (uint32_t *) malloc(sizeBytes);
        assert(_verificationInput);

        memcpy(_verificationInput, _input, sizeBytes);
    }


    if (p_printArray) {
        printArray<uint32_t>( "Unsorted Input", _input, _length, 1);
    }


    // Load kernel from file:
    hsa_agent_t *agent = static_cast<hsa_agent_t*> (_acc.get_hsa_agent());
    assert(agent);

    if (p_loadKernelFromFile) {
        _codeHandle = load_hsa_code_object_from_file(HSACO_FILENAME, HSACO_KERNELNAME, *agent);
    } else {
        _codeHandle = load_hsa_code_object(_BitonicSort_Kernels_HSA_CodeObjMem, 
                                           _BitonicSort_Kernels_HSA_CodeObjMemSz,
                                           HSACO_KERNELNAME, *agent);
    }

    /*
     * Determine how many stages and kernels we will need:
     */
    _numStages = 0;
    for(int temp = _length; temp > 1; temp >>= 1)
    {
        ++_numStages;
    }

    /* Signal creation involves a call into the kernel driver and can be an expensive 
     * operation.  This code creates the signal once at init time and then re-uses it for 
     * each kernel dispatch.
     */
    if (p_optPreallocSignal) {
        hsa_status_t hsa_status = hsa_signal_create(1, 0, NULL, &_signal);
        assert(HSA_STATUS_SUCCESS == hsa_status);
    }


    /*
     * pre-allocate the kernarg buffer to remove a moderately expensive operation from inside the kernel dispatch loop.
     * Also since we will be have many kernels in-flight at same time, each kernel needs its own kernarg buffer.
     */
    if (p_optPreallocKernarg) {
        /* Compute number of kernels that will be launched.  To count:
         *    - pair the first stage  (1 kernel)  and last stages (_numStages)                == _numStages + 1
         *    - pair the second stage (2 kernels) and the second-to-last stage (_numStages-1) == _numStages + 1
         *    - pair the third stage  (3 kernels) and the third-to-last stage (_numStages-2)  == _numStages + 1
         *    - and so on, creating _numStages/2 pairs each with _numStages+1 kernels:
         */
        int numKernels = _numStages * (_numStages + 1) / 2;

        /*
         * Allocate the kernel argument buffer from the correct region.
         * We use HCC's handy get_hsa_kernarg_region accessor:
         */   
        int alignedArgSize = (sizeof(BitonicSort_args_t) + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;

        if (p_printProgress) {
            printf ("numStages=%d, numKernels=%d, alignedArgSize=%d\n", _numStages, numKernels, alignedArgSize);
        }

        hsa_region_t kernarg_region = *(static_cast<hsa_region_t*> (_acc.get_hsa_kernarg_region()));
        hsa_status_t hsa_status = hsa_memory_allocate(kernarg_region, alignedArgSize * numKernels, (void**)(&_kernargPointer));
        assert(HSA_STATUS_SUCCESS == hsa_status);
    }
}


/*
 * Use the HSA Runtime API to run the BitonicSort kernel.
 *
 * Input: Must be called after setup.  
 * Notably _codeHandle must point to the BitonicSort kernel loaded from the .hsaco file.
 */
void BitonicSort::bitonicSortGPU(myclock::duration *d)
{
    size_t sizeBytes = _length * sizeof(uint32_t);

    hc::am_copy(_inputAccPtr, _input, sizeBytes);

    /*
     * Extract the hsaQueue from the HCC acclerator_view:
     */
    hc::accelerator_view av = _acc.get_default_view();
    hsa_queue_t  *hsaQueue = static_cast<hsa_queue_t*> (av.get_hsa_queue());

    hsa_status_t hsa_status = HSA_STATUS_SUCCESS;


    /* 
     * Get a signal
     */
    hsa_signal_t signal;
    if (p_optPreallocSignal) {
        signal = _signal;
        hsa_signal_store_relaxed(signal, 1); 
    } else {
        hsa_status = hsa_signal_create(1, 0, NULL, &signal);
        assert(HSA_STATUS_SUCCESS == hsa_status);
    }


    /*
     * Setup dispatch packet.
     */
    hsa_kernel_dispatch_packet_t aql;
    memset(&aql, 0, sizeof(aql));
    
    const int kNumDimension = 1;
    aql.completion_signal = signal;
    aql.setup = kNumDimension << HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS;
    aql.workgroup_size_x = (uint16_t) 256;
    aql.workgroup_size_y = 1;
    aql.workgroup_size_z = 1;
    aql.grid_size_x =      (uint32_t) (_length / 2);
    aql.grid_size_y = 1;
    aql.grid_size_z = 1;
    aql.header =
        (HSA_PACKET_TYPE_KERNEL_DISPATCH << HSA_PACKET_HEADER_TYPE) |
        (1 << HSA_PACKET_HEADER_BARRIER) |
        (HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_ACQUIRE_FENCE_SCOPE) |
        (HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_RELEASE_FENCE_SCOPE);
    aql.group_segment_size = 0;
    aql.private_segment_size = 0;
    aql.kernel_object = _codeHandle;  // set at initialization time.
   


    if (p_optPreallocKernarg) {
        aql.kernarg_address = _kernargPointer;
    } else {
        /*
         * Allocate the kernel argument buffer from the correct region.
         * We use HCC's handy get_hsa_kernarg_region accessor:
         */   
        hsa_region_t kernarg_region = *(static_cast<hsa_region_t*> (_acc.get_hsa_kernarg_region()));
        hsa_status = hsa_memory_allocate(kernarg_region, sizeof(BitonicSort_args_t), (void**)(&aql.kernarg_address));
        assert(HSA_STATUS_SUCCESS == hsa_status);

    } 


    /*
     * Write the args directly into the kernargs buffer:
     * Typecast the kernarg pointer to BitonicSort_args_t so arg-setting code can use the structure fields:
     */
    BitonicSort_args_t * args = (BitonicSort_args_t*) (aql.kernarg_address);
    args->theArray = _inputAccPtr;
    args->stage = 0;
    args->passOfStage = 0;
    args->direction = _sortIncreasing;


    const uint32_t queueSize = hsaQueue->size;
    const uint32_t queueMask = queueSize - 1;


    auto kernel_start_time = myclock::now();

    for(int stage = 0; stage < _numStages; ++stage) {
        args->stage = stage;
        for(int passOfStage = 0; passOfStage < stage + 1; ++passOfStage) {
            args->passOfStage  = passOfStage;

            if (p_printProgress) {
                printf ("  launching kernel for stage=%d pass=%d\n", stage, passOfStage); 
            }

            // Write AQL packet to queue to launch the kernel:
            {
                uint64_t writeIndex = hsa_queue_load_write_index_relaxed(hsaQueue);
                uint64_t readIndex  = hsa_queue_load_read_index_relaxed(hsaQueue);

                if ((writeIndex - readIndex) != queueMask) {
                    ((hsa_kernel_dispatch_packet_t*)(hsaQueue->base_address))[writeIndex & queueMask] = aql;

                    hsa_queue_store_write_index_relaxed(hsaQueue, writeIndex + 1);

                    // Ringdoor bell.
                    hsa_signal_store_relaxed(hsaQueue->doorbell_signal, writeIndex);
  
                    if (hsa_signal_wait_acquire(signal, HSA_SIGNAL_CONDITION_LT, 1, uint64_t(-1),
                                                HSA_WAIT_STATE_ACTIVE) != 0) {
                      printf("Signal wait returned unexpected value\n");
                      assert(0);
                    }

                    hsa_signal_store_relaxed(signal, 1);
                } else {
                    printf ("Error - queue full!\n");
                    assert(0);
                }
            }
        }
    };

    auto kernel_end_time = myclock::now();
    *d  = (kernel_end_time - kernel_start_time);

    if (!p_optPreallocSignal) {
        hsa_signal_destroy(signal);
    }
    if (!p_optPreallocKernarg) {
        hsa_memory_free(aql.kernarg_address);
    }

    hc::am_copy(_input, _inputAccPtr, sizeBytes);
}

/*
 * Use the HSA Runtime API to run the BitonicSort kernel.
 *
 * Input: Must be called after setup.  
 * Notably _codeHandle must point to the BitonicSort kernel loaded from the .hsaco file.
 *
 * This version is optimized to avoid host-side waiting between each kernel.
 * All kernels are launched up-front and the dependencies are resolved on the GPU.
 * Also, we optimize the AQL acquire/release fences since we know the data does not need to be visible
 * at the system scope - this can eliminate cache flushing between kernels.
 */
void BitonicSort::bitonicSortGPU_opt(myclock::duration *d)
{
    size_t sizeBytes = _length * sizeof(uint32_t);

    hc::am_copy(_inputAccPtr, _input, sizeBytes);

    /*
     * Extract the hsaQueue from the HCC acclerator_view:
     */
    hc::accelerator_view av = _acc.get_default_view();
    hsa_queue_t  *hsaQueue = static_cast<hsa_queue_t*> (av.get_hsa_queue());

    hsa_status_t hsa_status = HSA_STATUS_SUCCESS;


    /* 
     * Get a signal
     */
    hsa_signal_t signal;
    if (p_optPreallocSignal) {
        signal = _signal;
    } else {
        hsa_status = hsa_signal_create(1, 0, NULL, &signal);
        assert(HSA_STATUS_SUCCESS == hsa_status);
    }


    /*
     * Setup dispatch packet.
     */
    hsa_kernel_dispatch_packet_t aql;
    memset(&aql, 0, sizeof(aql));
    
    const int kNumDimension = 1;
    aql.completion_signal.handle = 0x0;
    aql.setup = kNumDimension << HSA_KERNEL_DISPATCH_PACKET_SETUP_DIMENSIONS;
    aql.workgroup_size_x = (uint16_t) 256;
    aql.workgroup_size_y = 1;
    aql.workgroup_size_z = 1;
    aql.grid_size_x =      (uint32_t) (_length / 2);
    aql.grid_size_y = 1;
    aql.grid_size_z = 1;
    if (!p_optFence) {
        aql.header =
            (HSA_PACKET_TYPE_KERNEL_DISPATCH << HSA_PACKET_HEADER_TYPE) |
            (1 << HSA_PACKET_HEADER_BARRIER) |
            (HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_ACQUIRE_FENCE_SCOPE) |
            (HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_RELEASE_FENCE_SCOPE);
    }
    aql.group_segment_size = 0;
    aql.private_segment_size = 0;
    aql.kernel_object = _codeHandle;  // set at initialization time.
   

    // Require this be pre-allocated:
    assert(p_optPreallocKernarg);

    hsa_signal_store_relaxed(signal, 1);


    const uint32_t queueSize = hsaQueue->size;
    const uint32_t queueMask = queueSize - 1;


    auto kernel_start_time = myclock::now();

    const int alignedArgSize = (sizeof(BitonicSort_args_t) + CACHE_LINE_SIZE - 1) / CACHE_LINE_SIZE * CACHE_LINE_SIZE;
    int kernelCount = 0;
    int numKernels = _numStages * (_numStages + 1) / 2;

    // Make sure we have room in the queue for all of the kernels:

    uint64_t writeIndex = hsa_queue_load_write_index_relaxed(hsaQueue);
    uint64_t readIndex  = hsa_queue_load_read_index_relaxed(hsaQueue);

    // Check once to make sure we have room for all the kernels:
    uint32_t availQueueEntries = (queueSize - (writeIndex - readIndex));
    if (availQueueEntries < numKernels) {
        printf ("Error - queue full!  queueSize=%d readIndex=%lu writeIndex=%lu\n", queueSize, readIndex, writeIndex);
        assert(0);
    }


    for(int stage = 0; stage < _numStages; ++stage) {
        for(int passOfStage = 0; passOfStage < stage + 1; ++passOfStage) {
            /*
             * Write the args directly into the kernargs buffer:
             * Typecast the kernarg pointer to BitonicSort_args_t so arg-setting code can use the structure fields:
             */
            aql.kernarg_address = static_cast<void*> (& (_kernargPointer[kernelCount * alignedArgSize ]));
            BitonicSort_args_t * args = (BitonicSort_args_t*) (aql.kernarg_address);
            args->theArray = _inputAccPtr;
            args->stage = stage;
            args->passOfStage  = passOfStage;
            args->direction = _sortIncreasing;

            kernelCount++;

            assert (kernelCount <= numKernels);

            if (p_optFence) {
                aql.header =
                    (HSA_PACKET_TYPE_KERNEL_DISPATCH << HSA_PACKET_HEADER_TYPE) |
                    (1 << HSA_PACKET_HEADER_BARRIER);
                bool setFence=false;
                if (kernelCount == 1) {
                    // first packet needs to acquire from system to make sure it gets the host->device copy:
                    aql.header |= (HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_ACQUIRE_FENCE_SCOPE);
                    aql.header |= (HSA_FENCE_SCOPE_AGENT  << HSA_PACKET_HEADER_RELEASE_FENCE_SCOPE);
                    setFence = true;
                } 
                if (kernelCount == numKernels) {
                    // last packet needs to release to system to make sure data is visible for device->host copy:
                    aql.header |= (HSA_FENCE_SCOPE_AGENT  << HSA_PACKET_HEADER_ACQUIRE_FENCE_SCOPE);
                    aql.header |= (HSA_FENCE_SCOPE_SYSTEM << HSA_PACKET_HEADER_RELEASE_FENCE_SCOPE);
                    setFence = true;
                }
                if (!setFence) {
                    // fences at agent scope:
                    aql.header |= (HSA_FENCE_SCOPE_AGENT  << HSA_PACKET_HEADER_ACQUIRE_FENCE_SCOPE);
                    aql.header |= (HSA_FENCE_SCOPE_AGENT  << HSA_PACKET_HEADER_RELEASE_FENCE_SCOPE);
                }
            }


            if (p_printProgress) {
                printf ("  launching kernel#%d  for stage=%d pass=%d\n", kernelCount, stage, passOfStage); 
            }

            if (kernelCount == numKernels) {
                aql.completion_signal = signal;
                if (p_printProgress) {
                    printf ("    <last kernel, set completion_signal");
                }
            }

            // Write AQL packet to queue to launch the kernel:
            {
                uint64_t writeIndex = hsa_queue_load_write_index_relaxed(hsaQueue);
                ((hsa_kernel_dispatch_packet_t*)(hsaQueue->base_address))[writeIndex & queueMask] = aql;

                hsa_queue_store_write_index_relaxed(hsaQueue, writeIndex + 1);

                // Ring door bell.
                hsa_signal_store_relaxed(hsaQueue->doorbell_signal, writeIndex);
            }
        }
    };

    // Host wait for last kernel to finish:
    if (hsa_signal_wait_acquire(signal, HSA_SIGNAL_CONDITION_LT, 1, uint64_t(-1),
                              HSA_WAIT_STATE_ACTIVE) != 0) {
        printf("Signal wait returned unexpected value\n");
        assert(0);
    }

    auto kernel_end_time = myclock::now();
    *d  = (kernel_end_time - kernel_start_time);

    if (!p_optPreallocSignal) {
        hsa_signal_destroy(signal);
    }
    if (!p_optPreallocKernarg) {
        hsa_memory_free(aql.kernarg_address);
    }

    hc::am_copy(_input, _inputAccPtr, sizeBytes);
}


myclock::duration BitonicSort::run()
{
    myclock::duration total_kernel_time(0), oneiter_kernel_time;
    for (int i=0; i<p_iterations; i++) {
        if (p_optAvoidHostSync) {
            bitonicSortGPU_opt(&oneiter_kernel_time);
        } else {
            bitonicSortGPU(&oneiter_kernel_time);
        }
        total_kernel_time += oneiter_kernel_time;
    }

    return total_kernel_time;
}

void 
BitonicSort::cleanup()
{
    if (_input) {
        if (p_optPinnedHost) {
            hc::am_free(_input);
        } else {
            free(_input);
        }
        _input = NULL;
    }

    if (_verificationInput) {
        free(_verificationInput);
        _verificationInput = NULL;
    }

    if (p_optPreallocSignal && (_signal.handle != -1)) {
        hsa_signal_destroy(_signal);
        _signal.handle = -1;
    }

    if (p_optPreallocKernarg && _kernargPointer) {
        hsa_memory_free(_kernargPointer);
        _kernargPointer = NULL;
    };

}


void printTime(const char * timeLabel, myclock::duration d,
               bool printPerIter=false)
{
    using namespace std;

    const int fw = 22;
    cout << left << setw(fw) << timeLabel
         << chrono::duration_cast<chrono::microseconds> (d).count()
         << " us";

    if (printPerIter) {
        cout << " (" 
             << chrono::duration_cast<chrono::microseconds> (d/p_iterations).count()
             << " us/iteration)";
    }

    cout << endl;
}

// Helper function for CPU implementation:
void
swapIfFirstIsGreater(uint32_t *a, uint32_t *b)
{
    if(*a > *b)
    {
        uint32_t temp = *a;
        *a = *b;
        *b = temp;
    }
}


/*
 * sorts the input array (in place) using the bitonic sort algorithm
 * sorts in increasing order if sortIncreasing is true
 * else sorts in decreasing order
 * length specifies the length of the array
 */
void
BitonicSort::bitonicSortCPUReference(
    uint32_t * input,
    const uint32_t length,
    const bool sortIncreasing)
{
    const uint32_t halfLength = length/2;

    uint32_t i;
    for(i = 2; i <= length; i *= 2)
    {
        uint32_t j;
        for(j = i; j > 1; j /= 2)
        {
            bool increasing = sortIncreasing;
            const uint32_t half_j = j/2;

            uint32_t k;
            for(k = 0; k < length; k += j)
            {
                const uint32_t k_plus_half_j = k + half_j;
                uint32_t l;

                if(i < length)
                {
                    if((k == i) || (((k % i) == 0) && (k != halfLength)))
                    {
                        increasing = !increasing;
                    }
                }

                for(l = k; l < k_plus_half_j; ++l)
                {
                    if(increasing)
                    {
                        swapIfFirstIsGreater(&input[l], &input[l + half_j]);
                    }
                    else
                    {
                        swapIfFirstIsGreater(&input[l + half_j], &input[l]);
                    }
                }
            }
        }
    }
}


void BitonicSort::verifyResults()
{
    // Run on CPU:
    auto cpu_start_time = myclock::now();
    for (int i=0; i<p_iterations; i++) {
        bitonicSortCPUReference(_verificationInput, _length, _sortIncreasing);
    }
    auto cpu_end_time = myclock::now();
    printTime("CPU run time", cpu_end_time - cpu_start_time, true);


    // Compare the results and see if they match
    if(memcmp(_input, _verificationInput, _length*sizeof(uint32_t)) == 0)
    {
        std::cout<<"Passed!\n" << std::endl;
    }
    else
    {
        std::cout<<"Failed\n" << std::endl;
    }
}


void printHelp() 
{
    printf ("usage: BitonicSort [options]\n");
    printf ("--opt                Enable all optimizations.\n");
    printf ("--optPreallocSignal  Pre-allocate the signal in setup.\n");
    printf ("--optPreallocKernarg Pre-allocate the kernarg in setup.\n");
    printf ("--optAvoidHostSync   Don't synchronize to host after each kernel launch\n");
    printf ("--optFence           Don't fence/flush after each kernel submission.\n");
    printf ("--optPinnedHost      Use pinned host memory for allocations.\n");
    printf ("\n");
    printf ("--printProgress      Print progress messages for each stage.  Will impact timing measurements.\n");
    printf ("--useHcArray         Use hc::array<> to allocate memory (default uses hc::am_alloc).\n");
    printf ("--loadKernelFromFile Load HSACO from file (rather than use embedded HSACO string)\n");
    exit(0);
}


int parseInt(const char *str, int *output)
{
    char *next;
    *output = strtol(str, &next, 0);
    return !strlen(next);
}


void parseArguments(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];

        if (!strcmp(arg, "--help")) {
            printHelp();
        } else if (!strcmp(arg, "--opt")) {
            p_optPreallocSignal  = true;
            p_optPreallocKernarg = true;
            p_optAvoidHostSync   = true;
            p_optFence           = true;
            p_optPinnedHost      = true;
        } else if (!strcmp(arg, "--optPreallocSignal")) {
            p_optPreallocSignal  = true;
        } else if (!strcmp(arg, "--optPreallocKernarg")) {
            p_optPreallocKernarg = true;
        } else if (!strcmp(arg, "--optAvoidHostSync")) {
            p_optAvoidHostSync   = true;
        } else if (!strcmp(arg, "--optFence")) {
            p_optFence           = true;
        } else if (!strcmp(arg, "--optPinnedHost")) {
            p_optPinnedHost      = true;
        } else if (!strcmp(arg, "--printProgress")) {
            p_printProgress      = true;
        } else if (!strcmp(arg, "--useHcArray")) {
            p_useHcArray         = true;
        } else if (!strcmp(arg, "--loadKernelFromFile")) {
            p_loadKernelFromFile = true;
        } else if (!strcmp(arg, "--iterations")) {
            if (++i >= argc || !parseInt(argv[i], &p_iterations)) {
               printf("Bad iterations argument\n"); 
               assert(0);
            }
        } else {
            printf ("error: bad argument '%s'\n", arg);
            assert(0);
        }
    };
}


int
main(int argc, char * argv[])
{
    parseArguments(argc, argv);
    printf ("optimizations: p_optPreallocSignal=%d p_optPreallocKernarg=%d p_optAvoidHostSync=%d p_optFence=%d p_optPinnedHost=%d\n",
                            p_optPreallocSignal,   p_optPreallocKernarg,   p_optAvoidHostSync,   p_optFence,   p_optPinnedHost);
    printf ("\n");

    // Create a class to stack state (kernels, queues, args, etc) between the different functions:
    BitonicSort bs((hc::accelerator()));

    auto start_time = myclock::now();

    bs.setup();

    auto setup_end_time = myclock::now();
    myclock::duration total_kernel_time = bs.run();

    auto run_end_time = myclock::now();

    printf   ("iterations=%d\n", p_iterations);
    printTime("GPU setup time",        setup_end_time - start_time);
    printTime("GPU run time",          run_end_time - setup_end_time, true);
    printTime("GPU run(kernel) time",  total_kernel_time, true);
    printTime("GPU setup+run time",    run_end_time - start_time);

    if (p_verify) {
        bs.verifyResults();
    }


    bs.cleanup();
}
