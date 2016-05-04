/**********************************************************************
Copyright ©2013 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

•   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
•   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

/**
*******************************************************************************
* @file <SyncVsAsyncArrayCopy.cpp>
*
* @brief Implements GPU based RGB to YUV 4:4:4 conversion depicting differences
*        in using HC Array copy() & HC Array copy_aysnc() to do the same
******************************************************************************/

/******************************************************************************
* Included header files                                                       *
******************************************************************************/
#include "SyncVsAsyncArrayCopy.hpp"
#include <iostream>


/******************************************************************************
* Implementation of RgbToYuv::initialize()                                    *
******************************************************************************/
int RgbToYuv::initialize()
{
    if(sampleArgs-> initialize())
    {
        return SDK_FAILURE;
    }

    Option* numInpValOpt = new Option;
    CHECK_ALLOCATION(numInpValOpt, "Memory Allocation error. (numInpValOpt)");

    numInpValOpt->_sVersion = "x";
    numInpValOpt->_lVersion = "samples";
    numInpValOpt->_description = "Number of example input values (multiples of 3)";
    numInpValOpt->_type = CA_ARG_INT;
    numInpValOpt->_value = &inpDataLen;
    sampleArgs->AddOption(numInpValOpt);
    delete numInpValOpt;

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of RgbToYuv::setup()                                         *
******************************************************************************/
int RgbToYuv::setup(std::vector<DATA_TYPE>& inputRgbData)
{
    /**************************************************************************
    * Ensure that input data size is a multiple of 3. Resize the input vector *
    * to reflect the update to the number of input data elements              *
    **************************************************************************/
    unsigned remainder = (inpDataLen % 3);
    inpDataLen = (remainder) ? inpDataLen+(3-remainder) : inpDataLen;
    inputRgbData.resize(inpDataLen);

    /**************************************************************************
    * Initialize random number generator and generate random values for input.*
    * In a practical usecase the input/output data would be read/written      *
    * from/to image files                                                     *
    **************************************************************************/
    srand(2012);

    DATA_TYPE *pData = inputRgbData.data();
    for (int i = 0; i < inpDataLen; ++i)
    {
        pData[i] = (unsigned char)((int)rand() * 10
                                   / (int)((RAND_MAX + 1)/(rand()+1)));
    }

    /**************************************************************************
    * Create an object of the first AMD accelerator (if & when available). If *
    * not available then ensure that the default accl is not a slow emulator  *
    **************************************************************************/
    sampleArgs->printDeviceList();
    if(sampleArgs->setDefaultAccelerator() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    std::cout << std::endl << std::endl;

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed setup() of SyncVsAsyncArrayCopy example\n";
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of RgbToYuv::run()                                           *
******************************************************************************/
int RgbToYuv::run(std::vector<DATA_TYPE>& inputRgbData,
                  std::vector<DATA_TYPE>& outputFromSync,
                  std::vector<DATA_TYPE>& outputFromAsync)
{
    /**************************************************************************
    * Create a HC C++ extent 1/3 the size of the input data.  This will make *
    * the loop execute 3 times over the input data                            *
    **************************************************************************/
    unsigned numThreads = inpDataLen/3;

    /**************************************************************************
    * Create & init timer objects to track the execution time of various runs *
    **************************************************************************/
    int syncTimer = sampleTimer->createTimer();
    int asyncTimer = sampleTimer->createTimer();

    /**************************************************************************
    * Warmup code with the sole purpose of avoiding JIT (just-in-time)        *
    * compilation overhead in affecting performance measurements. Create C++  *
    * HC arrays to be used for warmup only                                   *
    **************************************************************************/
    extent<1> inpExt(numThreads);
    array<DATA_TYPE, 1> rgbSyncArray(inpExt);
    array<DATA_TYPE, 1> yuvArray(inpExt);
    rgbToYuvHC(rgbSyncArray, yuvArray);
    if(!sampleArgs->quiet)
    {
        std::cout << "Completed Warm up run of HC C++ code" << std::endl;
    }

    sampleTimer->resetTimer(syncTimer);
    sampleTimer->resetTimer(asyncTimer);

    /**************************************************************************
    * Execute RGB -> YUV 4:4:4 using Synchronous Array copy                   *
    **************************************************************************/
    if(!sampleArgs->quiet)
    {
        std::cout << "Executing HC SynchronousCopy version of RGB to YUV(4:4:4)...\n";
    }

    sampleTimer->startTimer(syncTimer);
    syncArrayCopy(inputRgbData, outputFromSync, numThreads);
    sampleTimer->stopTimer(syncTimer);
    syncExecTime = (double)(sampleTimer->readTimer(syncTimer));

    /**************************************************************************
    * Execute RGB -> YUV 4:4:4 using Asynchronous Array copy                  *
    **************************************************************************/
    if(!sampleArgs->quiet)
    {
        std::cout <<
                  "Executing HC AsynchronousCopy version of RGB to YUV(4:4:4)...\n";
    }

    sampleTimer->startTimer(asyncTimer);
    asyncArrayCopy(inputRgbData, outputFromAsync, numThreads);
    sampleTimer->stopTimer(asyncTimer);
    asyncExecTime = (double)(sampleTimer->readTimer(asyncTimer));

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed run() of SyncVsAsyncArrayCopy example\n";
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of RgbToYuv::verifyResults()                                 *
******************************************************************************/
int RgbToYuv::verifyResults(std::vector<DATA_TYPE>& inputRgbData,
                            std::vector<DATA_TYPE>& outputFromSync,
                            std::vector<DATA_TYPE>& outputFromAsync,
                            std::vector<DATA_TYPE>& refVector)
{
    if(sampleArgs->verify)
    {
        std::cout << std::endl << "Verifying..." << std::endl;
        int cpuTimer = sampleTimer->createTimer();
        sampleTimer->resetTimer(cpuTimer);

        /**********************************************************************
        * Execute RGB -> YUV 4:4:4 on CPU and generate the reference output   *
        **********************************************************************/
        if(!sampleArgs->quiet)
        {
            std::cout << "Executing CPU(single core) version of RGB to YUV(4:4:4)...\n";
        }

        sampleTimer->startTimer(cpuTimer);
        rgbToYuvSingleCpu(inputRgbData.data(), refVector);
        sampleTimer->stopTimer(cpuTimer);
        cpuExecTime = (double)(sampleTimer->readTimer(cpuTimer));

        std::cout << "Output of HC Synchronous Copy ";
        if (compare(outputFromSync, refVector) != SDK_SUCCESS)
        {
            std::cout << "Data MISMATCH\n";
        }
        else
        {
            std::cout << "matches the reference output of CPU\n";
        }

        std::cout << "Output of HC Asynchronous Copy ";
        if (compare(outputFromAsync, refVector) != SDK_SUCCESS)
        {
            std::cout << "Data MISMATCH\n\n";
            return SDK_FAILURE;
        }
        std::cout << "matches the reference output of CPU\n\n";
    }

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed verifyResults() of SyncVsAsyncArrayCopy example\n";
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of RgbToYuv::printStats()                                    *
******************************************************************************/
void RgbToYuv::printStats()
{
    if (sampleArgs->timing)
    {
        std::stringstream cpuTimeStr;
        if (cpuExecTime != 0)
        {
            cpuTimeStr << std::setprecision(3) << cpuExecTime << "s\n";
        }
        else
        {
            cpuTimeStr << "Not run on CPU\n";
        }

        std::cout << "\nRgbToYuv of given " << inpDataLen << " values\n"
                  << std::setprecision(3) << std::fixed;
        std::cout << "Asynchronous Copy Exec Time : " << syncExecTime << "s\n";
        std::cout << "Synchronous Copy Exec Time  : " << asyncExecTime << "s\n";
        std::cout << "CPU Exec Time               : " << cpuTimeStr.str() << "\n";
    }

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed printStats() of SyncVsAsyncArrayCopy example\n";
    }
}


/******************************************************************************
* Implementation of RgbToYuv::rgbToYuvSingleCpu()                             *
******************************************************************************/
int RgbToYuv::rgbToYuvSingleCpu(DATA_TYPE *pRGBData,
                                std::vector<DATA_TYPE>& vYUV)
{
    for (int i = 0; i < inpDataLen; i+=3)
    {
        float R = (float)pRGBData[i];
        float G = (float)pRGBData[i+1];
        float B = (float)pRGBData[i+2];

        float Y = (0.257f * R) + (0.504f * G) + (0.098f * B) + 16.f;
        float V = (0.439f * R) - (0.368f * G) - (0.071f * B) + 128.f;
        float U = -(0.148f * R) - (0.291f * G) + (0.439f * B) + 128.f;

        vYUV[i] = (DATA_TYPE)(Y > 255 ? 255 : Y);
        vYUV[i+1] = (DATA_TYPE)(U > 255 ? 255 : U);
        vYUV[i+2] = (DATA_TYPE)(V > 255 ? 255 : V);
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of RgbToYuv::syncArrayCopy()                                 *
******************************************************************************/
int RgbToYuv::syncArrayCopy(std::vector<DATA_TYPE>& inputRgbData,
                            std::vector<DATA_TYPE>& outputFromSync,
                            unsigned numThreads)
{
    /**************************************************************************
    * Create a HC C++  extent of the required size and dimension              *
    **************************************************************************/
    extent<1> inpExt(numThreads);
    std::vector<DATA_TYPE>::iterator inpIter;

    /**************************************************************************
    * Create HC C++ arrays to be used                                        *
    **************************************************************************/
    array<DATA_TYPE, 1> rgbSyncArray(inpExt);
    array<DATA_TYPE, 1> yuvArray(inpExt);

    /**************************************************************************
    * Execute RGB -> YUV 4:4:4 on the GPU using copy().  The data to be used  *
    * in that iteration is copied to GPU, utilized by GPU algo & and the      *
    * resulting output is copied over to the host synchronously.              *
    **************************************************************************/
    for (unsigned i = 0; i < inpDataLen/numThreads; i++)
    {
        inpIter = inputRgbData.begin() + (i * numThreads);
        copy(inpIter, inpIter + numThreads, rgbSyncArray);
        rgbToYuvHC(rgbSyncArray, yuvArray);
        copy(yuvArray, outputFromSync.begin() + (i * numThreads));
    }

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed syncArrayCopy() of SyncVsAsyncArrayCopy example\n";
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of RgbToYuv::asyncArrayCopy()                                *
******************************************************************************/
int RgbToYuv::asyncArrayCopy(std::vector<DATA_TYPE>& inputRgbData,
                             std::vector<DATA_TYPE>& outputFromAsync,
                             unsigned numThreads)
{
    /**************************************************************************
    * Create a HC C++ extent of the required size and dimension              *
    **************************************************************************/
    extent<1> inpExt(numThreads);

    /**************************************************************************
    * Create HC C++ arrays & completion_future object to be used             *
    **************************************************************************/
    array<DATA_TYPE, 1> rgbAsyncInp1(inpExt);
    array<DATA_TYPE, 1> rgbAsyncInp2(inpExt);
    array<DATA_TYPE, 1> rgbAsyncInp3(inpExt);
    array<DATA_TYPE, 1> yuvArray1(inpExt);
    array<DATA_TYPE, 1> yuvArray2(inpExt);
    array<DATA_TYPE, 1> yuvArray3(inpExt);
    completion_future cfCopy[3];

    /**************************************************************************
    * An asynchronous copy is only scheduled and will need to be tracked by   *
    * using the completion_future objects that are returned. Here we schedule *
    * all the async copies together                                           *
    **************************************************************************/
    cfCopy[0] = copy_async(inputRgbData.begin(),
                           inputRgbData.begin() + numThreads, rgbAsyncInp1);
    cfCopy[1] = copy_async(inputRgbData.begin() + numThreads,
                           inputRgbData.begin() + (2*numThreads), rgbAsyncInp2);
    cfCopy[2] = copy_async(inputRgbData.begin() + (2*numThreads),
                           inputRgbData.begin() + (3*numThreads), rgbAsyncInp3);

    /**************************************************************************
    * Execute RGB -> YUV 4:4:4 on the GPU using copy_async(). Wait for the    *
    * async copy to complete before launching the kernel. After completion of *
    * RGB -> YUV 4:4:4 start another copy_async() to the required destination *
    * tracked by completion_future object to ensure the completion of copy    *
    **************************************************************************/
    cfCopy[0].wait();
    rgbToYuvHC(rgbAsyncInp1, yuvArray1);
    cfCopy[0] = copy_async(yuvArray1, outputFromAsync.begin());

    cfCopy[1].wait();
    rgbToYuvHC(rgbAsyncInp2, yuvArray2);
    cfCopy[1] = copy_async(yuvArray2, outputFromAsync.begin() + numThreads);

    cfCopy[2].wait();
    rgbToYuvHC(rgbAsyncInp3, yuvArray3);
    cfCopy[2] = copy_async(yuvArray3, outputFromAsync.begin() + (2*numThreads));

    cfCopy[0].wait();
    cfCopy[1].wait();
    cfCopy[2].wait();

    if(!sampleArgs->quiet)
    {
        std::cout << "Completed asyncArrayCopy() of SyncVsAsyncArrayCopy example\n";
    }

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of RgbToYuv::rgbToYuvHC()                                   *
******************************************************************************/
int RgbToYuv::rgbToYuvHC(array<DATA_TYPE, 1>& arrRGB,
                          array<DATA_TYPE, 1>& arrYUV)
{
    /**************************************************************************
    * Note that Arrays can ONLY be captured "by reference" in any overload    *
    * of parallel_for_each()                                                  *
    **************************************************************************/
    parallel_for_each(arrYUV.get_extent()/3, [&](index<1> idx) [[hc]]
    {
        DATA_TYPE i = idx[0] * 3;
        float R = (float)arrRGB[i];
        float G = (float)arrRGB[i+1];
        float B = (float)arrRGB[i+2];

        float Y = (0.257f * R) + (0.504f * G) + (0.098f * B) + 16.f;
        float V = (0.439f * R) - (0.368f * G) - (0.071f * B) + 128.f;
        float U = -(0.148f * R) - (0.291f * G) + (0.439f * B) + 128.f;

        arrYUV[i] = (DATA_TYPE)(Y > 255 ? 255 : Y);
        arrYUV[i+1] = (DATA_TYPE)(U > 255 ? 255 : U);
        arrYUV[i+2] = (DATA_TYPE)(V > 255 ? 255 : V);
    });

    return SDK_SUCCESS;
}


/******************************************************************************
* Implementation of RgbToYuv::compare()                                       *
******************************************************************************/
int RgbToYuv::compare(std::vector<unsigned>& vTest,
                      std::vector<unsigned>& vRef)
{
    int passed = SDK_SUCCESS;
    unsigned totalMismatches = 0;

    for (int i = 0; i < inpDataLen; ++i)
    {
        if (vTest[i] != vRef[i])
        {
            if (!totalMismatches)
            {
                std::cout << " mismatch found " << std::endl << "vTest["
                          << i << "] = " << vTest[i] << ", vRef[" << i << "] = "
                          << vRef[i] << std::endl;
                passed = SDK_FAILURE;
            }
            totalMismatches++;
        }
    }

    if (totalMismatches)
        std::cout << "Total number of mismatches found = "
                  << totalMismatches << std::endl;

    return passed;
}


/******************************************************************************
* Execution of program begins from here                                       *
******************************************************************************/
int main(int argc, char *argv[])
{
    std::cout << "**********************************************" << std::endl;
    std::cout << "HC C++ Synchronous Vs Asynchronous Array Copy" << std::endl;
    std::cout << "**********************************************" << std::endl;

    /**************************************************************************
    * Create an object of RgbToYuv class                                      *
    **************************************************************************/
    RgbToYuv objRgbToYuv;

    /**************************************************************************
    * Initialize the additional cmd line options of the example               *
    **************************************************************************/
    if(objRgbToYuv.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    std::cout << std::endl << std::endl;

    /**************************************************************************
    * Parse command line options                                              *
    **************************************************************************/
    if(objRgbToYuv.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Create input data vector                                                *
    **************************************************************************/
    int numElements = objRgbToYuv.getInputDataLenth();
    if (numElements <= 0)
    {
        std::cout << "Input data length should be greater than Zero" << std::endl;
        return SDK_FAILURE;
    }

    std::vector<DATA_TYPE> inputRgbData(numElements);

    /**************************************************************************
    * Initialize the random array of input samples                            *
    **************************************************************************/
    if(objRgbToYuv.setup(inputRgbData) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Create vectors to be used as output with the updated size of input len  *
    **************************************************************************/
    numElements = objRgbToYuv.getInputDataLenth();
    std::vector<DATA_TYPE> outputFromSync(numElements);
    std::vector<DATA_TYPE> outputFromAsync(numElements);
    std::vector<DATA_TYPE> refVector(numElements);

    /**************************************************************************
    * Execute RgbToYuv 4:4:4 color conversion on the the input RGB data using *
    * HC array and HC array_view                                            *
    **************************************************************************/
    if(objRgbToYuv.run(inputRgbData, outputFromSync,
                       outputFromAsync) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Verify the results that were generated                                  *
    **************************************************************************/
    if(objRgbToYuv.verifyResults(inputRgbData, outputFromSync,
                                 outputFromAsync, refVector) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Print performance statistics                                            *
    **************************************************************************/
    objRgbToYuv.printStats();

    return 0;
}
