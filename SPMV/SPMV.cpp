/**********************************************************************
Copyright ©2013 Advanced Micro Devices, Inc. All rights reserved.

HC C++ kernels within this source tree are derivatives of kernels
from the SHOC project. Source or binary distribution of this project must
disclose derivation and include the SHOC license:

SHOC 1.1.2  license Copyright ©2011, UT-Battelle, LLC. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

•   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
•   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.
•   Neither the name of Oak Ridge National Laboratory, nor UT-Battelle, LLC, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

/******************************************************************************
* Included header files                                                       *
******************************************************************************/
#include "SPMV.h"


/******************************************************************************
* Implementation of SPMV::setup()                                             *
******************************************************************************/
template <class T>
int SPMV<T>::setup()
{
    if(length % 4 != 0||length < 10)
    {
       std::cout << "\nInput error!  Length of the row of Matrix must bigger than 10 and must be a  multiple of 4."
             << std::endl;
        return SDK_FAILURE;
    }

    if(iter <= 0)
    {
        std::cerr << "Error: iter (" << iter << ") should be bigger than 0. \n\n";
        return SDK_FAILURE;
    }

    denseEdge = length;
    nonZeroProb = cNonZeroPercent / 100.0;
    nNonZeros = int((denseEdge * denseEdge) * nonZeroProb);

    if(chooseLayout == true)
    {
        Layout=SparseLayout::ELLPACKR;
    }
    // Randomize matrix and vector values.
    std::uniform_real_distribution<T> rngDist(-1.0, 1.0);
    std::mt19937 rng;

    for(int i = 0; i < nNonZeros; ++i)
    {
        csrVals.push_back(rngDist(rng));
    }

    for(int i = 0; i < denseEdge; ++i)
    {
        inVector.push_back(rngDist(rng));
        storeArrayOut.push_back(0);
    }

    // Randomize sparse matrix structure.
    std::uniform_real_distribution<T> probDist(0.0, 1.0);
    int nNonZerosAssigned = 0;

    for(int y = 0; y < denseEdge; ++y)
    {
        csrRows.push_back(nNonZerosAssigned);

        for(int x = 0; x < denseEdge; ++x)
        {
            // Fill according to non-zero rate but ensure all matrix values are accounted for by end.
            int nMatrixEntriesLeft = (denseEdge * denseEdge) - (y * denseEdge + x);
            int nNonZerosToAssign = nNonZeros - nNonZerosAssigned;
            bool fillRemaining = (nMatrixEntriesLeft <= nNonZerosToAssign);

            if(nNonZerosAssigned < nNonZeros && (probDist(rng) < nonZeroProb ||
                                                 fillRemaining))
            {
                // This dense position will correlate to a sparse value.
                csrCols.push_back(x);
                ++nNonZerosAssigned;
            }
        }
    }

    csrRows.push_back(nNonZeros);
    if(Layout == SparseLayout::ELLPACKR)
    {
        // Compute the number of non-zeros in each row, and the maximum.
        ellpackrRows.resize(denseEdge);
        ellpackrRowMax = 0;

        for(int y = 0; y < denseEdge; ++y)
        {
            ellpackrRows[y] = csrRows[y + 1] - csrRows[y];
            if(ellpackrRowMax<ellpackrRows[y])
            {
                ellpackrRowMax=ellpackrRows[y];
            }
        }

        // Build the column-major data structures.
        ellpackrVals.resize(ellpackrRowMax * denseEdge);
        ellpackrCols.resize(ellpackrRowMax * denseEdge);

        int ellpackrIdx = 0;

        for(int y = 0; y < ellpackrRowMax; ++y)
        {
            for(int x = 0; x < denseEdge; ++x)
            {
                if(csrRows[x] + y < csrRows[x + 1])
                {
                    ellpackrVals[ellpackrIdx] = csrVals[csrRows[x] + y];
                    ellpackrCols[ellpackrIdx] = csrCols[csrRows[x] + y];
                }
                else
                {
                    ellpackrVals[ellpackrIdx] = 0;
                }
                ++ellpackrIdx;
            }
        }
    }
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of SPMV::runArray()                                          *
******************************************************************************/
template <class T>
int SPMV<T>::runArray()
{
    array<T, 1>  *dCSRVals = NULL;
    array<T, 1>  *dELLPACKRVals = NULL;
    array<int,1> *dCSRCols = NULL;
    array<int,1> *dCSRRows = NULL;
    array<int,1> *dELLPACKRCols = NULL;
    array<int,1> *dELLPACKRRows = NULL;
    int status = SDK_SUCCESS;
    storeArrayOut.resize(denseEdge);
    //calculate data copy time from cpu tp gpu
    int ctgTimer = sampleTimer->createTimer();
    status = sampleTimer->resetTimer(ctgTimer);
    status = sampleTimer->startTimer(ctgTimer);
    if(Layout == SparseLayout::CSR)
    {
        dCSRVals = new array<T, 1> (nNonZeros,csrVals.begin());
        dCSRCols = new array<int,1> (nNonZeros,csrCols.begin());
        dCSRRows = new array<int,1> (denseEdge + 1, csrRows.begin());
    }
    else
    {
        dELLPACKRVals = new array<T, 1> (ellpackrRowMax * denseEdge,
                                         ellpackrVals.begin());
        dELLPACKRCols = new array<int,1> (ellpackrRowMax * denseEdge,
                                          ellpackrCols.begin());
        dELLPACKRRows = new array<int,1> (denseEdge, ellpackrRows.begin());
    }
    array<T, 1> dInVector (denseEdge, inVector.begin());
    array<T, 1> dOutVector(denseEdge);
    status = sampleTimer->stopTimer(ctgTimer);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    cpToDeviceTime = (double)(sampleTimer->readTimer(ctgTimer));
    //warm up
    if(Layout == SparseLayout::CSR)
    {
        SPMVCSR(*dCSRVals, *dCSRCols, *dCSRRows, dInVector, dOutVector, denseEdge,
                nNonZeros);
    }
    else
    {
        SPMVELLPACKR(*dELLPACKRVals, *dELLPACKRCols, *dELLPACKRRows, dInVector,
                     dOutVector, denseEdge);
    }
    accelerator_view accView = accelerator().get_default_view();
    accView.flush();
    accView.wait();

    //calculate total kernel execute time
    int timer = sampleTimer->createTimer();
    status = sampleTimer->resetTimer(timer);
    status = sampleTimer->startTimer(timer);
    for(int run = 0; run < iter; ++run)
    {
        // Execute and time the kernel.
        if(Layout == SparseLayout::CSR)
        {
            SPMVCSR(*dCSRVals, *dCSRCols, *dCSRRows, dInVector, dOutVector, denseEdge,
                    nNonZeros);
        }
        else
        {
            SPMVELLPACKR(*dELLPACKRVals, *dELLPACKRCols, *dELLPACKRRows, dInVector,
                         dOutVector, denseEdge);
        }

        accView.flush();
    }
    accView.wait();
    status = sampleTimer->stopTimer(timer);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    totalKernelTime = (double)(sampleTimer->readTimer(timer));
    averageKernelTime = totalKernelTime / iter;

    //calculate data copy time from gpu to cpu
    int gtcTimer = sampleTimer->createTimer();
    status = sampleTimer->resetTimer(gtcTimer);
    status = sampleTimer->startTimer(gtcTimer);
    copy(dOutVector, storeArrayOut.begin());
    status = sampleTimer->stopTimer(gtcTimer);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    cpToHostTime = (double)(sampleTimer->readTimer(gtcTimer));

    totalTime = cpToDeviceTime + totalKernelTime + cpToHostTime;
    if(dCSRVals != NULL)
    {
        delete dCSRVals;
    }
    if(dCSRCols != NULL)
    {
        delete dCSRCols;
    }
    if(dCSRRows != NULL)
    {
        delete dCSRRows;
    }
    if(dELLPACKRVals != NULL)
    {
        delete dELLPACKRVals;
    }
    if(dELLPACKRCols != NULL)
    {
        delete dELLPACKRCols;
    }
    if(dELLPACKRRows != NULL)
    {
        delete dELLPACKRRows;
    }
    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of SPMV::runArray_View()                                      *
******************************************************************************/
template <class T>
int SPMV<T>::runArray_View()
{
    array_view<T, 1>  *dCSRVals = NULL;
    array_view<T, 1>  *dELLPACKRVals = NULL;
    array_view<int,1> *dCSRCols = NULL;
    array_view<int,1> *dCSRRows = NULL;
    array_view<int,1> *dELLPACKRCols = NULL;
    array_view<int,1> *dELLPACKRRows = NULL;
    int status = SDK_SUCCESS;
    storeArrayOut.resize(denseEdge);

    if(Layout == SparseLayout::CSR)
    {
        dCSRVals = new array_view<T, 1> (nNonZeros, csrVals);
        dCSRCols = new array_view<int,1> (nNonZeros, csrCols);
        dCSRRows = new array_view<int,1> (denseEdge + 1, csrRows);
    }
    else
    {
        dELLPACKRVals = new array_view<T, 1> (ellpackrRowMax * denseEdge, ellpackrVals);
        dELLPACKRCols = new array_view<int,1> (ellpackrRowMax * denseEdge,
                                               ellpackrCols);
        dELLPACKRRows = new array_view<int,1> (denseEdge, ellpackrRows);
    }
    array_view<T, 1> dInVector (denseEdge, inVector);
    array_view<T, 1> dOutVector(denseEdge,storeArrayOut);
    // accelerator view for synchronize
    accelerator_view accView = accelerator().get_default_view();

    //warm up
    if(Layout == SparseLayout::CSR)
    {
        SPMVCSR(*dCSRVals, *dCSRCols, *dCSRRows, dInVector, dOutVector, denseEdge,
                nNonZeros);
    }
    else
    {
        SPMVELLPACKR(*dELLPACKRVals, *dELLPACKRCols, *dELLPACKRRows, dInVector,
                     dOutVector, denseEdge);
    }
    dOutVector.synchronize();

    //calculate total time
    int timer = sampleTimer->createTimer();
    status = sampleTimer->resetTimer(timer);
    status = sampleTimer->startTimer(timer);
    for(int run = 0; run < iter; ++run)
    {
        // Execute and time the kernel.
        if(Layout == SparseLayout::CSR)
        {
            SPMVCSR(*dCSRVals, *dCSRCols, *dCSRRows, dInVector, dOutVector, denseEdge,
                    nNonZeros);
        }
        else
        {
            SPMVELLPACKR(*dELLPACKRVals, *dELLPACKRCols, *dELLPACKRRows, dInVector,
                         dOutVector, denseEdge);
        }
        accView.flush();
    }
    dOutVector.synchronize();
    status = sampleTimer->stopTimer(timer);
    if(status != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    totalTime = (double)(sampleTimer->readTimer(timer));
    if(dCSRVals != NULL)
    {
        delete dCSRVals;
    }
    if(dCSRCols != NULL)
    {
        delete dCSRCols;
    }
    if(dCSRRows != NULL)
    {
        delete dCSRRows;
    }
    if(dELLPACKRVals != NULL)
    {
        delete dELLPACKRVals;
    }
    if(dELLPACKRCols != NULL)
    {
        delete dELLPACKRCols;
    }
    if(dELLPACKRRows != NULL)
    {
        delete dELLPACKRRows;
    }
    return SDK_SUCCESS;

}

/******************************************************************************
* Implementation of SPMV::run()                                               *
******************************************************************************/
template <class T>
int SPMV<T>::run()
{
    // Report mean/SD/throughput to the user.
    std::stringstream modeStr;
    modeStr << (sizeof(T) == 4 ? "SP" : "DP") << " ";
    if(Layout == SparseLayout::CSR)
    {
        modeStr << "SPMV " <<  "CSR";
    }
    else
    {
        modeStr << "SPMV " <<  "ELLPACKR";
    }
    modeStr << " " << length << "x" << length;
    modeStr << ", " << cNonZeroPercent << "% non-zero";
    std::cout << "Run " << modeStr.str() << std::fixed << std::setprecision(
                  3) << "\n";
    if(arrayview)
    {
        if(runArray_View() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }
    else
    {
        if(runArray() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }
    if(!sampleArgs->quiet)
    {
        printArray();
    }

    return SDK_SUCCESS;
}

/******************************************************************************
* Implementation of SPMV::SPMVCSR()                                           *
******************************************************************************/
template <class T>
void SPMV<T>::SPMVCSR(array<T, 1>   &csrVals,
                      array<int, 1> &csrCols,
                      array<int, 1> &csrRows,
                      array<T, 1>   &inVector,
                      array<T, 1>   &outVector,
                      const int denseEdge,
                      const int nNonZeros)
{extent<1> ex(denseEdge * cCSRNThreadsPerRow);
  tiled_extent<1> tiled_ex = ex.tile(cCSRNThreadsPerTile);
    parallel_for_each(tiled_ex,
                      [&, denseEdge, nNonZeros](tiled_index<1>& idx) [[hc]]
    {
        // Zero partial sum for this thread.
        tile_static T partialSums[cCSRNThreadsPerTile];

        // Compute the matrix row our thread is responsible for.
        const int rowIdx = (idx.tile[0] * (cCSRNThreadsPerTile / cCSRNThreadsPerRow))
                           + (idx.local[0] / cCSRNThreadsPerRow);

        // Compute start/end indices of our matrix row in the sparse value array.
        const int colStart = csrRows[rowIdx    ];
        const int colEnd   = csrRows[rowIdx + 1];

        // Build a partial sum for the row, striding by cCSRNThreadsPerRow.
        T thrPartialSum = 0;
        const int localIdxInRow = idx.local[0] & (cCSRNThreadsPerRow - 1);

        for(int i = colStart + localIdxInRow;
                i < colEnd;
                i += cCSRNThreadsPerRow)
        {
            thrPartialSum += csrVals[i] * inVector[csrCols[i]];
        }

        // Reduce partial sums across the tile.
        partialSums[idx.local[0]] = thrPartialSum;
        idx.barrier.wait_with_tile_static_memory_fence();

        if(localIdxInRow < 16)
        {
            partialSums[idx.local[0]] += partialSums[idx.local[0] + 16];
        }
        idx.barrier.wait_with_tile_static_memory_fence();
        if(localIdxInRow <  8)
        {
            partialSums[idx.local[0]] += partialSums[idx.local[0] +  8];
        }
        idx.barrier.wait_with_tile_static_memory_fence();
        if(localIdxInRow <  4)
        {
            partialSums[idx.local[0]] += partialSums[idx.local[0] +  4];
        }
        idx.barrier.wait_with_tile_static_memory_fence();
        if(localIdxInRow <  2)
        {
            partialSums[idx.local[0]] += partialSums[idx.local[0] +  2];
        }
        idx.barrier.wait_with_tile_static_memory_fence();

        if(localIdxInRow == 0)
        {
            outVector[rowIdx] = partialSums[idx.local[0]] + partialSums[idx.local[0] + 1];
        }
    });
}

/******************************************************************************
* Implementation of SPMV::SPMVCSR()                                           *
******************************************************************************/
template <class T>
void SPMV<T>::SPMVCSR(array_view<T, 1>  &csrVals_view,
                      array_view<int,1> &csrCols_view,
                      array_view<int,1> &csrRows_view,
                      array_view<T, 1>  &inVector_view,
                      array_view<T, 1>  &outVector_view,
                      const int denseEdge,
                      const int nNonZeros)
{
extent<1> ex(denseEdge * cCSRNThreadsPerRow);
  tiled_extent<1> tiled_ex = ex.tile(cCSRNThreadsPerTile);
    parallel_for_each(tiled_ex,
                      [=](tiled_index<1>& idx) [[hc]]
    {
        // Zero partial sum for this thread.
        tile_static T partialSums[cCSRNThreadsPerTile];

        // Compute the matrix row our thread is responsible for.
        const int rowIdx = (idx.tile[0] * (cCSRNThreadsPerTile / cCSRNThreadsPerRow))
                           + (idx.local[0] / cCSRNThreadsPerRow);

        // Compute start/end indices of our matrix row in the sparse value array.
        const int colStart = csrRows_view[rowIdx];
        const int colEnd   = csrRows_view[rowIdx + 1];

        // Build a partial sum for the row, striding by cCSRNThreadsPerRow.
        T thrPartialSum = 0;
        const int localIdxInRow = idx.local[0] & (cCSRNThreadsPerRow - 1);

        for(int i = colStart + localIdxInRow;
                i < colEnd;
                i += cCSRNThreadsPerRow)
        {
            thrPartialSum += csrVals_view[i] * inVector_view[csrCols_view[i]];
        }

        // Reduce partial sums across the tile.
        partialSums[idx.local[0]] = thrPartialSum;
        idx.barrier.wait_with_tile_static_memory_fence();

        if(localIdxInRow < 16)
        {
            partialSums[idx.local[0]] += partialSums[idx.local[0] + 16];
        }
        idx.barrier.wait_with_tile_static_memory_fence();
        if(localIdxInRow <  8)
        {
            partialSums[idx.local[0]] += partialSums[idx.local[0] +  8];
        }
        idx.barrier.wait_with_tile_static_memory_fence();
        if(localIdxInRow <  4)
        {
            partialSums[idx.local[0]] += partialSums[idx.local[0] +  4];
        }
        idx.barrier.wait_with_tile_static_memory_fence();
        if(localIdxInRow <  2)
        {
            partialSums[idx.local[0]] += partialSums[idx.local[0] +  2];
        }
        idx.barrier.wait_with_tile_static_memory_fence();

        if(localIdxInRow == 0)
        {
            outVector_view[rowIdx] = partialSums[idx.local[0]] + partialSums[idx.local[0] +
                                     1];
        }
    });
}

/******************************************************************************
* Implementation of SPMV::SPMVELLPACKR()                                      *
******************************************************************************/
template <class T>
void SPMV<T>::SPMVELLPACKR(array<T, 1>   &ellpackrVals,
                           array<int, 1> &ellpackrCols,
                           array<int, 1> &ellpackrRows,
                           array<T, 1>   &inVector,
                           array<T, 1>   &outVector,
                           const int denseEdge)
{
    parallel_for_each(hc::extent<1> (denseEdge),
                      [&, denseEdge](index<1> idx) [[hc]]
    {
        T sum = 0;
        const int rowLength = ellpackrRows[idx[0]];

        for(int i = 0; i < rowLength; ++i)
        {
            const int offset = i * denseEdge + idx[0];
            sum += ellpackrVals[offset] * inVector[ellpackrCols[offset]];
        }

        outVector[idx[0]] = sum;
    });
}

/******************************************************************************
* Implementation of SPMV::SPMVELLPACKR()                                      *
******************************************************************************/
template <class T>
void SPMV<T>::SPMVELLPACKR(array_view<T, 1>  &ellpackrVals_view,
                           array_view<int,1> &ellpackrCols_view,
                           array_view<int,1> &ellpackrRows_view,
                           array_view<T, 1>  &inVector_view,
                           array_view<T, 1>  &outVector_view,
                           const int denseEdge)
{
    parallel_for_each(hc::extent<1> (denseEdge),
                      [=](index<1> idx) [[hc]]
    {
        T sum = 0;
        const int rowLength = ellpackrRows_view[idx[0]];

        for(int i = 0; i < rowLength; ++i)
        {
            const int offset = i * denseEdge + idx[0];
            sum += ellpackrVals_view[offset] * inVector_view[ellpackrCols_view[offset]];
        }

        outVector_view[idx[0]] = sum;
    });
}

/******************************************************************************
* Implementation of SPMV::verifyResults()                                     *
******************************************************************************/
template <class T>
int SPMV<T>::verifyResults()
{

    //printStatus(modeStr.str(), nGFLOP, "GFLOPS");

    if(sampleArgs->verify)
    {
       std::cout << "  [Validating... ";
       std::cout.flush();

        // Compare the reference and AMP-computed arrays.

        for(int y = 0; y < denseEdge; ++y)
        {
            T sum = 0;

            for(int i = csrRows[y]; i < csrRows[y + 1]; ++i)
            {
                sum += csrVals[i] * inVector[csrCols[i]];
            }

            if(fabs(sum - storeArrayOut[y]) > 0.001)
            {
                std::cerr << "failed at row " << y << ": ";
                std::cerr << "expected " << sum << " but have " << outVector[y] << "]\n\n";
                std::cerr << "iteration aborted due to error\n";
                exit(1);
            }
        }

        std::cout << "passed]\n";
    }

    std::cout << "\n";
    std::cout.flush();
    return SDK_SUCCESS;
}
template<class T>
int SPMV<T>::printArray()
{
    std::cout<<" "<<std::endl;
    int number=nNonZeros;
    if(number>64)
    {
        number=64;
    }
    std::cout<<"Input (The first nonZero ";
    std::cout<< number ;
    std::cout<<" elements)"<<std::endl;
    for (int i = 0; i < number; i++)
    {
        std::cout<<std::setw(12)<<inVector[i]<<" ";
        if((i + 1) % 10 == 0 )
        {
            std::cout<<std::endl;
        }
    }
    std::cout<<std::endl;
    std::cout<<" "<<std::endl;
    int out=denseEdge;
    if(out>64)
    {
        out=64;
    }
    std::cout<<"Output (The first ";
    std::cout<<out;
    std::cout<<" elements)"<<std::endl;
    for (int i = 0; i < out; i++)
    {
        std::cout<<std::setw(12)<<storeArrayOut[i]<<" ";
        if((i + 1) % 10 == 0 )
        {
            std::cout<<std::endl;
        }
    }
    std::cout<<std::endl;

    return SDK_FAILURE;
}
/******************************************************************************
* Implementation of SPMV::printStatus()                                       *
******************************************************************************/
template <class T>
void SPMV<T>::printStatus()
{
    // Report mean/SD/throughput to the user.
    std::stringstream modeStr;
    std::string workRateUnit = "GFLOPS";
    modeStr << (sizeof(T) == 4 ? "SP" : "DP") << " ";
    if(Layout == SparseLayout::CSR)
    {
        modeStr << "SPMV " <<  "CSR";
    }
    else
    {
        modeStr << "SPMV " <<  "ELLPACK";
    }
    modeStr << " " << length << "x" << length;
    modeStr << ", " << cNonZeroPercent << "% non-zero";
    std::string modeText = modeStr.str();
    double nGFLOP = double(2 * nNonZeros) / 1000000000.0;
    double workDone = nGFLOP;
    if(iter >= 1)
    {
        if(!arrayview)
        {
            double workRate = workDone / averageKernelTime;
            std::cout << "Using array!" << std::endl;
            int workRatePrecision = (workRate >= 100.0 ? 0 : 1);
            std::cout << modeText << std::fixed << std::setprecision(6) << " finished!\n";
            if(sampleArgs->timing)
            {
                std::cout << "\nTime Information" << std::endl;
                std::cout << "(Total time(sec): " << totalTime << ") " << std::endl;

                std::string strArray[4] = { "Data Transfer to Accelerator(sec)", "Mean Execution Time (sec)", workRateUnit, "Data Transfer to Host(sec)"};
                std::string stats[4];
                stats[0] = toString<double> (cpToDeviceTime, std::dec);
                stats[1] = toString<double> (averageKernelTime, std::dec);
                stats[2] = toString<double> (workRate, std::dec);
                stats[3] = toString<double> (cpToHostTime, std::dec);
                printStatistics(strArray, stats, 4);
            }
        }
        else
        {
            // Print time information while using array_view
            std::cout << "Using array_view!" << std::endl;
            std::cout << modeText << std::fixed << std::setprecision(6) << " finished!\n";
            if(sampleArgs->timing)
            {
                std::cout << "(Total time(sec): " << totalTime << ") " << std::endl;
            }
        }

    }
}

/******************************************************************************
* Implementation of SPMV::initialize()                                        *
******************************************************************************/
template <class T>
int SPMV<T>::initialize()
{
    //Call base class Initialize to get default configuration
    if(sampleArgs-> initialize())
    {
        return SDK_FAILURE;
    }

    Option* num_length = new Option;
    CHECK_ALLOCATION(num_length,"num_length memory allocation failed");
    num_length->_sVersion = "l";
    num_length->_lVersion = "length";
    num_length->_description = "Length of row / column of square matrix.";
    num_length->_type = CA_ARG_INT;
    num_length->_value = &length;
    sampleArgs->AddOption(num_length);
    delete num_length;

    Option* arrayView = new Option;
    CHECK_ALLOCATION(arrayView,"Memory Allocation error.(arrayview)");
    arrayView->_sVersion = "V";
    arrayView->_lVersion = "array_view";
    arrayView->_description = "use array_view instead of array.";
    arrayView->_type = CA_NO_ARGUMENT;
    arrayView->_value = &arrayview;
    sampleArgs->AddOption(arrayView);
    delete arrayView;

    Option* iteration_option = new Option;
    CHECK_ALLOCATION(iteration_option,"Memory Allocation error.(iteration_option)");
    iteration_option->_sVersion = "i";
    iteration_option->_lVersion = "iterations";
    iteration_option->_description = "Number of times to repeat each algorithm.";
    iteration_option->_type = CA_ARG_INT;
    iteration_option->_value = &iter;
    sampleArgs->AddOption(iteration_option);
    delete iteration_option;

    Option* choose = new Option;
    CHECK_ALLOCATION(choose,"choose sparseLayout::ELLPACKR");
    choose->_sVersion = "p";
    choose->_lVersion = "ellpackr";
    choose->_description =
        " Use Ellpackr format instead of Compressed Sparse Row (CSR) format";
    choose->_type = CA_NO_ARGUMENT;
    choose->_value = &chooseLayout;
    sampleArgs->AddOption(choose);
    delete choose;

    return SDK_SUCCESS;
}

int main(int argc, char* argv[])
{
    std::cout << "**********************************************" << std::endl;
    std::cout << "                      SPMV" << std::endl;
    std::cout << "**********************************************" << std::endl;
    std::cout << std::endl << std::endl;

    int status = SDK_SUCCESS;
    /**************************************************************************
    * Create an object of SPMV class                                          *
    **************************************************************************/
    SPMV <float> myInstance;

    /**************************************************************************
    * Initialize options of the sample                                        *
    **************************************************************************/
    status = myInstance.initialize();
    if(status != SDK_SUCCESS)
    {
        std::cout << "SPMV initialize failed." << std::endl;
        return SDK_FAILURE;
    }
    /**************************************************************************
    * Parse command line options                                              *
    **************************************************************************/
    if(myInstance.sampleArgs->parseCommandLine(argc,argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /*******************************************************************************
    * Print all devices                                                            *
    *******************************************************************************/
    myInstance.sampleArgs->printDeviceList();

    /*******************************************************************************
    * Set default accelerator                                                      *
    *******************************************************************************/
    if(myInstance.sampleArgs->setDefaultAccelerator() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Initialize the input data                                               *
    **************************************************************************/
    status = myInstance.setup();
    if(status != SDK_SUCCESS)
    {
        std::cout << "SPMV setup failed." << std::endl;
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Execute SPMV                                                            *
    **************************************************************************/
    status = myInstance.run();
    if(status != SDK_SUCCESS)
    {
        std::cout << "SPMV run failed." << std::endl;
        return SDK_FAILURE;
    }

    /**************************************************************************
    * Print some timer information                                            *
    **************************************************************************/
    myInstance.printStatus();

    /**************************************************************************
    * Verify the results that were generated                                  *
    **************************************************************************/
    if(myInstance.verifyResults() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }
    return SDK_SUCCESS;
}
