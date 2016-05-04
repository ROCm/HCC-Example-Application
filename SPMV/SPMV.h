/**********************************************************************
Copyright ©2013 Advanced Micro Devices, Inc. All rights reserved.

HC C++ kernels within this source tree are derivatives of kernels
from the SHOC project. Source or binary distribution of this project must
disclose derivation and include the SHOC license:

SHOC 1.1.2  license Copyright ©2011, UT-Battelle, LLC. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

•	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
•	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.
•	Neither the name of Oak Ridge National Laboratory, nor UT-Battelle, LLC, nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#ifndef SPMV_H_
#define SPMV_H_

/******************************************************************************
* Defined macros                                                              *
* cCSRNThreadsPerRow: Number of threads working on a single matrix row for CSR.
* cCSRNThreadsPerTile: Number of threads per HC tile for CSR.
* cNonZeroPercent: Percentage of non-zero elements in sparse matrix.
* SAMPLE_VERSION: Sample version of app_sdk sample.
******************************************************************************/
#define cCSRNThreadsPerRow 32
#define cCSRNThreadsPerTile 128
#define cNonZeroPercent 1

#define SAMPLE_VERSION "AMD-APP-SDK-v2.9.233.1"

/******************************************************************************
* Included header files                                                       *
******************************************************************************/
#include <hc.hpp>
#include <iostream>
#include <string.h>

#include "HCUtil.hpp"

#include <algorithm>
#include <random>
#include <sstream>

/******************************************************************************
* namespace hc                                                                *
******************************************************************************/
using namespace hc;
using namespace appsdk;

/**************************************************************************
* SparseLayout                                                            *
* Class implements CSR and ELLPACKR sample                                *
**************************************************************************/
enum class SparseLayout {
    CSR,
    ELLPACKR
};

/**************************************************************************
* SPMV                                                                    *
* Class implements CSR and ELLPACKR sample                                *
**************************************************************************/
template <class T>
class SPMV 
{
    int                denseEdge;   /**< The row and col of matrix */
    double           nonZeroProb;   /**< The percentage of non-zero elements*/
    int                nNonZeros;   /**< The number of non-zero elements*/

    std::vector<T>      storeArrayOut;   /**< Vector storeArrayOut*/
    std::vector<T>            csrVals;   /**< Vector csrVals*/
    std::vector<int>          csrCols;   /**< Vector csrCols*/
    std::vector<int>          csrRows;   /**< Vector csrRows*/
    std::vector<T>           inVector;   /**< Vector inVector*/
    std::vector<T>          outVector;   /**< Vector outVector*/

    std::vector<T>       ellpackrVals;   /**< Vector ellpackrVals*/
    std::vector<int>     ellpackrCols;   /**< Vector ellpackrCols*/
    std::vector<int>     ellpackrRows;   /**< Vector ellpackrRows*/
    int           ellpackrRowMax;   /**< Row max*/

    SparseLayout          Layout;
    double                 bytes;   /**< Number of Bytes handled by each thread */
    int                   length;   /**< Length of the row / column of Matrix */
    int                     iter;   /**< Number of iteratons for kernel excution */

    bool               arrayview;   /**< Array view support */
    bool            chooseLayout;   /**< Choose CSR or ELLPACKR formation*/

    double    totalTime;           /**< Total time */
    double    totalKernelTime;     /**< Total kernel execution time */
    double    cpToDeviceTime;        /**< data copy time from CPU to GPU */
    double    cpToHostTime;        /**< data copy time from GPU to CPU */
    double    averageKernelTime;   /**<average kernel execute time */

    SDKTimer *sampleTimer;                     /**< Timer object */
public:
    HCCommandArgs *sampleArgs;               /**< Command line helper object */

    /**
    *******************************************************************
    * @brief Constructor
    * Initialize member variables
    ******************************************************************/
    SPMV()
    {
        denseEdge = 0;
        nonZeroProb = 0.0;
        nNonZeros = 0;
        chooseLayout = false;//choose CSR formation
        Layout = SparseLayout::CSR;
        bytes = 0.0;

        length = 16384;
        iter = 10;
    
        totalTime = 0;
        totalKernelTime = 0;
        arrayview = false;//use nonzerocopy

        cpToDeviceTime = 0;
        cpToHostTime = 0;
        averageKernelTime = 0;
        sampleTimer =  new SDKTimer();
        sampleArgs = new HCCommandArgs();
        sampleArgs->sampleVerStr = SAMPLE_VERSION;
	}

    /**
    *******************************************************************
    * @brief Override from HCSample. Initialize
    * command line parser, add custom options
    * @return SDK_SUCCESS on success and SDK_FAILURE on failure
    ******************************************************************/
    int initialize();

    /**
    ************************************************************************
    * Override from HCSample
    * @brief Allocate and initialize the input vector with random values and
    *  set zero to the output vectors
    * @return SDK_SUCCESS on success and SDK_FAILURE on failed
    ***********************************************************************/
    int setup();

    /**
    *******************************************************************
    * @fn SPMVCSR
    ******************************************************************/
    void SPMVCSR(array<T, 1>  &csrVals,
                array<int,1> &csrCols,
                array<int,1> &csrRows,
                array<T, 1>  &inVector,
                array<T, 1>  &outVector,
                const int denseEdge,
                const int nNonZeros);

    /**
    *******************************************************************
    * @fn SPMVELLPACKR
    ******************************************************************/
    void SPMVELLPACKR(array<T, 1>  &ellpackrVals,
                     array<int,1> &ellpackrCols,
                     array<int,1> &ellpackrRows,
                     array<T, 1>  &inVector,
                     array<T, 1>  &outVector,
                     const int denseEdge);
    /**
    *******************************************************************
    * @brief OverLoad function
    * used for Array_View
    * @return SDK_SUCCESS on success and SDK_FAILURE on failed
    ******************************************************************/
    void SPMVCSR(array_view<T, 1>  &csrVals_view,
                array_view<int,1> &csrCols_view,
                array_view<int,1> &csrRows_view,
                array_view<T, 1>  &inVector_view,
                array_view<T, 1>  &outVector_view,
                const int denseEdge,
                const int nNonZeros);
    /**
    *******************************************************************
    * @fn SPMVELLPACKR
    ******************************************************************/
    void SPMVELLPACKR(array_view<T, 1>  &ellpackrVals_view,
                     array_view<int,1> &ellpackrCols_view,
                     array_view<int,1> &ellpackrRows_view,
                     array_view<T, 1>  &inVector_view,
                     array_view<T, 1>  &outVector_view,
                     const int denseEdge);
    /**
    *******************************************************************
    * @brief Run SPMVHC
    * @return SDK_SUCCESS on success and SDK_FAILURE on failure
    ******************************************************************/
    int run();

    /**
    *******************************************************************
    * @brief Run SPMVHC, using array
    * @return SDK_SUCCESS on success and SDK_FAILURE on failure
    ******************************************************************/
    int runArray();

    /**
    *******************************************************************
    * @brief Run SPMVHC, using array_view
    * @return SDK_SUCCESS on success and SDK_FAILURE on failure
    ******************************************************************/
    int runArray_View();

    /**
    *******************************************************************
    * @breif Override from HCSample
    * Verify against reference implementation
    * @return SDK_SUCCESS on success and SDK_FAILURE on failure
    ******************************************************************/
    int verifyResults();

    /**
    *******************************************************************
    * @fn printStats
    * @return SDK_SUCCESS on success and SDK_FAILURE on failure
    ******************************************************************/
    void printStatus();
    int printArray();
};
#endif
