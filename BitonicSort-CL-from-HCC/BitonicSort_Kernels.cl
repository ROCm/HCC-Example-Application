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

/*
 * For a description of the algorithm and the terms used, please see the
 * documentation for this sample.
 *
 * One invocation of this kernel, i.e one work thread writes two output values.
 * Since every pass of this algorithm does width/2 comparisons, each compare
 * operation is done by one work thread.
 * 
 * Depending of the direction of sort for the work thread, the output values
 * are written either as greater value to left element or lesser value to the
 * left element. Right element and left element are the two elements we are 
 * comparing and "left" is the element with a smaller index into the array.
 *
 * if direction is CL_TRUE, i.e evaluates to non zero, it means "increasing".
 *
 * For an explanation of the terms "blockWidth", "sameDirectionBlockWidth",
 * stage, pass, pairDistance please go through the document shipped with this
 * sample.
 *
 * Since an explanation of the terms and the code here would be quite lengthy,
 * confusing and will greatly reduce the readability of this kernel, the code 
 * has been explained in detail in the document mentioned above.
 */

__kernel 
void bitonicSort(__global uint * theArray,
                 const uint stage, 
                 const uint passOfStage,
                 const uint direction)
{
    uint sortIncreasing = direction;
    uint threadId = get_global_id(0);
    
    uint pairDistance = 1 << (stage - passOfStage);
    uint blockWidth   = 2 * pairDistance;

    uint leftId = (threadId % pairDistance) 
                   + (threadId / pairDistance) * blockWidth;

    uint rightId = leftId + pairDistance;
    
    uint leftElement = theArray[leftId];
    uint rightElement = theArray[rightId];
    
    uint sameDirectionBlockWidth = 1 << stage;
    
    if((threadId/sameDirectionBlockWidth) % 2 == 1)
        sortIncreasing = 1 - sortIncreasing;

    uint greater;
    uint lesser;
    if(leftElement > rightElement)
    {
        greater = leftElement;
        lesser  = rightElement;
    }
    else
    {
        greater = rightElement;
        lesser  = leftElement;
    }
    
    if(sortIncreasing)
    {
        theArray[leftId]  = lesser;
        theArray[rightId] = greater;
    }
    else
    {
        theArray[leftId]  = greater;
        theArray[rightId] = lesser;
    }
}


