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

#include <assert.h>
#include <iostream>
#include <fstream>

#include <hsa/hsa.h>

#include "hsa_utils.hpp"

uint64_t 
load_hsa_code_object(const char *raw_code_object, size_t code_object_size, const char *kernelName, hsa_agent_t agent)
{
  hsa_status_t hsa_status = HSA_STATUS_SUCCESS;

  // Deserialize code object.
  hsa_code_object_t code_object = {0};
  hsa_status = hsa_code_object_deserialize((void*)raw_code_object, code_object_size, NULL, &code_object);
  assert(HSA_STATUS_SUCCESS == hsa_status);
  assert(0 != code_object.handle);

  // Create executable.
  hsa_executable_t hsaExecutable;
  hsa_status = hsa_executable_create(HSA_PROFILE_FULL, HSA_EXECUTABLE_STATE_UNFROZEN, NULL, &hsaExecutable);
  assert(HSA_STATUS_SUCCESS == hsa_status);

  // Load code object.
  hsa_status = hsa_executable_load_code_object(hsaExecutable, agent, code_object, NULL);
  assert(HSA_STATUS_SUCCESS == hsa_status);

  // Freeze executable.
  hsa_status = hsa_executable_freeze(hsaExecutable, NULL);
  assert(HSA_STATUS_SUCCESS == hsa_status);

  // Get symbol handle.
  hsa_executable_symbol_t kernelSymbol;
  hsa_status = hsa_executable_get_symbol(hsaExecutable, NULL, kernelName, agent, 0, &kernelSymbol);
  assert(HSA_STATUS_SUCCESS == hsa_status);

  // Get code handle.
  uint64_t codeHandle;
  hsa_status = hsa_executable_symbol_get_info(kernelSymbol, HSA_EXECUTABLE_SYMBOL_INFO_KERNEL_OBJECT, &codeHandle);
  assert(HSA_STATUS_SUCCESS == hsa_status);

  return codeHandle;
}



/*
 * Extract the code handle for the specified kernelName from the specified fileName
 * Returns a 64-bit code object which can be used with an AQL packet
 */
uint64_t 
load_hsa_code_object_from_file(const char *fileName, const char *kernelName, hsa_agent_t agent)
{
  // Open file.
  std::ifstream file(fileName, std::ios::in | std::ios::binary);
  assert(file.is_open() && file.good());

  // Find out file size.
  file.seekg(0, file.end);
  size_t size = file.tellg();
  file.seekg(0, file.beg);

  // Allocate memory for raw code object.
  char *raw_code_object = (char*)malloc(size);
  assert(raw_code_object);

  // Read file contents.
  file.read(raw_code_object, size);

  // Close file.
  file.close();

  uint64_t codeHandle = load_hsa_code_object(raw_code_object, size, kernelName, agent);

  // Free raw code object memory.
  free((void*)raw_code_object);

  return codeHandle;
};


