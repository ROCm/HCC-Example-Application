#################################################################################
# Copyright ©2013 Advanced Micro Devices, Inc. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# •	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# •	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
#  other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#################################################################################


set( FOLDER_NAME C++Amp )
set( SUBDIRECTORIES ArrayBandwidth
HCFFT
MD
SPMV
SyncVsAsyncArrayCopy )

############################################################################

set(CMAKE_SUPPRESS_REGENERATION TRUE)
cmake_minimum_required( VERSION 2.6.0 )
project( ${FOLDER_NAME} )

# Group samples by folder
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set( FOLDER_GROUP ${FOLDER_GROUP}/${FOLDER_NAME} )

if( MSVC)
	set(SUBDIRECTORIES  ${SUBDIRECTORIES} ${SUBDIRECTORIES_WIN})
elseif (UNIX)
  include_directories( include/SDKUtil )

  # set the compile options for Kalmar 
  execute_process(COMMAND hcc-config  --cxxflags OUTPUT_VARIABLE KALMAR_COMPILE_FLAGS)
  set( COMPILER_FLAGS "${COMPILER_FLAGS} ${KALMAR_COMPILE_FLAGS}" )
 
  set( COMPILER_FLAGS "${COMPILER_FLAGS} -mcmodel=small" )

  # set the link options for Kalmar
  execute_process(COMMAND hcc-config  --ldflags OUTPUT_VARIABLE KALMAR_LINKER_FLAGS)
  set( LINKER_FLAGS "${LINKER_FLAGS} ${KALMAR_LINKER_FLAGS}")

  set( LINKER_FLAGS "${LINKER_FLAGS} -mcmodel=small -lm")

  # Set output directory to bin
  set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/${BITNESS_SUFFIX})

endif()

# Auto-select bitness based on platform
if( NOT BITNESS )
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(BITNESS 64)
        set(BITNESS_SUFFIX x86_64)
    else()
        set(BITNESS 32)
        set(BITNESS_SUFFIX x86)
    endif()
endif()

# Set output directory to bin
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin/${BITNESS_SUFFIX})

set( SUBDIRECTORIES_WIN "")
foreach( subdir ${SUBDIRECTORIES} )
    add_subdirectory( ${subdir} )
endforeach( subdir )

