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

set( SAMPLE_NAME ArrayBandwidth )
set( SOURCE_FILES ArrayBandwidth.cpp )

############################################################################

set(CMAKE_SUPPRESS_REGENERATION TRUE)
cmake_minimum_required( VERSION 2.6.0 )
project( ${SAMPLE_NAME} )

file(GLOB INCLUDE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.h" )

add_executable( ${SAMPLE_NAME} ${SOURCE_FILES} ${INCLUDE_FILES} ${EXTRA_FILES})

if (WIN32)
  include_directories( ../../include/SDKUtil $ENV{AMDAPPSDKROOT}/include/SDKUtil )

  # Samples can specify additional libs/flags using EXTRA* defines
  add_definitions( "/W3 /D_CRT_SECURE_NO_WARNINGS /wd4005 /wd4996 /nologo" )
  if(${SAMPLE_NAME} MATCHES "HCFFT")
  	if(CMAKE_CXX_FLAGS_DEBUG MATCHES "/Od")
	    string(REPLACE "/Od" "/O1" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
	  endif()
	  if(CMAKE_C_FLAGS_DEBUG MATCHES "/RTC1")
	    string(REPLACE "/Od" "O1" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
	  endif()
	  if(CMAKE_CXX_FLAGS_DEBUG MATCHES "/RTC1")
	    string(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
	  endif()
	  if(CMAKE_C_FLAGS_DEBUG MATCHES "/RTC1")
	    string(REPLACE "/RTC1" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
	  endif()
  endif()
  set( COMPILER_FLAGS "${COMPILER_FLAGS} ${EXTRA_COMPILER_FLAGS_MSVC} " )
  set( LINKER_FLAGS "${LINKER_FLAGS} ${EXTRA_LINKER_FLAGS_MSVC} " )
  set( ADDITIONAL_LIBRARIES ${ADDITIONAL_LIBRARIES} ${EXTRA_LIBRARIES_MSVC} )

endif()

set_target_properties( ${SAMPLE_NAME} PROPERTIES
                        COMPILE_FLAGS ${COMPILER_FLAGS}
                        LINK_FLAGS ${LINKER_FLAGS}
                     )
target_link_libraries( ${SAMPLE_NAME} ${ADDITIONAL_LIBRARIES} )

# Copy extra files to binary directory
foreach( extra_file ${EXTRA_FILES} )
    add_custom_command(
        TARGET ${SAMPLE_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/${extra_file}  ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/${extra_file}  ./
        )
endforeach( extra_file )

# Group sample based on FOLDER_GROUP defined in parent folder
if( FOLDER_GROUP )
    set_target_properties(${SAMPLE_NAME} PROPERTIES FOLDER ${FOLDER_GROUP})
endif( )
