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
//
// Copyright (c) 2008 Advanced Micro Devices, Inc. All rights reserved.
//

#ifndef HCSAMPLE_H_
#define HCSAMPLE_H_

/******************************************************************************
* Included header files                                                       *
******************************************************************************/
#include<hc.hpp>
#include "SDKUtil.hpp"

/******************************************************************************
* namespace hc                                                       *
******************************************************************************/
using namespace hc;


namespace appsdk
{
/******************************************************************************
* HCSample                                                                   *
* class implements various resources required by the test                     *
* Publically Inherited from HCApplication                                    *
* Initialize the resources used by tests                                      *
******************************************************************************/
class HCCommandArgs: public SDKCmdArgsParser
{
    public:
        unsigned int deviceId;       /**< Cmd Line Option- if deviceId */
        bool enableDeviceId;         /**< Cmd Line Option- if enableDeviceId */
        accelerator deviceAccl;      /**< Cmd Line Option- if enableDeviceId */

    

        /**
        ***********************************************************************
        * @fn initialize
        * @brief Initialize the resources used by tests
        * @return 0 on success Positive if expected and Non-zero on failure
        **********************************************************************/
        int initialize()
        {
            int defaultOptions = 5;
            Option *optionList = new Option[defaultOptions];
            CHECK_ALLOCATION(optionList, "Error. Failed to allocate memory (optionList)\n");
            optionList[0]._sVersion = "q";
            optionList[0]._lVersion = "quiet";
            optionList[0]._description = "Quiet mode. Suppress all text output.";
            optionList[0]._type = CA_NO_ARGUMENT;
            optionList[0]._value = &quiet;
            optionList[1]._sVersion = "e";
            optionList[1]._lVersion = "verify";
            optionList[1]._description = "Verify results against reference implementation.";
            optionList[1]._type = CA_NO_ARGUMENT;
            optionList[1]._value = &verify;
            optionList[2]._sVersion = "t";
            optionList[2]._lVersion = "timing";
            optionList[2]._description = "Print timing.";
            optionList[2]._type = CA_NO_ARGUMENT;
            optionList[2]._value = &timing;
            optionList[3]._sVersion = "v";
            optionList[3]._lVersion = "version";
            optionList[3]._description = "AMD APP SDK version string.";
            optionList[3]._type = CA_NO_ARGUMENT;
            optionList[3]._value = &version;
            optionList[4]._sVersion = "d";
            optionList[4]._lVersion = "deviceId";
            optionList[4]._description =
                "Select deviceId to be used[0 to N-1 where N is number devices available].";
            optionList[4]._type = CA_ARG_INT;
            optionList[4]._value = &deviceId;
            _numArgs = defaultOptions;
            _options = optionList;
            return SDK_SUCCESS;
        }


        /**
        ***********************************************************************
        * @brief Destroy the resources used by tests
        **********************************************************************/
        virtual ~HCCommandArgs()
        {
        }

        /**
        ***********************************************************************
        * @brief Constructor, initialize the resources used by tests
        * @param sampleName Name of the Sample
        **********************************************************************/
        HCCommandArgs()
        {
            deviceId = 0;
            enableDeviceId = false;
        }

        /**
        ***********************************************************************
        * @brief parseCommandLine parses the command line options given by user
        * @param argc Number of elements in cmd line input
        * @param argv array of char* storing the CmdLine Options
        * @return 0 on success Positive if expected and Non-zero on failure
        **********************************************************************/
        int parseCommandLine(int argc, char **argv)
        {
            if(!parse(argv,argc))
            {
                usage();
                if(isArgSet("h",true) == true)
                {
                    exit(SDK_SUCCESS);
                }
                return SDK_FAILURE;
            }
            if(isArgSet("h",true) == true)
            {
                usage();
                exit(SDK_SUCCESS);
            }
            if(isArgSet("v", true)
                    || isArgSet("version", false))
            {
                std::cout << "SDK version : " << sampleVerStr.c_str()
                          << std::endl;
                exit(0);
            }
            if(isArgSet("d",true)
                    || isArgSet("deviceId",false))
            {
                enableDeviceId = true;
                if(validateDeviceOptions() != SDK_SUCCESS)
                {
                    std::cout << "validateDeviceOptions failed.\n ";
                    return SDK_FAILURE;
                }
            }
            return SDK_SUCCESS;
        }

        /**
        ***********************************************************************
        * @brief Print all available devices
        * @return 0 on success Positive if expected and Non-zero on failure
        **********************************************************************/
        int printDeviceList()
        {
            std::vector<accelerator> allAccl = accelerator::get_all();
            unsigned int numAccelerator = (unsigned int)allAccl.size();
            std::cout << "Available Accelerators:" << std::endl;
            for (unsigned i = 0; i < numAccelerator; ++i)
            {
                std::wcout<<"Accelerator " <<i;
                std::wcout<< " : " << allAccl[i].get_description() << std::endl;
            }
            std::cout << "\n";
            return SDK_SUCCESS;
        }

        /**
        ***********************************************************************
        * @brief Validates if the intended platform and device is used
        * @return 0 on success Positive if expected and Non-zero on failure
        **********************************************************************/
        int validateDeviceOptions()
        {
            std::vector<accelerator> allAccl = accelerator::get_all();
            unsigned int numAccelerator = (unsigned int)allAccl.size();
            if(deviceId >= numAccelerator)
            {
                if(deviceId - 1 == 0)
                {
                    std::cout << "deviceId should be 0" << std::endl;
                }
                else
                {
                    std::cout << "deviceId should be 0 to " << numAccelerator - 1
                              << std::endl;
                }
                usage();
                return SDK_FAILURE;
            }
            return SDK_SUCCESS;
        }


        /**
        ********************************************************************************
        * @fn setDefaultAccelerator
        * @brief Set a default accelerator
        *******************************************************************************/
        int setDefaultAccelerator()
        {
            std::vector<accelerator> allAccl = accelerator::get_all();
            /**************************************************************************
            * if deviceID is not set, the default accelerator is AMD Readon           *
            **************************************************************************/
            if(!enableDeviceId)
            {
                for (unsigned i = 0; i < allAccl.size(); ++i)
                {
                    if (allAccl[i].get_description().find(L"AMD Radeon") != std::wstring::npos ||
                            allAccl[i].get_description().find(L"ATI Radeon") != std::wstring::npos )
                    {
                        deviceAccl = allAccl[i];
                        break;
                    }
                }
            }
            else
            {
                deviceAccl = allAccl[deviceId];
            }
            accelerator::set_default(deviceAccl.get_device_path());
            std::wcout << L"Selected accelerator : " << deviceAccl.get_description()
                       << std::endl;

#if defined(_WIN32) || defined(_WIN64)
            if (deviceAccl == accelerator(accelerator::direct3d_ref))
            {
                std::cout << "WARNING!! Running on very slow emulator!" << std::endl;
            }
            if(deviceAccl == accelerator(accelerator::cpu_accelerator))
            {
                std::cout << "There is no need to run on single CPU !"<<std::endl;
                return SDK_FAILURE;
            }
#endif
            return SDK_SUCCESS;
        }

};


}
#endif
