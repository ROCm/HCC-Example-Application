# Intro

This project demonstrates how to call an OpenCL kernel from HCC, using the HSA runtime.  
The intent is to allow OpenCL kenels to be combined with HCC runtime, to allow 
existing OpenCL kernels to be leveraged and also to provide more developer options.

The code also shows several optimizations that are bracked with "p_opt\*" variables and
can be enabled with command-line switches.  

See associated blog here: http://gpuopen.com/rocm-with-harmony-combining-opencl-hcc-hsa-in-a-single-program/

# Setup
This example requires a ROCM installation + the CLOC (CL Offline Compiler) tool.  
https://github.com/HSAFoundation/CLOC



# Run
// Run baseline configuration
$./BitonicSort_hcc 

// Run with all optimizatiosn enabled:
$./BitonicSort_hcc --opt

// Show help
$ ./BitonicSort_hcc --help
usage: BitonicSort [options]
--opt                Enable all optimizations.
--optPreallocSignal  Pre-allocate the signal in setup.
--optPreallocKernarg Pre-allocate the kernarg in setup.
--optAvoidHostSync   Don't synchronize to host after each kernel launch
--optFence           Don't fence/flush after each kernel submission.
--optPinnedHost      Use pinned host memory for allocations.

--printProgress      Print progress messages for each stage.  Will impact timing measurements.
--useHcArray         Use hc::array<> to allocate memory (default uses hc::am_alloc).
--loadKernelFromFile Load HSACO from file (rather than use embedded HSACO string)



