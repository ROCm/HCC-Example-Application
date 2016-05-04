
## Getting Started ##

First, follow the instruction to install [ROCm](https://github.com/RadeonOpenCompute/ROCm), which provides the HCC compiler, the ROCr runtime and the toolchains to compile the samples.  

### How to Build the Samples ###

#### HC C++ samples ####

1. Make sure the HCC compiler is in your path.  The default installation of HCC is /opt/rocm/bin.
2. In the HCC-Example-Applications directory, create a build directory.
3. Go into the build directory, then type `CXX=hcc cmake ..` to generate the makefiles.
4. Type `make` to compile the samples.
