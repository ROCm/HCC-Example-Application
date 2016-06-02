ROCM_PATH?=/opt/rocm

OPT=-O3
CXXFLAGS=-Wall 

HSACO= BitonicSort_Kernels.hsaco
EMBEDDED_HSACO= BitonicSort_Kernels.hsaco.o

TARGET0 = BitonicSort_hcc 
SOURCES0 = BitonicSort_hcc.cpp hsa_utils.cpp
OBJECTS0=${SOURCES0:.cpp=.o}



$(TARGET0): $(OBJECTS0) $(HSACO) $(EMBEDDED_HSACO)
	hcc `hcc-config --ldflags` -lhc_am $(OPT) -L${ROCM_PATH}/lib -lhsa-runtime64  $(OBJECTS0) $(EMBEDDED_HSACO) -o $@  

clean: 
	rm -rf *.o  $(HSACO) $(HSACO).cpp
	rm -rf $(TARGET0)
	rm -rf $(TARGET1)

.cpp.o:
	hcc  `hcc-config --cxxflags` -I$(ROCM_PATH)/include -c  $(CXXFLAGS) $(OPT) $< -o $@


%.hsaco : %.cl
	$(ROCM_PATH)/cloc/bin/cloc.sh $< -o $@

%.hsaco.o : %.hsaco
	./embed_hsaco.sh $< $<.cpp BitonicSort
	gcc -c $<.cpp -o $@

	

	

BitonicSort_hcc.o: BitonicSort_hcc.cpp BitonicSort_hcc.hpp

