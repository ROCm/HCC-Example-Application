#!/bin/bash
# This shell script converts an .hsaco file into an c++ file containing a "serialized"
# version of the hsaco file which is accessible as a global string.  
# The serialized version contains the all of the sections defines in the hsaco file,
# including the code and symbols.
# The resulting .cpp file can be compiled with a C compiler (gcc) and linked into
# an application, and the .hsaco file accessed with the global string.
#usage embed_hsaco.sh INFILE OUTFILE HSACO_SYMBOL

HSACO_INFILE=$1
HSACO_OUTFILE=$2
SYMBOLNAME=$(basename -s .hsaco $1)

echo "#include <stddef.h>" > $HSACO_OUTFILE
echo "char _${SYMBOLNAME}_HSA_CodeObjMem[] = {" >> $HSACO_OUTFILE
hexdump -v -e '"0x" 1/1 "%02X" ","' $HSACO_INFILE  >> $HSACO_OUTFILE
echo "};" >> $HSACO_OUTFILE
echo "size_t _${SYMBOLNAME}_HSA_CodeObjMemSz = sizeof(_${SYMBOLNAME}_HSA_CodeObjMem);" >> $HSACO_OUTFILE
