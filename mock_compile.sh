#!/bin/bash
mkdir -p dummy_inc
echo "typedef unsigned char uint8_t;" > dummy_inc/stdint.h
echo "typedef unsigned short uint16_t;" >> dummy_inc/stdint.h
echo "typedef unsigned int uint32_t;" >> dummy_inc/stdint.h
echo "typedef unsigned long long uint64_t;" >> dummy_inc/stdint.h
echo "typedef signed char int8_t;" >> dummy_inc/stdint.h
echo "typedef signed short int16_t;" >> dummy_inc/stdint.h
echo "typedef signed int int32_t;" >> dummy_inc/stdint.h
echo "typedef signed long long int64_t;" >> dummy_inc/stdint.h
echo "#define NULL 0" > dummy_inc/stdlib.h
echo "typedef int bool;" > dummy_inc/stdbool.h
echo "#define true 1" >> dummy_inc/stdbool.h
echo "#define false 0" >> dummy_inc/stdbool.h

# Dummy up everything missing
touch dummy_inc/string.h
touch dummy_inc/stdio.h
echo "typedef void* va_list;" > dummy_inc/stdarg.h

while true; do
  OUTPUT=$(gcc -std=c89 -pedantic -Werror=declaration-after-statement -fsyntax-only \
  -I dummy_inc \
  -DPLATFORM_BEKEN=1 -DPLATFORM_BK7231N=1 -DPLATFORM_BEKEN_NEW=1 -DUINT32="unsigned int" -I src -I include \
  -include dummy_inc/stdint.h \
  -include dummy_inc/stdbool.h \
  -include dummy_inc/stdarg.h \
  -include src/hal/hal_flashVars.h \
  -include src/new_common.h \
  src/driver/drv_hlw8112.c 2>&1)

  if echo "$OUTPUT" | grep -q "fatal error: '.*' file not found"; then
    MISSING_FILE=$(echo "$OUTPUT" | grep "fatal error:" | sed -E "s/.*fatal error: '(.*)' file not found.*/\1/" | head -n 1)
    echo "Missing file: $MISSING_FILE"
    mkdir -p "dummy_inc/$(dirname "$MISSING_FILE")"
    touch "dummy_inc/$MISSING_FILE"
  else
    echo "Compilation output:"
    echo "$OUTPUT"
    break
  fi
done
touch dummy_inc/gw_intf.h
