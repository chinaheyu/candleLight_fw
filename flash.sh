#!/bin/bash

set -e

# flash firmware with pyocd
# pyocd flash -t stm32g431cbux -f 500000 ./bin/SCUT_candleLightFD_fw.bin

# flash firmware with dfu-util
dfu-util --dfuse-address -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D ./bin/SCUT_candleLightFD_fw.bin
