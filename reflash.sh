#!/bin/bash

set -e

dfu-util -D ./bin/SCUT_candleLightFD_fw.bin -a 0 -s 0x08000000:leave
