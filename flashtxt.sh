#!/bin/bash
#. ${IDF_PATH}/add_path.sh																		|Had to change this two lines to
#esptool.py --chip esp32 --port "COM4" --baud $((230400*4)) write_flash -fs 4MB 0x100000 "$1" 	|the following (otherwise i got: "esptool.py command not found"!??)
${IDF_PATH}/components/esptool_py/esptool/esptool.py --chip esp32 --port "COM7" --baud $((230400*4)) write_flash -fs 4MB 0x68000 "$1"
#. ${IDF_PATH}/add_path.sh
#esptool.py --chip esp32 --port "COM4" --baud $((230400*4)) write_flash -fs 4MB 0x100000 "$1"