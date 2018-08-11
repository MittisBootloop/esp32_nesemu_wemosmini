ESP32-NESEMU, a Nintendo Entertainment System emulator for the ESP32
====================================================================

This is the quick and dirty port of Nofrendo, a Nintendo Entertainment System emulator (original by Espressif, https://github.com/espressif/esp32-nesemu).
I've added (even more dirty) multi Rom support and a little Menu with sound-, fullscreen- and brightness-settings. Fullscreen can cause graphical
glitches and it still lacks sound.

Warning
-------

This is a proof-of-concept and not an official application note. As such, this code is entirely unsupported by Espressif.


Compiling
---------

You can compile this code with espressifs esp-idf (ESP32) or you can download a precompiled version here:
	https://drive.google.com/open?id=1vhh_dH3Y5HFa4yqkVhevmMNTjaeFxtNw


Display
-------

To display the NES output, please connect a 320x240 ili9341-based SPI display to the ESP32 in this way:

    =====  =======================
    Pin    GPIO
    =====  =======================
    MISO   25
    MOSI   23
    CLK    19
    CS     22
    DC     21
    RST    18
    LED    27
    =====  =======================

Also connect the power supply and ground. For now, the LCD is controlled using a SPI peripheral, fed using the 2nd CPU. This is less than ideal; feeding
the SPI controller using DMA is better, but was left out due to this being a proof of concept.


Controller
----------


    | Button | GPIO |
	| ---    | ---  |
    |	UP	 |	34	|
	|  DOWN  |	33	|
	|  RIGHT |  32  |
	|  LEFT  |  39  |
	| SELECT |  17  |
	|  START |  14  |
	|	 B	 |  35  |
	|	 A	 |  13  |
	| ON/OFF |  12  |
	|  MENU	 |  16  |

	
Connect also 3.3V to the Buttons

Sound
-----

Connect one Speaker-Pin to GPIO 26 and the other one to GND

ROM
---

This includes no Roms. You'll have to flash your own Roms and modify the roms.txt according to your needs.
Don't change the Layout from roms.txt, otherwise it could happen, that the menu will not load.
Description on how to change the roms.txt and where to place the Roms are in the file itself.

You can flash up to 14 Roms.
4x up to 100KB
2x up to 132KB
4x up to 260KB
1x up to 296KB, 388KB, 516KB, 772KB

Flash the Roms with the flashrom.sh script, you'll need to add an argument for flash adress(0x12345) and one for the 
file you want to flash: ./flashrom.sh 0xTargetAdress RomName.nes
Flash the roms.txt with the flashtxt.sh script: ./flashtxt roms.txt

If you flash, for example, only to Rom-Partition 2, you'll have to give Rom1 in roms.txt anyhow an Name (a place-maker)
otherwise the menu couldn't load the right partition.

Copyright
---------

Code in this repository is Copyright (C) 2016 Espressif Systems, licensed under the Apache License 2.0 as described in the file LICENSE. Code in the
components/nofrendo is Copyright (c) 1998-2000 Matthew Conte (matt@conte.com) and licensed under the GPLv2.