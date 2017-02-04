5-way relay controller Reference Firmware
===========================================

### Overview

* [Quick Start in Chinese](http://wiki.jackslab.org/IBaihe_QuickStart)


![iBaihe](doc/ibaihe-v0.8-top.jpg)


![iBaihe](doc/ibaihe-v0.8-bottom.jpg)


![iBaihe](doc/ibaihe-v0.8-pcb.jpg)


### Pin Resource

1. GPIO13(D4) control the 1st relay
2. GPIO12(D5) control the 2nd relay 
3. GPIO14(D6) control the 3rd relay
4. GPIO16(D7) control the 4th relay
5. GPIO4 (D8) control the 5th relay
6. GPIO5 (D9) control the 6th relay

GPIO0 is related to the key, pull up, pressed is low


### Build

```bash
$ make
```

Or make output more message:

```bash
$ make V=1
```

The target file is at build/ directory.


### Upload

```bash
$ make produce ESPPORT=/dev/ttyUSB0 
```

It use the ```/dev/ttyUSB0``` serial device to upload the firmware into board.

You need to modify the varible ```ESPPORT``` according to your system in
Makefile. It should be /dev/cu.SLAB_USBtoUART in Mac OS X or COM3 in windows.


### Erase Flash

Enter flush mode and execute:

```bash
$ make erase
```

### Dump Flash

Enter flush mode and execute:

```bash
$ make dump
```

The flash.bin is the whole flash content


### Re-Init RF

Enter flush mode and execute:

```bash
$ make rfinit
```

It's override the rfinit data in flash using the esp_init_data_default.bin
in sdk1.4


### Show the memory usage

After building the firmware:

```bash
$ make mem
   Section|                   Description| Start (hex)|   End (hex)|Used space
------------------------------------------------------------------------------
      data|        Initialized Data (RAM)|    3FFE8000|    3FFE83B0|     944
    rodata|           ReadOnly Data (RAM)|    3FFE83B0|    3FFE9B54|    6052
       bss|      Uninitialized Data (RAM)|    3FFE9B60|    3FFF0B20|   28608
      lit4|     Uninitialized Data (IRAM)|    40107408|    40107408|       0
      text|          Uncached Code (IRAM)|    40100000|    40107408|   29704
irom0_text|             Cached Code (SPI)|    40240000|    40289AC8|  301768
------------------------------------------------------------------------------
Entry Point : 40100004 call_user_start()
Total Used RAM : 65308
Free IRam : 3064 or 19448 (option 48k IRAM)
Free Heap : 46316
Total Free RAM : 65764
```


### Clean the system parameter

If you want to clean the ssid and password stored in flash by SDK:

```bash
$ make sysclean
```


### Clean

```bash
$ make clean
```
