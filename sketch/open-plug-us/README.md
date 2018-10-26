Open Plug (US version) Reference Firmware
===========================================

### Overview

![Open Plug](doc/open-plug-us-1.jpg)


![Open Plug](doc/open-plug-us-2.jpg)


UART baud rate: 74880

```bash
 ets Jan  8 2013,rst cause:1, boot mode:(3,7)

load 0x40100000, len 816, room 16 
tail 0
chksum 0x8d
load 0x3ffe8000, len 788, room 8 
tail 12
chksum 0xcf
ho 0 tail 12 room 4
load 0x3ffe8314, len 288, room 12 
tail 4
chksum 0xcf
csum 0xcf

2nd boot version : 1.2
  SPI Speed      : 40MHz
  SPI Mode       : QIO
  SPI Flash Size : 8Mbit

jump to run user1

SDK ver: 1.0.1 compiled @ Apr 24 2015 19:31:11
phy ver: 329, pp ver: 8.4

ver��8266_socket_CDT_1.3_2016.1.19

ver:2 1 2 5 0
read user params 0
after read status 1
read user params 0
load user param ok
after init time 1
read user params 0
load user param ok

after init handcontrol 1
config len is:1
os time set becon
mode : sta(5c:cf:7f:14:11:d8)
add if0
scandone
no UPGADE-AP found, reconnect after 1s
reconnect
scandone
no UPGADE-AP found, reconnect after 1s
reconnect
scandone
......
......
```


### Pin Resource

Open Plug XiangShang used resource (OpenPlug hw_ver: 2.2):

1. GPIO12 control the LED (pull down to turn on)
2. GPIO13 related to the key (pull up, pressed is low)
3. GPIO15 control the relay (pull up to turn on)


Open Plug YiMu used resource (OpenPlug hw_ver: 2.4):

1. GPIO12/13/15 control the LED (pull down to turn on)
2. GPIO1 related to the key (pull up, pressed is low)
3. GPIO14 control the relay (pull up to turn on)


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

Using a FT232RL USB to Serial Board, and connect:

* FT232_RXD -----> Plug_TX
* FT232_TXD -----> Plug_RX
* FT232_GND -----> Plug_GND
* FT232_GND ------> Plug_GPIO0

Then connect the 3.3V to power up the board:

* FT232_VCC3.3 ---> Plug_3.3V (Power supply must be 3.3V!)


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
