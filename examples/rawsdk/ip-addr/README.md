Noduino SDK - Ip
===========================================

This simple project shows how to output "Hello World" to the UART0

It shows how to use the uart0 in Noduino SDK.


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
$ make flash
```

It use the ```/dev/ttyUSB0``` serial device to upload the firmware into board.

You need to modify the varible ```ESPPORT``` according to your system in
Makefile. It should be /dev/cu.SLAB_USBtoUART in Mac OS X or COM3 in windows.


### Check

Using the cutecom to check the Serial port message in Linux:

```bash
$ sudo apt-get install cutecom
```

Run the cutecom, set:

 * Device: /dev/ttyUSB0
 * Baud rate: 115200
 * Data bits: 8
 * Stop bits: 1
 * Parity: None

Then click the 'Open', and it will get a "Hello World!" every 3 seconds.


### Clean

```bash
$ make clean
```
