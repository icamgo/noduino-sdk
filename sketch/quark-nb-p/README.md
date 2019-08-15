Quark-NB Pressure Sensor
=========================

### Overview

* ADC0: pin38, 10bits, 0.1~1.4V


### Pin Resource

```
UART0 --- DBG_UART, 115200 8N1
UART1 --- AT UART, 115200 8N1
```

### Upload

* Connect the /dev/ttyUSB0 to UART0 (DBG_UART)
* Open the 'IoT Flash Tool' to open the flash_download.cfg
* click the 'Start' button of the 'IoT Flash Tool' then power on the NB-IoT module board


### Clean

```bash
$ make clean
```
