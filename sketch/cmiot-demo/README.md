OpenCPU demo
=====================

### Overview

```
m5311_opencpu.c		Main function
onenet_test.c		Testing OneNet function
opencpu_base_func_demo.c	Testing CMIoT module
opencpu_fota_demo.c
opencpu_gpio_demo.c
opencpu_iic_demo.c		Testing I2C BMP180
opencpu_network_demo.c	Testing TCP/IP
opencpu_other_dev_demo.c
opencpu_spi_demo.c
opencpu_uart_demo.c
```


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
