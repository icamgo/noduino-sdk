/*
   opencpu_uart.c
   created by xgl,2019/1/22
*/

#include "m5311_opencpu.h"

void spi_test()
{
	unsigned char spi_rx_buf[2]= {0};
	unsigned char spi_tx_buf[2]= {0};
	unsigned char spi_tx = 5;

	hal_spi_master_config_t l_config;   
	hal_spi_master_send_and_receive_config_t spi_send_and_receive_config;
	//配置SPI参数
	l_config.bit_order = HAL_SPI_MASTER_MSB_FIRST;
	l_config.slave_port = HAL_SPI_MASTER_SLAVE_0;
	l_config.clock_frequency = 1000000;
	l_config.phase = HAL_SPI_MASTER_CLOCK_PHASE1;
	l_config.polarity = HAL_SPI_MASTER_CLOCK_POLARITY0;
	//初始化SPI
	opencpu_spi_init(&l_config);
	//spi发送一个字节
	opencpu_spi_write(&spi_tx,1);
	spi_tx_buf[0]= 0xEE;
	spi_tx_buf[1] = 0x25;
	//发送一个字节，读取两个字节，使用这个函数，读取长度必须大于发送长度，否则会直接报错
	opencpu_spi_rw(spi_tx_buf,1,spi_rx_buf,2);

	//使用完后，调用释放函数：
	opencpu_spi_deinit();

}
