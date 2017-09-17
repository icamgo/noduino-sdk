#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_interface.h"

#include "driver/uart.h"
#include "softuart.h"

#define user_procTaskPrio        0
#define user_procTaskQueueLen    1
os_event_t    user_procTaskQueue[user_procTaskQueueLen];

//create global softuart instances
Softuart softuart;

// loop function will be execute by "os" periodically
irom static void loop(os_event_t *events)
{
	uint8_t ic = 0;

	if(Softuart_Available(&softuart)) {
		ic = Softuart_Read(&softuart);
		os_printf("rs485 rx: %c (0x%02X)\r\n", ic, ic);
	}

	if(ic == 'r') {
		os_printf("rx read cmd, send rs485 echo...\r\n");
		Softuart_Puts(&softuart,"rx read cmd\n");
	}

	//some delay until we run this task again
    os_delay_us(100000);

	// run (schedule) this loop task again 
    system_os_post(user_procTaskPrio, 0, 0 );
}

irom void user_init()
{
	// Initialize UART0 to use as debug
    //uart_div_modify(0, UART_CLK_FREQ / 9600);
	uart_init(BIT_RATE_115200, BIT_RATE_115200);

	wifi_set_opmode(NULL_MODE);

	//init software uart
	Softuart_SetPinRx(&softuart,14);	
	Softuart_SetPinTx(&softuart,12);

	//startup
	Softuart_Init(&softuart,9600);

	//set pin 13 as output to control tx enable/disable of rs485 driver
	Softuart_EnableRs485(&softuart, 13);

	os_printf("RS485 slave init OK\r\n");

    //Start our loop task
    system_os_task(loop, user_procTaskPrio,user_procTaskQueue, user_procTaskQueueLen);
    system_os_post(user_procTaskPrio, 0, 0 );
}
