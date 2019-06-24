/*
   opencpu_gpio.c
   created by xgl,2018/4/2
*/

#include "m5311_opencpu.h"

//GPIO外部中断回调函数示例
void test_eint_handler(void *parameter)
{
	hal_eint_mask(HAL_EINT_NUMBER_7);
	opencpu_printf("\nEINT 7\n");
	hal_eint_unmask(HAL_EINT_NUMBER_7);
}	

/*
 * GPIO常规使用示例：
 * 先配置引脚复用功能，然后设置引脚方向，
 * 设置上下拉，设置输出高低，获取引脚输入等
*/
void test_gpio()
{
	unsigned char temp;
	unsigned char temp2;
	hal_gpio_init(HAL_GPIO_7);
	hal_pinmux_set_function(HAL_GPIO_7, HAL_GPIO_7_GPIO7);  //设置复用功能为GPIO
	
	hal_gpio_set_direction(HAL_GPIO_7, HAL_GPIO_DIRECTION_OUTPUT);	//设置引脚为输出
	hal_gpio_pull_up(HAL_GPIO_7);
	hal_gpio_set_output(HAL_GPIO_7,0);	//设置引脚为高
	hal_gpio_set_output(HAL_GPIO_7,1);	//设置引脚为低

	hal_gpio_set_direction(HAL_GPIO_7, HAL_GPIO_DIRECTION_INPUT);	//设置引脚为输入
	hal_gpio_get_input(HAL_GPIO_7,&temp);	//获取引脚输入的电平，temp为0则为低电平，temp为1则为高电平
}

/*
 * GPIO中断使用示例
*/
void test_gpio_irq()
{
    hal_eint_config_t eint1_config;
	hal_gpio_init(HAL_GPIO_7);
	hal_pinmux_set_function(HAL_GPIO_7,  HAL_GPIO_7_EINT7);
	hal_gpio_set_direction(HAL_GPIO_7, HAL_GPIO_DIRECTION_INPUT);
    hal_gpio_pull_up(HAL_GPIO_7);
	hal_eint_mask(HAL_EINT_NUMBER_7);
    eint1_config.trigger_mode = HAL_EINT_EDGE_FALLING;
    eint1_config.debounce_time = 10;
	hal_eint_init(HAL_EINT_NUMBER_7, &eint1_config);
	hal_eint_register_callback(HAL_EINT_NUMBER_7, test_eint_handler, NULL);
	hal_eint_unmask(HAL_EINT_NUMBER_7);
}
