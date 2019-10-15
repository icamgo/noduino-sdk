/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/

#include "quark-nb.h"

void gpio_init()
{
	// init the gpio0, ctrl the power of the sensors
	hal_gpio_init(HAL_GPIO_19);
	hal_pinmux_set_function(HAL_GPIO_19, HAL_GPIO_19_GPIO19);  //设置复用功能为GPIO

	hal_gpio_set_direction(HAL_GPIO_19, HAL_GPIO_DIRECTION_OUTPUT);	//设置引脚为输出

	//hal_gpio_pull_down(HAL_GPIO_19);
	hal_gpio_set_output(HAL_GPIO_19, 0);
}

void sensor_power_on()
{
	hal_gpio_set_output(HAL_GPIO_19, 1);
}

void sensor_power_off()
{
	hal_gpio_set_output(HAL_GPIO_19, 0);
}
