/* Copyright Statement:
 *
 * (C) 2005-2017 MediaTek Inc. All rights reserved.
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. ("MediaTek") and/or its licensors.
 * Without the prior written permission of MediaTek and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 * You may only use, reproduce, modify, or distribute (as applicable) MediaTek Software
 * if you have agreed to and been bound by the applicable license agreement with
 * MediaTek ("License Agreement") and been granted explicit permission to do so within
 * the License Agreement ("Permitted User"). If you are not a Permitted User,
 * please cease any access or use of MediaTek Software immediately.
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT MEDIATEK SOFTWARE RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES
 * ARE PROVIDED TO RECEIVER ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 */

#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__
#include "hal_platform.h"
#include "hal_pinmux_define.h"






#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup hal_gpio_enum Enum
  * @{
  */

/** @brief This enum defines the GPIO direction. */
typedef enum {
    HAL_GPIO_DIRECTION_INPUT  = 0,              /**<  GPIO input direction. */
    HAL_GPIO_DIRECTION_OUTPUT = 1               /**<  GPIO output direction. */
} hal_gpio_direction_t;


/** @brief This enum defines the data type of GPIO. */
typedef enum {
    HAL_GPIO_DATA_LOW  = 0,                     /**<  GPIO data low. */
    HAL_GPIO_DATA_HIGH = 1                      /**<  GPIO data high. */
} hal_gpio_data_t;


/** @brief This enum defines the return type of GPIO API. */
typedef enum {
    HAL_GPIO_STATUS_ERROR             = -3,     /**< The GPIO function failed to execute.*/
    HAL_GPIO_STATUS_ERROR_PIN         = -2,     /**< Invalid input pin number. */
    HAL_GPIO_STATUS_INVALID_PARAMETER = -1,     /**< Invalid input parameter. */
    HAL_GPIO_STATUS_OK                = 0       /**< The GPIO function executed successfully. */
} hal_gpio_status_t;


/** @brief This enum defines the return type of pinmux API. */
typedef enum {
    HAL_PINMUX_STATUS_ERROR             = -3,   /**< The pinmux function failed to execute.*/
    HAL_PINMUX_STATUS_ERROR_PORT        = -2,   /**< Invalid input pin port. */
    HAL_PINMUX_STATUS_INVALID_FUNCTION  = -1,   /**< Invalid input function. */
    HAL_PINMUX_STATUS_OK                = 0     /**< The pinmux function executed successfully. */
} hal_pinmux_status_t;


/** @brief This enum defines driving current. */
typedef enum {
    HAL_GPIO_DRIVING_CURRENT_4MA    = 0,        /**< Defines GPIO driving current as 4mA.  */
    HAL_GPIO_DRIVING_CURRENT_8MA    = 1,        /**< Defines GPIO driving current as 8mA.  */
    HAL_GPIO_DRIVING_CURRENT_12MA   = 2,        /**< Defines GPIO driving current as 12mA. */
    HAL_GPIO_DRIVING_CURRENT_16MA   = 3         /**< Defines GPIO driving current as 16mA. */
} hal_gpio_driving_current_t;


/**
  * @}
  */



/**
 * @brief     This function initializes the GPIO hardware with basic functionality. The target pin must be initialized before use.
 * @param[in] gpio_pin specifies the pin number to initialize.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_init(hal_gpio_pin_t gpio_pin);


/**
 * @brief     This function deinitializes the GPIO hardware to its default status. The target pin must be deinitialized if not used.
 * @param[in] gpio_pin specifies pin number to deinitialize.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_deinit(hal_gpio_pin_t gpio_pin);

/**
 * @brief     This function configures the pinmux of target GPIO.
 *            Pin Multiplexer (pinmux) connects the pin and the onboard peripherals,
 *            so the pin will operate in a specific mode once the pin is programmed to a peripheral's function.
 *            The alternate functions of every pin are provided in hal_pinmux_define.h.
 * @param[in] gpio_pin specifies the pin number to configure.
 * @param[in] function_index specifies the function for the pin.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PINMUX_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PINMUX_STATUS_INVALID_FUNCTION, a wrong alternate function is given, the parameter must be verified.
 *            If the return value is #HAL_PINMUX_STATUS_ERROR_PORT, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_PINMUX_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */

hal_pinmux_status_t hal_pinmux_set_function(hal_gpio_pin_t gpio_pin, uint8_t function_index);

/**
 * @brief     This function gets the input data of target GPIO when the direction of the GPIO is input.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[in] gpio_data is the input data received from the target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_get_input(hal_gpio_pin_t gpio_pin, hal_gpio_data_t *gpio_data);

/**
 * @brief     This function sets the output data of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[in] gpio_data is the output data of the target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_output(hal_gpio_pin_t gpio_pin, hal_gpio_data_t gpio_data);


/**
 * @brief     This function gets the output data of the target GPIO when the direction of the GPIO is output.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[in] gpio_data is output data of the target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_get_output(hal_gpio_pin_t gpio_pin, hal_gpio_data_t *gpio_data);


/**
 * @brief     This function sets the direction of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to set.
 * @param[in] gpio_direction is the direction of the target GPIO, the direction can be input or output.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_direction(hal_gpio_pin_t gpio_pin, hal_gpio_direction_t gpio_direction);


/**
 * @brief     This function gets the direction of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[in] gpio_direction is the direction of target GPIO, the direction can be input or output.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_get_direction(hal_gpio_pin_t gpio_pin, hal_gpio_direction_t *gpio_direction);



/**
 * @brief     This function sets the target GPIO to high impedance state.
 *            High impedance can prevent the target GPIO from electric leakage.
 *            The pin in high impedance state acts as an open circuit, although it is connected to a low impedance circuit, it will not be affected.
 *            It is recommended to put the pin into high impedance state, if the pin is not in use to optimize the power consumption.
 * @param[in] gpio_pin specifies the pin number to set.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_high_impedance(hal_gpio_pin_t gpio_pin);

/**
 * @brief     This function removes the high impedance state for the target GPIO.
 *            High impedance can prevent the target GPIO from electric leakage.
 *            It is necessary to call this function before further configuration if the pin is in high impedance state.
 * @param[in] gpio_pin specifies the pin number to set.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 * @par       Example
 * @code
 *
 *            hal_gpio_status_t ret;
 *
 *            ret = hal_gpio_init(gpio_pin);
 *            ret = hal_gpio_set_high_impedance(gpio_pin);
 *            // Do something else
 *            ret = hal_gpio_clear_high_impedance(gpio_pin); // Put the target GPIO out of high impedance state before other configuration.
 *            ret = hal_gpio_deinit(gpio_pin);
 *
 * @endcode
 */
hal_gpio_status_t hal_gpio_clear_high_impedance(hal_gpio_pin_t gpio_pin);



/**
 * @brief     This function toggles the output data of the target GPIO when the direction of the pin is output. After this function,
 *            the output data of the target GPIO will be inversed.
 * @param[in] gpio_pin specifies the pin number to toggle.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 * @par       Example
 * @code
 *            hal_gpio_status_t ret;
 *            hal_pinmux_status_t ret_pinmux_status;
 *
 *            ret = hal_gpio_init(gpio_pin);
 *            ret_pinmux_status = hal_pinmux_set_function(gpio_pin, function_index); // Set the pin to GPIO mode.
 *            ret = hal_gpio_set_direction(gpio_pin, HAL_GPIO_DIRECTION_OUTPUT);
 *            ret = hal_gpio_set_output(gpio_pin, HAL_GPIO_DATA_HIGH);
 *            ret = hal_gpio_toggle_pin(gpio_pin);  // Output data of gpio_pin will be toggled to low from high.
 *            ret = hal_gpio_deinit(gpio_pin);
 * @endcode
 */
hal_gpio_status_t hal_gpio_toggle_pin(hal_gpio_pin_t gpio_pin);



/**
 * @brief     This function enables the input data inversion of the target GPIO, after this function,
 *            the input data of the target GPIO will always be inversed until the inverse function is disabled.
 * @param[in] gpio_pin specifies the pin number to inverse.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 * @par       Example
 * @code
 *            hal_gpio_status_t ret;
 *
 *            ret = hal_gpio_init(gpio_pin);
 *            ret = hal_gpio_enable_inversion(gpio_pin);
 *            // Do something else
 *            ret = hal_gpio_disable_inversion(gpio_pin);
 *            ret = hal_gpio_deinit(gpio_pin);
 * @endcode
 */
hal_gpio_status_t hal_gpio_enable_inversion(hal_gpio_pin_t gpio_pin);


/**
 * @brief     This function disables the input data inversion of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to configure.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 * @par       Example
 * @code
 *            hal_gpio_status_t ret;
 *
 *            ret = hal_gpio_init(gpio_pin);
 *            ret = hal_gpio_enable_inversion(gpio_pin);
 *            // Do something else
 *            ret = hal_gpio_disable_inversion(gpio_pin);
 *            ret = hal_gpio_deinit(gpio_pin);
 * @endcode
 */
hal_gpio_status_t hal_gpio_disable_inversion(hal_gpio_pin_t gpio_pin);



/**
 * @brief     This function sets the target GPIO to pull-up state, after this function,
 *            the input data of the target pin will be equivalent to high if the pin is disconnected.
 *            This function operates on the pins with only one pull-up resistor.
 * @param[in] gpio_pin specifies the pin number to set.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 * @par       Example
 * @code
 *            hal_gpio_status_t ret;
 *            hal_pinmux_status_t ret_pinmux_status;
 *
 *            ret = hal_gpio_init(gpio_pin);
 *            ret_pinmux_status = hal_pinmux_set_function(gpio_pin, function_index); // Set the pin to GPIO mode.
 *            ret = hal_gpio_set_direction(gpio_pin, HAL_GPIO_DIRECTION_INPUT);
 *            ret = hal_gpio_pull_up(gpio_pin);   // Pull state of the target GPIO is set to pull-up.
 *            ret = hal_gpio_deinit(gpio_pin);
 * @endcode
 */
hal_gpio_status_t hal_gpio_pull_up(hal_gpio_pin_t gpio_pin);


/**
 * @brief     This function sets the target GPIO to the pull-down state, after this function,
 *            the input data of the target pin will be equivalent to low if the pin is disconnected.
 *            This function operates on the pin with one pull-down resistor.
 * @param[in] gpio_pin specifies the pin number to set.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 * @par       Example
 * @code
 *            hal_gpio_status_t ret;
 *            hal_pinmux_status_t ret_pinmux_status;
 *
 *            ret = hal_gpio_init(gpio_pin);
 *            ret_pinmux_status = hal_pinmux_set_function(gpio_pin, function_index); // Set the pin to GPIO mode.
 *            ret = hal_gpio_set_direction(gpio_pin, HAL_GPIO_DIRECTION_INPUT);
 *            ret = hal_gpio_pull_down(gpio_pin);   // Pull state of the target GPIO is set to pull-down.
 *            ret = hal_gpio_deinit(gpio_pin);
 * @endcode
 */
hal_gpio_status_t hal_gpio_pull_down(hal_gpio_pin_t gpio_pin);


/**
 * @brief     This function disables pull-up or pull-down of the target GPIO.
 *            This function operates on the pins with one pull-up and one pull-down resistors.
 * @param[in] gpio_pin specifies the pin number to set.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 * @par       Example
 * @code
 *            hal_gpio_status_t ret;
 *            hal_pinmux_status_t ret_pinmux_status;
 *
 *            ret = hal_gpio_init(gpio_pin);
 *            ret_pinmux_status = hal_pinmux_set_function(gpio_pin, function_index); // Set the pin to GPIO mode.
 *            ret = hal_gpio_set_direction(gpio_pin, HAL_GPIO_DIRECTION_INPUT);
 *            ret = hal_gpio_pull_down(gpio_pin);
 *            ret = hal_gpio_disable_pull(gpio_pin);   // Pull state of the target GPIO is disabled.
 *            ret = hal_gpio_deinit(gpio_pin);
 * @endcode
 */
hal_gpio_status_t hal_gpio_disable_pull(hal_gpio_pin_t gpio_pin);



/**
 * @brief     This function sets the pull up/down state of the GPIO that has more than one pull-up or pull-down resistor.
 * @param[in] gpio_pin specifies the pin number to configure.
 * @param[in] gpio_pupd specifies the pull-up or pull-down of the target GPIO.
 * @param[in] gpio_r0 works with gpio_r1 to specify the pull-up and pull-down resistor of the target GPIO.
 * @param[in] gpio_r1 works with gpio_r0 to specify the pull-up and pull-down resistor of the target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 * @par       Example
 * @code
 *            hal_gpio_status_t ret;
 *            hal_pinmux_status_t ret_pinmux_status;
 *
 *            ret = hal_gpio_init(gpio_pin);
 *            ret_pinmux_status = hal_pinmux_set_function(gpio_pin, function_index); // Set the pin to GPIO mode.
 *            ret = hal_gpio_set_direction(gpio_pin, HAL_GPIO_DIRECTION_INPUT);
 *            ret = hal_gpio_set_pupd_register(gpio_pin,gpio_pupd,gpio_r0,gpio_r1); // Pull state of the target GPIO is set to a state determined by the combination of gpio_pupd,gpio_r0 and gpio_r1.
 *            ret = hal_gpio_deinit(gpio_pin);
 * @endcode
 */
hal_gpio_status_t hal_gpio_set_pupd_register(hal_gpio_pin_t gpio_pin, uint8_t gpio_pupd, uint8_t gpio_r0, uint8_t gpio_r1);




#ifdef __cplusplus
}
#endif


/**
* @}
* @}
*/

#endif /* __HAL_GPIO_H__ */

