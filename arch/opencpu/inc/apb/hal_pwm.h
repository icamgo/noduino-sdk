
#ifndef __HAL_PWM_H__
#define __HAL_PWM_H__
#include "hal_platform.h"







#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup hal_pwm_enum Enum
  * @{
  */

/** @brief	This enum defines the API return type.  */
typedef enum {
    HAL_PWM_STATUS_ERROR                = -4,         /**< An error occurred during the function call. */
    HAL_PWM_STATUS_ERROR_CHANNEL        = -3,         /**< A wrong PWM channel is given. */
    HAL_PWM_STATUS_INVALID_PARAMETER    = -2,         /**< A wrong parameter is given. */
    HAL_PWM_STATUS_INVALID_FREQUENCY    = -1,         /**< A smaller frequency is given.*/
    HAL_PWM_STATUS_OK                   =  0          /**< No error during the function call. */
} hal_pwm_status_t;



/** @brief   This enum defines the PWM running status. */
typedef enum {
    HAL_PWM_IDLE = 0,                         /**<The PWM status is idle. */
    HAL_PWM_BUSY = 1                          /**< The PWM status is busy. */
} hal_pwm_running_status_t;

/**
  * @}
  */



/** @defgroup hal_pwm_enum Enum
  * @{
  */

/** @brief  This enum defines PWM clock division advanced configuration */
typedef enum {
    HAL_PWM_CLOCK_DIVISION_2 = 1,            /**< Specify the PWM source clock 2 division. */
    HAL_PWM_CLOCK_DIVISION_4 = 2,            /**< Specify the PWM  source clock 4 division. */
    HAL_PWM_CLOCK_DIVISION_8 = 3             /**< Specify the PWM  source clock 8 division. */
} hal_pwm_advanced_config_t;

/**
  * @}
  */


hal_pwm_status_t hal_pwm_init(hal_pwm_channel_t pwm_channel, hal_pwm_source_clock_t source_clock);


/**
 * @brief   This function deinitializes the PWM hardware.
 * @param[in]  pwm_channel is the PWM channel number. For more details about the parameter, please refer to #hal_pwm_channel_t.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PWM_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PWM_STATUS_INVALID_PARAMETER, a wrong parameter is given. The parameter needs to be verified.
 * @par       Example
 * For a sample code, please refer to @ref HAL_PWM_Driver_Usage_Chapter.
 * @sa  hal_pwm_init()
 */
hal_pwm_status_t hal_pwm_deinit(hal_pwm_channel_t pwm_channel);



/**
 * @brief This function sets the PWM frequency and retrieves total count of the PWM hardware at the specified frequency.
 * @param[in]  pwm_channel is the PWM channel number. For more details about the parameter, please refer to #hal_pwm_channel_t.
 * @param[in]  frequency is the PWM output frequency.
 * @param[out]   total_count is PWM hardware total count, the value of this parameter varies based on the given PWM frequency.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PWM_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PWM_STATUS_INVALID_PARAMETER, a wrong parameter is given. The parameter needs to be verified.
 * @par       Example
 * For a sample code, please refer to @ref HAL_PWM_Driver_Usage_Chapter.
 */
hal_pwm_status_t hal_pwm_set_frequency(hal_pwm_channel_t pwm_channel, uint32_t frequency, uint32_t *total_count);


/**
 * @brief This function sets the PWM  duty cycle.
 * @param[in]  pwm_channel is the PWM channel number. For more details about the parameter, please refer to #hal_pwm_channel_t.
 * @param[in]  duty_cycle is the PWM hardware duty cycle, which is calculated as a product of application's duty ratio and hardware 's total count.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PWM_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PWM_STATUS_INVALID_PARAMETER, a wrong parameter is given. The parameter needs to be verified.
 * @par       Example
 * For a sample code, please refer to @ref HAL_PWM_Driver_Usage_Chapter.
 * @sa  hal_pwm_get_duty_cycle()
 */

hal_pwm_status_t hal_pwm_set_duty_cycle(hal_pwm_channel_t pwm_channel, uint32_t duty_cycle);



/**
 * @brief This function starts the PWM execution.
 * @param[in]  pwm_channel is the PWM channel number. For more details about the parameter, please refer to #hal_pwm_channel_t.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PWM_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PWM_STATUS_INVALID_PARAMETER, a wrong parameter is given. The parameter needs to be verified.
 * @par       Example
 * For a sample code, please refer to @ref HAL_PWM_Driver_Usage_Chapter.
 * @sa  hal_pwm_stop()
 */

hal_pwm_status_t hal_pwm_start(hal_pwm_channel_t pwm_channel);

/**
 * @brief  This function stops the PWM execution.
 * @param[in]  pwm_channel is PWM channel number. For more details about the parameter, please refer to #hal_pwm_channel_t.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PWM_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PWM_STATUS_INVALID_PARAMETER, a wrong parameter is given. The parameter needs to be verified.
 * @par       Example
 * For a sample code, please refer to @ref HAL_PWM_Driver_Usage_Chapter.
 * @sa  hal_pwm_start()
 */

hal_pwm_status_t hal_pwm_stop(hal_pwm_channel_t pwm_channel);


/**
 * @brief This function gets current frequency of the PWM, the unit of frequency is Hz.
 * @param[in]  pwm_channel is the PWM channel number. For more details about the parameter, please refer to #hal_pwm_channel_t.
 * @param[out]  frequency  is PWM output frequency.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PWM_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PWM_STATUS_INVALID_PARAMETER, a wrong parameter is given. The parameter needs to be verified.
 * @par       Example
 * For a sample code, please refer to @ref HAL_PWM_Driver_Usage_Chapter.
 * @sa  hal_pwm_set_frequency()
 */

hal_pwm_status_t hal_pwm_get_frequency(hal_pwm_channel_t pwm_channel, uint32_t *frequency);



/**
 * @brief  This function gets the current duty cycle of the PWM.
 * @param[in]  pwm_channel is the PWM channel number. For more details about the parameter, please refer to #hal_pwm_channel_t.
 * @param[out]   *duty_cycle is PWM hardware duty cycle, which is calculated as a product of application's duty ratio and hardware 's total count.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PWM_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PWM_STATUS_INVALID_PARAMETER, a wrong parameter is given. The parameter needs to be verified.
 * @par       Example
 * For a sample code, please refer to @ref HAL_PWM_Driver_Usage_Chapter.
 * @sa  hal_pwm_set_duty_cycle()
 */

hal_pwm_status_t hal_pwm_get_duty_cycle(hal_pwm_channel_t pwm_channel, uint32_t *duty_cycle);


/**
 * @brief  This function gets the current status of PWM.
 * @param[in]  pwm_channel is the PWM channel number. For more details about the parameter, please refer to #hal_pwm_channel_t.
 * @param[out]  running_status is PWM busy or idle status, For details about this parameter, please refer to #hal_pwm_running_status_t .
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PWM_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PWM_STATUS_INVALID_PARAMETER, a wrong parameter is given. The parameter needs to be verified.
 * @par       Example
 * For a sample code, please refer to @ref HAL_PWM_Driver_Usage_Chapter.
 */

hal_pwm_status_t hal_pwm_get_running_status(hal_pwm_channel_t pwm_channel, hal_pwm_running_status_t *running_status);



#ifdef __cplusplus
}
#endif



#endif /* __HAL_PWM_H__ */

