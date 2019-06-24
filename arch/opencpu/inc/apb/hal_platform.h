/* Copyright Statement:
 *
 * (C) 2005-2016  MediaTek Inc. All rights reserved.
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
 * the License Agreement ("Permitted User").  If you are not a Permitted User,
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

#ifndef __HAL_PLATFORM_H__
#define __HAL_PLATFORM_H__




#ifdef __cplusplus
extern "C" {
#endif

#define bool    unsigned char
#define true    1
#define false   0

/**
 * @addtogroup HAL
 * @{
 * @addtogroup SLEEP_MANAGER
 * @{
 * @addtogroup hal_sleep_manager_enum Enum
 * @{
 */
/*****************************************************************************
 * Enum
 *****************************************************************************/
/** @brief Sleep modes */
typedef enum {
    HAL_SLEEP_MODE_NONE = 0,        /**< No sleep. */
    HAL_SLEEP_MODE_IDLE = 0x1,            /**< Idle state. */
    HAL_SLEEP_MODE_LIGHT_SLEEP = 0x2,     /**< Light sleep state. */
    HAL_SLEEP_MODE_DEEP_SLEEP = 0x4,      /**< Deep Sleep state. */
    HAL_SLEEP_MODE_DEEPER_SLEEP = 0x8,    /**< Retention state. */
    HAL_SLEEP_MODE_NUMBER = 0x10,           /**< To support range detection. */
} hal_sleep_mode_t;
/** @brief sleep_manager wake up source */
typedef enum {
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_GPT = 0,        /**< General purpose timer. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_EINT,           /**< External interrupt. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_SDIO_SLV,       /**< SDIO SLV. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_SPI_SLAVE_A,    /**< SPI SLAVE A. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_WDT,            /**< WDT(RGU). */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_KP,             /**< KP. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_MD,             /**< MD. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_OST,            /**< OST. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_MD_DEBUGSYS,    /**< MD_DEBUGSYS. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_DEBUGSYS,       /**< DEBUGSYS. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_SENSOR_CTRLER,  /**< SENSOR_CTRLER. */
    HAL_SLEEP_MANAGER_WAKEUP_SOURCE_ALL             /**< All wakeup source. */
} hal_sleep_manager_wakeup_source_t;
/** @brief sleep lock levels */
typedef enum {
    HAL_SLEEP_LOCK_ALL = 0,
    HAL_SLEEP_LOCK_DEEP,
    HAL_SLEEP_LOCK_NUMBER
} hal_sleep_lock_t;
/**
 * @}
 * @}
 * @}
 */


#define HAL_UART_MODULE_ENABLED
#ifdef HAL_UART_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup UART
 * @{
 * @addtogroup hal_uart_enum Enum
 * @{
 */
/*****************************************************************************
* UART
*****************************************************************************/
/** @brief UART port index
 * There are total of four UART ports. And only UART0 and UART1 support hardware flow control.
 * | UART port | Hardware Flow Control |
 * |-----------|-----------------------|
 * |  UART0    |           V           |
 * |  UART1    |           V           |
 * |  UART2    |           X           |
 * |  UART3    |           X           |
 */
typedef enum {
    HAL_UART_0 = 0,                            /**< UART port 0. */
    HAL_UART_1 = 1,                            /**< UART port 1. */
    HAL_UART_2 = 2,                            /**< UART port 2. */
    HAL_UART_3 = 3,                            /**< UART port 3. */
    HAL_UART_MAX                               /**< The total number of UART ports (invalid UART port number). */
} hal_uart_port_t;

/**
  * @}
  */

/**@addtogroup hal_uart_define Define
 * @{
  */

/** @brief  The maximum timeout value for UART timeout interrupt, unit is ms.
  */
#define HAL_UART_TIMEOUT_VALUE_MAX  (2500)

/**
  * @}
  */

/**
 * @}
 * @}
 */
#endif




/**
 * @addtogroup HAL
 * @{
 * @addtogroup I2C_MASTER
 * @{
 * @section HAL_I2C_Transaction_Pattern_Chapter API transaction length and packets
 *  The Transaction packet is a packet sent by the I2C master using SCL and SDA.
 *  Different APIs have different transaction packets, as shown below.
 * - \b Transaction \b length \b supported \b by \b the \b APIs \n
 *  The total transaction length is determined by 4 parameters:\n
 *  send_packet_length(Ns), which indicates the number of sent packet.\n
 *  send_bytes_in_one_packet(Ms).\n
 *  receive_packet_length(Nr).\n
 *  receive_bytes_in_one_packet(Mr).\n
 *  Next, the relationship between transaction packet and these 4 parameters is introduced.
 *  - Total transaction length = Ns * Ms + Nr * Mr.
 *   - Ns is the packet length to be sent by the I2C master.
 *   - Ms is the total number of bytes in a sent packet.
 *   - Nr is the packet length to be received by the I2C master.
 *   - Mr is the total number of bytes in a received packet.
 *  - NA means the related parameter should be ignored.
 *  - 1~8 specifies the parameter range is from 1 to 8. 1~15 specifies the parameter range is from 1 to 15. 1~255 specifies the parameter range from 1 to 255.
 *  - 1 means the parameter value can only be 1.
 *  - Note1: functions with the suffix "_ex" have these 4 parameters. Other functions only have the "size" parameter and the driver splits the "size" into these 4 parameters.
 *  - Note2: The maximum of total transaction length is 256K.\n
 *    #hal_i2c_master_send_polling() for example, the "size" will be divided like this: send_packet_length = 1, send_bytes_in_one_packet = size.
 *          As a result, the total size should be: send_packet_length * send_bytes_in_one_packet = 1 * size = size. The range of "size" should be from 1 to 8.
 * |API                                         |send_packet_length(Ns) | send_bytes_in_one_packet(Ms) | receive_packet_length(Nr) | receive_bytes_in_one_packet(Mr) |
 * |--------------------------------------------|-----------------------|------------------------------|---------------------------|---------------------------------|
 * |hal_i2c_master_send_polling                 |          1            |            1~8               |            NA             |                NA               |
 * |hal_i2c_master_receive_polling              |          NA           |            NA                |            1              |                1~8              |
 * |hal_i2c_master_send_to_receive_polling      |          1            |            1~8               |            1              |                1~8              |
 * |hal_i2c_master_send_dma                     |          1            |            1~65535           |            NA             |                NA               |
 * |hal_i2c_master_receive_dma                  |          NA           |            NA                |            1~65535        |                1                |
 * |hal_i2c_master_send_to_receive_dma          |          1            |            1~65535           |            1~65534        |                1                |
 * |hal_i2c_master_send_dma_ex                  |          1~65535      |            1~65535           |            NA             |                NA               |
 * |hal_i2c_master_receive_dma_ex               |          NA           |            NA                |            1~65535        |                1~65535          |
 * |hal_i2c_master_send_to_receive_dma_ex       |          1            |            1~65535           |            1~65534        |                1~65535          |
 *
 * - \b Waveform \b pattern \b supported \b by \b the \b APIs \n
 *  The 4 parameters (send_packet_length(Ns), send_bytes_in_one_packet(Ms), receive_packet_length(Nr), receive_bytes_in_one_packet(Mr) will also affect the transaction packet.
 *  The relationship between transaction packet and these 4 parameters is shown below.
 *  - Ns is the send_packet_length.
 *  - Ms is the send_bytes_in_one_packet.
 *  - Nr is the receive_packet_length.
 *  - Mr is the receive_bytes_in_one_packet.
 * |API                                          |transaction packet format                                 |
 * |---------------------------------------------|----------------------------------------------------------|
 * | hal_i2c_master_send_polling                 |  @image html hal_i2c_send_poling_waveform.png            |
 * | hal_i2c_master_receive_polling              |  @image html hal_i2c_receive_poling_waveform.png         |
 * | hal_i2c_master_send_to_receive_polling      |  @image html hal_i2c_send_to_receive_poling_waveform.png |
 * | hal_i2c_master_send_dma                     |  @image html hal_i2c_send_dma_waveform.png            |
 * | hal_i2c_master_receive_dma                  |  @image html hal_i2c_receive_dma_waveform.png         |
 * | hal_i2c_master_send_to_receive_dma          |  @image html hal_i2c_send_to_receive_dma_waveform.png |
 * | hal_i2c_master_send_dma_ex                  |  @image html hal_i2c_send_dma_ex_waveform.png            |
 * | hal_i2c_master_receive_dma_ex               |  @image html hal_i2c_receive_dma_ex_waveform.png         |
 * | hal_i2c_master_send_to_receive_dma_ex       |  @image html hal_i2c_send_to_receive_dma_ex_waveform.png |
 *
 *
 *
 *
 */


/** @defgroup hal_i2c_master_define Define
 * @{
  */

/** @brief  The maximum polling mode transaction size.
  */
#define HAL_I2C_MAXIMUM_POLLING_TRANSACTION_SIZE  8

/** @brief  The maximum DMA mode transaction size.
  */
#define HAL_I2C_MAXIMUM_DMA_TRANSACTION_SIZE   15

/**
  * @}
  */

/** @addtogroup hal_i2c_master_enum Enum
  * @{
  */

/*****************************************************************************
* I2C master
*****************************************************************************/
/** @brief This enum defines the I2C port.
 *  The platform supports 3 I2C masters. Two of them support polling mode and DMA mode,
 *  while the other one only supports polling mode. For more information about the polling mode,
 *  DMA mode, queue mode, please refer to @ref HAL_I2C_Features_Chapter. The details
 *  are shown below:
 *  - I2C masters supported feature table \n
 *    V : means support;  X : means not support.
 * |I2C Master   | Polling mode | DMA mode | Extended DMA mode |
 * |-------------|--------------|----------|-------------------|
 * |I2C0         |      V       |    V     |         V         |
 * |I2C1         |      V       |    V     |         V         |
 * |I2C2         |      V       |    V     |         V         |
 *
 *
*/
typedef enum {
    HAL_I2C_MASTER_0 = 0,                /**< I2C master 0. */
    HAL_I2C_MASTER_1 = 1,                /**< I2C master 1. */
    HAL_I2C_MASTER_2 = 2,                /**< I2C master 2. */
    HAL_I2C_MASTER_MAX                   /**< The total number of I2C masters (invalid I2C Master number). */
} hal_i2c_port_t;





/**
* @addtogroup HAL
* @{
* @addtogroup GPIO
* @{
*
* @addtogroup hal_gpio_enum Enum
* @{
*/

/*****************************************************************************
* GPIO
*****************************************************************************/
/** @brief This enum defines the GPIO port.
 *  The platform supports a total of 49 GPIO pins with various functionality.
 *
*/

typedef enum {
    HAL_GPIO_0 = 0,                            /**< GPIO pin0. */
    HAL_GPIO_1 = 1,                            /**< GPIO pin1. */
    HAL_GPIO_2 = 2,                            /**< GPIO pin2. */
    HAL_GPIO_3 = 3,                            /**< GPIO pin3. */
    HAL_GPIO_4 = 4,                            /**< GPIO pin4. */
    HAL_GPIO_5 = 5,                            /**< GPIO pin5. */
    HAL_GPIO_6 = 6,                            /**< GPIO pin6. */
    HAL_GPIO_7 = 7,                            /**< GPIO pin7. */
    HAL_GPIO_8 = 8,                            /**< GPIO pin8. */
    HAL_GPIO_9 = 9,                            /**< GPIO pin9. */
    HAL_GPIO_10 = 10,                          /**< GPIO pin10. */
    HAL_GPIO_11 = 11,                          /**< GPIO pin11. */
    HAL_GPIO_12 = 12,                          /**< GPIO pin12. */
    HAL_GPIO_13 = 13,                          /**< GPIO pin13. */
    HAL_GPIO_14 = 14,                          /**< GPIO pin14. */
    HAL_GPIO_15 = 15,                          /**< GPIO pin15. */
    HAL_GPIO_16 = 16,                          /**< GPIO pin16. */
    HAL_GPIO_17 = 17,                          /**< GPIO pin17. */
    HAL_GPIO_18 = 18,                          /**< GPIO pin18. */
    HAL_GPIO_19 = 19,                          /**< GPIO pin19. */
    HAL_GPIO_20 = 20,                          /**< GPIO pin20. */
    HAL_GPIO_21 = 21,                          /**< GPIO pin21. */
    HAL_GPIO_22 = 22,                          /**< GPIO pin22. */
    HAL_GPIO_23 = 23,                          /**< GPIO pin23. */
    HAL_GPIO_24 = 24,                          /**< GPIO pin24. */
    HAL_GPIO_25 = 25,                          /**< GPIO pin25. */
    HAL_GPIO_26 = 26,                          /**< GPIO pin26. */
    HAL_GPIO_27 = 27,                          /**< GPIO pin27. */
    HAL_GPIO_28 = 28,                          /**< GPIO pin28. */
    HAL_GPIO_29 = 29,                          /**< GPIO pin29. */
    HAL_GPIO_30 = 30,                          /**< GPIO pin30. */
    HAL_GPIO_31 = 31,                          /**< GPIO pin31. */
    HAL_GPIO_32 = 32,                          /**< GPIO pin32. */
    HAL_GPIO_33 = 33,                          /**< GPIO pin33. */
    HAL_GPIO_34 = 34,                          /**< GPIO pin34. */
    HAL_GPIO_35 = 35,                          /**< GPIO pin35. */
    HAL_GPIO_36 = 36,                          /**< GPIO pin36. */
    HAL_GPIO_MAX                               /**< The total number of GPIO pins (invalid GPIO pin number). */
} hal_gpio_pin_t;



#ifdef HAL_GPIO_FEATURE_CLOCKOUT
/**
 * @addtogroup HAL
 * @{
 * @addtogroup GPIO
 * @{
 * @addtogroup hal_gpio_enum Enum
 * @{
 */
/*****************************************************************************
* CLKOUT
*****************************************************************************/
/** @brief  This enum defines output clock number of GPIO */
typedef enum {
    HAL_GPIO_CLOCK_0   = 0,              /**< define GPIO output clock 0 */
    HAL_GPIO_CLOCK_1   = 1,              /**< define GPIO output clock 1 */
    HAL_GPIO_CLOCK_2   = 2,              /**< define GPIO output clock 2 */
    HAL_GPIO_CLOCK_3   = 3,              /**< define GPIO output clock 3 */
    HAL_GPIO_CLOCK_4   = 4,              /**< define GPIO output clock 4 */
    HAL_GPIO_CLOCK_5   = 5,              /**< define GPIO output clock 5 */
    HAL_GPIO_CLOCK_MAX                   /**< define GPIO output clock max number(invalid) */
} hal_gpio_clock_t;


/** @brief This enum defines output clock mode of GPIO */
typedef enum {
    HAL_GPIO_CLOCK_MODE_32K = 0,        /**< define GPIO output clock mode as 32KHz */
    HAL_GPIO_CLOCK_MODE_24M = 2,        /**< define GPIO output clock mode as 24MHz */
    HAL_GPIO_CLOCK_MODE_48M = 3,        /**< define GPIO output clock mode as 48MHz */
    HAL_GPIO_CLOCK_MODE_40K = 5,        /**< define GPIO output clock mode as 40MHz */
    HAL_GPIO_CLOCK_MODE_MAX             /**< define GPIO output clock mode of max number(invalid) */
} hal_gpio_clock_mode_t;
/**
  * @}
  */

/**
 * @}
 * @}
 */
#endif



/**
* @addtogroup HAL
* @{
* @addtogroup ADC
* @{
*
* @addtogroup hal_adc_enum Enum
* @{
*/

/*****************************************************************************
* ADC
*****************************************************************************/
/** @brief adc channel */
typedef enum {
    HAL_ADC_CHANNEL_0 = 0,                        /**< ADC channel 0. */
    HAL_ADC_CHANNEL_1 = 1,                        /**< ADC channel 1. */
    HAL_ADC_CHANNEL_2 = 2,                        /**< ADC channel 2. */
    HAL_ADC_CHANNEL_3 = 3,                        /**< ADC channel 3. */
    HAL_ADC_CHANNEL_4 = 4,                        /**< ADC channel 4. */
    HAL_ADC_CHANNEL_5 = 5,                        /**< ADC channel 5. */
    HAL_ADC_CHANNEL_6 = 6,                        /**< ADC channel 6. */
    HAL_ADC_CHANNEL_MAX                           /**< The total number of ADC channels (invalid ADC channel).*/
} hal_adc_channel_t;

/**
  * @}
  */


/**
 * @}
 * @}
 */




#ifdef HAL_I2S_MODULE_ENABLED
/**
* @addtogroup HAL
* @{
* @addtogroup I2S
* @{
*
* @addtogroup hal_i2s_enum Enum
* @{
*/

/*****************************************************************************
* I2S
*****************************************************************************/
/** @brief This enum defines initial type of the I2S.
 */

typedef enum {
    HAL_I2S_0  = 0,    /**< I2S interfeace 0. */
    HAL_I2S_1  = 1,    /**< I2S interfeace 1. */
    HAL_I2S_MAX
} hal_i2s_port_t;


#ifdef HAL_I2S_EXTENDED

/** @brief I2S DMA configure structure */
typedef struct {
    uint32_t *buffer;           /**< buffer is the pointer to memory buffer for the DMA buffer.*/
    uint32_t threshold;         /**< threshold is the value of the DMA buffer threshold.*/
    uint32_t buffer_length;     /**< buffer_length is the length to memory buffer for the DMA buffer.*/ 
} hal_i2s_dma_config_t;

/** @brief I2S sample widths.  */
typedef enum {
    HAL_I2S_SAMPLE_WIDTH_8BIT  = 0,   /**< I2S sample width is 8bit. (Invalid)*/
    HAL_I2S_SAMPLE_WIDTH_16BIT = 1,   /**< I2S sample width is 16bit. */
    HAL_I2S_SAMPLE_WIDTH_24BIT = 2    /**< I2S sample width is 24bit. */
} hal_i2s_sample_width_t;


/** @brief Number of bits per frame sync(FS). This parameter determines the bits of a complete sample of both left and right channels.*/
typedef enum {
    HAL_I2S_FRAME_SYNC_WIDTH_32  = 0,   /**< 32 bits per frame. */
    HAL_I2S_FRAME_SYNC_WIDTH_64  = 1,   /**< 64 bits per frame. */
    HAL_I2S_FRAME_SYNC_WIDTH_128  = 2   /**< 128 bits per frame. (Invalid)*/
} hal_i2s_frame_sync_width_t;


/** @brief Enable or disable right channel of the I2S TX to send the same data as on the left channel of the I2S TX.\n
        This function only works when the sample width of the I2S is 16 bits.*/
typedef enum {
    HAL_I2S_TX_DUPLICATE_DISABLE = 0,        /**< Keep data to its channel. */
    HAL_I2S_TX_DUPLICATE_LEFT_CHANNEL  = 1   /**< Assume input is mono data for left channel, the data is duplicated to right channel.\n
                                                                                  Enable this funtion is not allowed when the channel number of I2S TX is set to HAL_I2S_STEREO.*/
} hal_i2s_tx_mode_t;

#endif //  #ifdef HAL_I2S_EXTENDED


/** @brief Enable or disable data swapping between right and left channels of the I2S link. */
typedef enum {
    HAL_I2S_LR_SWAP_DISABLE = 0,  /**< Disable the data swapping. */
    HAL_I2S_LR_SWAP_ENABLE  = 1   /**< Enable the data swapping.  */
} hal_i2s_lr_swap_t;


/** @brief Enable or disable word select clock inverting of the I2S link. */
typedef enum {
    HAL_I2S_WORD_SELECT_INVERSE_DISABLE = 0,  /**< Disable the word select clock inverting. */
    HAL_I2S_WORD_SELECT_INVERSE_ENABLE  = 1   /**< Enable the word select clock inverting.  */
} hal_i2s_word_select_inverse_t;

/** @brief This enum defines initial type of the I2S.
 */

typedef enum {
    HAL_I2S_TYPE_EXTERNAL_MODE          = 0,        /**< External mode. I2S mode with external codec.*/
    HAL_I2S_TYPE_EXTERNAL_TDM_MODE      = 1,        /**< External TDM mode. I2S TDM mode with external codec. (Invalid)*/
    HAL_I2S_TYPE_INTERNAL_MODE          = 2,        /**< Internal mode. I2S mode with internal codec. (Invalid)*/
    HAL_I2S_TYPE_INTERNAL_LOOPBACK_MODE = 3,        /**< I2S internal loopback mode. Data loopback mode.*/
    HAL_I2S_TYPE_INTERNAL_TDM_LOOPBACK_MODE = 4,    /**< TDM internal loopback mode. Data loopback mode. (Invalid)*/
    HAL_I2S_TYPE_MAX = 5
} hal_i2s_initial_type_t;


/** @brief I2S event */
typedef enum {
    HAL_I2S_EVENT_ERROR               = -1, /**<  An error occurred during the function call. */
    HAL_I2S_EVENT_NONE                =  0, /**<  No error occurred during the function call. */
    HAL_I2S_EVENT_OVERFLOW            =  1, /**<  RX data overflow. */
    HAL_I2S_EVENT_UNDERFLOW           =  2, /**<  TX data underflow. */
    HAL_I2S_EVENT_DATA_REQUEST        =  3, /**<  Request for user-defined data. */
    HAL_I2S_EVENT_DATA_NOTIFICATION   =  4, /**<  Notify user the RX data is ready. */
    HAL_I2S_EVENT_END                 =  5  /**<  No more data from user.*/
} hal_i2s_event_t;


/** @brief I2S sampling rates */
typedef enum {
    HAL_I2S_SAMPLE_RATE_8K        = 0,  /**<  8000Hz  */
    HAL_I2S_SAMPLE_RATE_11_025K   = 1,  /**<  11025Hz */
    HAL_I2S_SAMPLE_RATE_12K       = 2,  /**<  12000Hz */
    HAL_I2S_SAMPLE_RATE_16K       = 3,  /**<  16000Hz */
    HAL_I2S_SAMPLE_RATE_22_05K    = 4,  /**<  22050Hz */
    HAL_I2S_SAMPLE_RATE_24K       = 5,  /**<  24000Hz */
    HAL_I2S_SAMPLE_RATE_32K       = 6,  /**<  32000Hz */
    HAL_I2S_SAMPLE_RATE_44_1K     = 7,  /**<  44100Hz */
    HAL_I2S_SAMPLE_RATE_48K       = 8,  /**<  48000Hz */
    HAL_I2S_SAMPLE_RATE_88_2K     = 9,  /**<  88200Hz */
    HAL_I2S_SAMPLE_RATE_96K       = 10, /**<  96000Hz */
    HAL_I2S_SAMPLE_RATE_176_4K    = 11, /**<  176400Hz */
    HAL_I2S_SAMPLE_RATE_192K      = 12  /**<  192000Hz */
} hal_i2s_sample_rate_t;


/**
  * @}
  */

/**
 * @}
 * @}
 */
#endif






#ifdef HAL_RTC_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup RTC
 * @{
 * @defgroup hal_rtc_define Define
 * @{
 */

/** @brief  This macro defines a maximum number for backup data that used in #hal_rtc_set_data(),
  * #hal_rtc_get_data(), #hal_rtc_clear_data functions.
  */
#define HAL_RTC_BACKUP_BYTE_NUM_MAX     (4)

/** @brief  This macro defines a maximum number for RTC retention sram cells.
  */
#define HAL_RTC_RETENTION_SRAM_NUMBER_MAX     (4)

/** @brief  This macro defines the mask value for all RTC retention sram cells.
  */
#define HAL_RTC_RETENTION_SRAM_NUMBER_MASK     (0xf)

/** @brief  This macro defines the mask value for the smallest
 *          negative CALI value.
  */
#define HAL_RTC_NEGATIVE_CALI_MASK     (0x1000)

/**
 * @}
 */

/** @defgroup hal_rtc_struct Struct
  * @{
  */

/** @brief RTC time structure definition. */
typedef struct {
    uint8_t rtc_sec;                                  /**< Seconds after minutes     - [0,59]  */
    uint8_t rtc_min;                                  /**< Minutes after the hour    - [0,59]  */
    uint8_t rtc_hour;                                 /**< Hours after midnight      - [0,23]  */
    uint8_t rtc_day;                                  /**< Day of the month          - [1,31]  */
    uint8_t rtc_mon;                                  /**< Months                    - [1,12]  */
    uint8_t rtc_week;                                 /**< Days in a week            - [0,6]   */
    uint8_t rtc_year;                                 /**< Years                     - [0,127] */
    uint16_t rtc_milli_sec;                           /**< Millisecond value, when in time API, this represents the read only register rtc_int_cnt - [0,65535]; when in alarm API, this represents millisecond alarm value - [0,15], equals to 0 to 937.5ms */
} hal_rtc_time_t;

/** @brief RTC GPIO control structure definition. */
typedef struct {
    bool is_sw_control;               /**< RTC GPIO output is controlled by software or hardware signals like EINT, ALARM, or TICK. */
    bool is_sw_output_high;           /**< Software makes RTC GPIO output high or not. Only valid when is_sw_control parameter is true. */
    bool is_eint_output_high;         /**< Hardware signal EINT makes RTC GPIO output high or not. */
    bool is_alarm_output_high;        /**< Hardware signal ALARM makes RTC GPIO output high or not. */
    bool is_tick_output_high;         /**< Hardware signal TICK(default is 1/8 second) makes RTC GPIO output high or not. */
    bool is_clear_output;             /**< Clear GPIO output high to low or not, which caused output high by former hardware signal. Only valid when is_sw_control parameter is false. */
} hal_rtc_gpio_control_t;

/** @brief RTC GPIO control structure definition. */
typedef struct {
    bool is_enable_rtc_eint;        /**< Enable RTC EINT or not. */
    bool is_falling_edge_active;    /**< Configure RTC EINT as falling edge active or not. */
    bool is_enable_debounce;        /**< Enable RTC EINT debounce or not, if enabled, EINT debounce time is 4T*32k. */
} hal_rtc_eint_config_t;
/**
  * @}
  */

/**
 * @defgroup hal_rtc_enum Enum
 * @{
 */

/** @brief This enum defines the mode of rtc retention sram. */
typedef enum {
    HAL_RTC_SRAM_NORMAL_MODE = 0,     /**< Change SRAM mode to normal mode. */
    HAL_RTC_SRAM_SLEEP_MODE = 1,      /**< Change SRAM mode to sleep mode. */
    HAL_RTC_SRAM_PD_MODE = 2,         /**< Change SRAM mode power down mode. */
} hal_rtc_sram_mode_t;

/** @brief RTC current time change notification period selections. */
typedef enum {
    HAL_RTC_TIME_NOTIFICATION_NONE = 0,                     /**< No need for a time notification. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_SECOND = 1,             /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every second. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_MINUTE = 2,             /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every minute. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_HOUR = 3,               /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every hour. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_DAY = 4,                /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every day. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_MONTH = 5,              /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every month. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_YEAR = 6,               /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every year. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_SECOND_1_2 = 7,         /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every one-half of a second. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_SECOND_1_4 = 8,         /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every one-fourth of a second. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_SECOND_1_8 = 9,         /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every one-eighth of a second. */
    HAL_RTC_TIME_NOTIFICATION_EVERY_SECOND_1_16 = 10        /**< Execute a callback function set by #hal_rtc_set_time_notification_period() for every one-sixteenth of a second. */
} hal_rtc_time_notification_period_t;

/** @brief This enum defines the type of RTC GPIO. */
typedef enum {
    HAL_RTC_GPIO_0 = 0,     /**< RTC GPIO 0 output. */
    HAL_RTC_GPIO_1 = 1,     /**< RTC GPIO 1 output. */
} hal_rtc_gpio_t;

/**
 * @}
 */

/**
 * @}
 * @}
 */
#endif



/**
 * @addtogroup HAL
 * @{
 * @addtogroup EINT
 * @{
 * @addtogroup hal_eint_enum Enum
 * @{
 */

/*****************************************************************************
* eint
*****************************************************************************/
/** @brief EINT pins. */
typedef enum {
    HAL_EINT_NUMBER_0 = 0,
    HAL_EINT_NUMBER_1 = 1,
    HAL_EINT_NUMBER_2 = 2,
    HAL_EINT_NUMBER_3 = 3,
    HAL_EINT_NUMBER_4 = 4,
    HAL_EINT_NUMBER_5 = 5,
    HAL_EINT_NUMBER_6 = 6,
    HAL_EINT_NUMBER_7 = 7,
    HAL_EINT_NUMBER_8 = 8,
    HAL_EINT_NUMBER_9 = 9,
    HAL_EINT_NUMBER_10 = 10,
    HAL_EINT_NUMBER_11 = 11,
    HAL_EINT_NUMBER_12 = 12,
    HAL_EINT_USB = 13,
    HAL_EINT_NUMBER_14 = 14,
    HAL_EINT_NUMBER_15 = 15,
    HAL_EINT_NUMBER_16 = 16,
    HAL_EINT_NUMBER_17 = 17,
    HAL_EINT_NUMBER_18 = 18,
    HAL_EINT_NUMBER_19 = 19,
    HAL_EINT_NUMBER_20 = 20,
    HAL_EINT_NUMBER_21 = 21,
    HAL_EINT_NUMBER_22 = 22,
    HAL_EINT_NUMBER_23 = 23,      /**< WLAN_ACT   */
    HAL_EINT_NUMBER_24 = 24,      /**< UART0_RXD  */   
    HAL_EINT_NUMBER_25 = 25,      /**< UART1_RXD  */
    HAL_EINT_NUMBER_26 = 26,      /**< UART2_RXD  */
    HAL_EINT_NUMBER_27 = 27,      /**< UART3_RXD  */
    HAL_EINT_NUMBER_28 = 28,      /**< RTC_EVENT  */
    HAL_EINT_NUMBER_29 = 29,      /**< KP_EVENT   */
    HAL_EINT_NUMBER_30 = 30,      /**< PDN_USB11  */
    HAL_EINT_NUMBER_31 = 31,      /**< PMU_WAKEUP */
    HAL_EINT_NUMBER_MAX           /**< The total number of EINT channels (invalid EINT channel). */
} hal_eint_number_t;

/**
  * @}
  */
/**
 * @}
 * @}
 */


#ifdef HAL_GPT_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup GPT
 * @{
 * @addtogroup hal_gpt_enum Enum
 * @{
 */

/*****************************************************************************
* GPT
*****************************************************************************/
/** @brief GPT port */
typedef enum {
    HAL_GPT_0 = 0,                          /**< GPT port 0. */
    HAL_GPT_1 = 1,                          /**< GPT port 1. */
    HAL_GPT_2 = 2,                          /**< GPT port 2. */
    HAL_GPT_3 = 3,                          /**< GPT port 3. */
    HAL_GPT_4 = 4,                          /**< GPT port 4. */
    HAL_GPT_5 = 5,                          /**< GPT port 5. */
    HAL_GPT_MAX_PORT = 6,                        /**< The total number of GPT ports (invalid GPT port). */
    HAL_GPT_MAX = 6
} hal_gpt_port_t;

/** @brief GPT clock source */
typedef enum {
    HAL_GPT_CLOCK_SOURCE_32K = 0,            /**< Set the GPT clock source to 32kHz. */
    HAL_GPT_CLOCK_SOURCE_1M  = 1             /**< Set the GPT clock source to 1MHz. */
} hal_gpt_clock_source_t;

/** @brief  The maximum time of millisecond timer.
  */
#define HAL_GPT_MAXIMUM_MS_TIMER_TIME    (130150523)

/**
  * @}
  */
/**
 * @}
 * @}
 */
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup FLASH
 * @{
 */

/*****************************************************************************
* Flash
*****************************************************************************/

/** @defgroup hal_flash_define Define
 * @{
  */

 /** @brief  This macro defines the Flash base address.
   */
#define HAL_FLASH_BASE_ADDRESS    (0x08000000)
/**
  * @}
  */

/**
 * @}
 * @}
 */
#endif

#ifdef HAL_GDMA_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup GDMA
 * @{
 * @addtogroup hal_gdma_enum Enum
 * @{
 */
/*****************************************************************************
* GDMA
*****************************************************************************/
/** @brief gdma channel */
typedef enum {
    HAL_GDMA_CHANNEL_0 = 0,                        /**< GDMA channel 0, for AP side user*/
    HAL_GDMA_CHANNEL_1 = 1,                    /**< GDMA channel 1, for MD side user*/
    HAL_GDMA_CHANNEL_MAX                           /**< The total number of GDMA channels (invalid GDMA channel). */
} hal_gdma_channel_t;

/**
  * @}
  */


/**
 * @}
 * @}
 */
#endif


/**
 * @addtogroup HAL
 * @{
 * @addtogroup PWM
 * @{
 * @addtogroup hal_pwm_enum Enum
 * @{
 */
/*****************************************************************************
* PWM
*****************************************************************************/
/** @brief The PWM channels */
typedef enum {
    HAL_PWM_0 = 0,                            /**< PWM channel 0. */
    HAL_PWM_1 = 1,                            /**< PWM channel 1. */
    HAL_PWM_2 = 2,                            /**< PWM channel 2. */
    HAL_PWM_3 = 3,                            /**< PWM channel 3. */
    //HAL_PWM_4 = 4,                            /**< PWM channel 4. */
    //HAL_PWM_5 = 5,                            /**< PWM channel 5. */
    HAL_PWM_MAX_CHANNEL                     /**< The total number of PWM channels (invalid PWM channel).*/
} hal_pwm_channel_t;


/** @brief PWM clock source options */
typedef enum {
    HAL_PWM_CLOCK_13MHZ = 0,                /**< PWM clock source 13MHz. */
    HAL_PWM_CLOCK_32KHZ = 1,                /**< PWM clock srouce 32kHz. */
} hal_pwm_source_clock_t ;

/**
  * @}
  */
/**
 * @}
 * @}
 */


#ifdef HAL_WDT_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup WDT
 * @{
 */

/*****************************************************************************
* WDT
*****************************************************************************/
/** @brief  Define the platform related WDT restart address.
  */
#define WDT_RESTART_ADDRESS    (0xA209000C)

/** @brief  Define the platform related magic number for WDT restart address.
  */
#define WDT_RESTART_KEY        (0x1456789A)

/**
 * @}
 * @}
 */
#endif

#ifdef HAL_CACHE_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup CACHE
 * @{
 */

/*****************************************************************************
* Cache
*****************************************************************************/
/* NULL */

/**
 * @}
 * @}
 */
#endif

#ifdef HAL_GPC_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup GPC
 * @{
 * @addtogroup hal_gpc_enum Enum
 * @{
 */
/** @brief GPC port */
typedef enum {
    HAL_GPC_0 = 0,                          /**< GPC port 0. */
    HAL_GPC_MAX_PORT                        /**< The total number of GPC ports (invalid GPC port). */
} hal_gpc_port_t;


/**
  * @}
  */
/**
 * @}
 * @}
 */
#endif


#ifdef HAL_SD_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup SD
 * @{
 * @addtogroup hal_sd_enum Enum
 * @{
 */
/*****************************************************************************
* SD
*****************************************************************************/
/** @brief  This enum defines the SD/eMMC port. */
typedef enum {
    HAL_SD_PORT_0 = 0,                                             /**<  SD/eMMC port 0. */
    HAL_SD_PORT_1 = 1                                              /**<  SD/eMMC port 1. */
} hal_sd_port_t;

/**
  * @}
  */
/**
 * @}
 * @}
 */
#endif


#ifdef HAL_SDIO_SLAVE_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup SDIO
 * @{
 * @addtogroup hal_sdio_enum Enum
 * @{
 */
/*****************************************************************************
* SDIO
*****************************************************************************/
/** @brief  This enum defines the SDIO port.  */
typedef enum {
    HAL_SDIO_SLAVE_PORT_0 = 0,                                             /**< SDIO slave port 0. */
} hal_sdio_slave_port_t;


/**
  * @}
  */
/**
 * @}
 * @}
 */
#endif


#ifdef HAL_SDIO_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup SDIO
 * @{
 * @addtogroup hal_sdio_enum Enum
 * @{
 */
/*****************************************************************************
* SDIO
*****************************************************************************/
/** @brief  This enum defines the SDIO port.  */
typedef enum {
    HAL_SDIO_PORT_0 = 0,                                             /**< SDIO port 0. */
    HAL_SDIO_PORT_1 = 1                                              /**< SDIO port 1. */
} hal_sdio_port_t;

/**
  * @}
  */
/**
 * @}
 * @}
 */
#endif

#ifdef HAL_CLOCK_MODULE_ENABLED
/**
 * @addtogroup HAL
 * @{
 * @addtogroup CLOCK
 * @{
 */
//#include "hal_pmu_internal.h"

/*****************************************************************************
* Clock
*****************************************************************************/
/** @addtogroup clock
 * @{
 *
 * @section CLOCK_CG_ID_Usage_Chapter HAL_CLOCK_CG_ID descriptions
 *
 * Each #hal_clock_cg_id is related to one hardware clock gate. Please check the following impacts before controlling the clock.
 *
 * The description of API parameters for HAL_CLOCK_CG_ID is listed below:
 * | HAL_CLOCK_CG_ID            |Details                                                                            |
 * |----------------------------|-----------------------------------------------------------------------------------|
 * |\b HAL_CLOCK_CG_CM4_OSTIMER | The clock for OS timer. It cannot be disabled, otherwise the system will fail.|
 * |\b HAL_CLOCK_CG_UART0       | The clock for UART0. It cannot be disabled, otherwise the system will fail.|
 * |\b HAL_CLOCK_CG_SPISLV      | The clock for SPI slave. This clock should be enabled when it is connected to the master device if choosing a custom driver.|
 * |\b HAL_CLOCK_CG_GPCOUNTER   | The clock for general purpose counter timer. It cannot be disabled, otherwise the system will fail.|
 * |\b HAL_CLOCK_CG_CM_SYSROM   | The clock for system ROM. It cannot be disabled, otherwise the system will fail.|
 * |\b HAL_CLOCK_CG_SFC         | The clock for serial flash controller. It cannot be disabled, otherwise the system will fail.|
 * |\b HAL_CLOCK_CG_DISP_PWM    | The clock for display. This clock should be enabled if choosing a custom display driver.|
 * |\b HAL_CLOCK_CG_AAL         | The clock for Ambient-Light Adaptive Luma. This clock should be enabled if choosing a custom display driver.|
 * |\b HAL_CLOCK_CG_DSI0        | The clock for display serial interface. This clock should be enabled if choosing a custom display driver.|
 * |\b HAL_CLOCK_CG_SENSOR_DMA  | The clock for sensor DMA. This clock should be enabled when using I2C0 or I2C1 with custom driver.|
 */
/** @brief Use clock_id in CLOCK API. */
/*************************************************************************
 * Define clock gating registers and bit structure.
 * Note: Mandatory, modify clk_cg_mask in hal_clock.c source file, if hal_clock_cg_id has changed.
 *************************************************************************/
typedef enum {
    /* NR_PDN_COND0 = 30 */
    HAL_CLOCK_CG_DMA                    =  0,        /* bit 0, PDN_COND0_FROM */
    HAL_CLOCK_CG_I2S1                   =  1,        /* bit 1, */
    HAL_CLOCK_CG_SPISLV                 =  2,        /* bit 2, */
    HAL_CLOCK_CG_SDIOMST0               =  3,        /* bit 3, */
    HAL_CLOCK_CG_SDIOMST1               =  4,        /* bit 4, */
    HAL_CLOCK_CG_I2S0                   =  5,        /* bit 5, */
    HAL_CLOCK_CG_SPIMST0                =  6,        /* bit 6, */
    HAL_CLOCK_CG_SPIMST1                =  7,        /* bit 7, */
    HAL_CLOCK_CG_UART1                  =  8,        /* bit 8, */
    HAL_CLOCK_CG_UART2                  =  9,        /* bit 9, */
    HAL_CLOCK_CG_USIM_26M               = 10,        /* bit 10, */
    HAL_CLOCK_CG_UART3                  = 11,        /* bit 11, */
    HAL_CLOCK_CG_PWMSG_BUS              = 12,        /* bit 12, */
    HAL_CLOCK_CG_PWMSG                  = 13,        /* bit 13, */
    HAL_CLOCK_CG_PWMSG1                 = 14,        /* bit 14, */
    HAL_CLOCK_CG_PWMSG2                 = 15,        /* bit 15, */
    HAL_CLOCK_CG_CM_SYSROM              = 16,        /* bit 16, */
    HAL_CLOCK_CG_SFC                    = 17,        /* bit 17, */
    HAL_CLOCK_CG_TRNG                   = 18,        /* bit 18, */
    HAL_CLOCK_CG_AESOTF                 = 19,        /* bit 19, */
    HAL_CLOCK_CG_UART0                  = 20,        /* bit 20, */
    HAL_CLOCK_CG_CRYPTO                 = 21,        /* bit 21, */
    HAL_CLOCK_CG_SDIOSLV                = 22,        /* bit 22, */
    HAL_CLOCK_CG_GPTIMER                = 23,        /* bit 23, */
    HAL_CLOCK_CG_OSTIMER                = 24,        /* bit 24, */
    HAL_CLOCK_CG_USB48M                 = 25,        /* bit 25, */
    HAL_CLOCK_CG_USB_BUS                = 26,        /* bit 26, */
    HAL_CLOCK_CG_CIPHER                 = 27,        /* bit 27, */
    HAL_CLOCK_CG_SDIOMST0_BUS           = 28,        /* bit 28, */
    HAL_CLOCK_CG_SDIOMST1_BUS           = 29,        /* bit 29, PDN_COND0_TO */

    /* NR_XO_PDN_COND0 = 17 */
    HAL_CLOCK_CG_48M                    = (0 + 32),    /* bit 0, XO_PDN_COND0_FROM */
    HAL_CLOCK_CG_SENSOR_DMA             = (1 + 32),    /* bit 1, */
    HAL_CLOCK_CG_PWM0                   = (4 + 32),    /* bit 4, */
    HAL_CLOCK_CG_PWM1                   = (5 + 32),    /* bit 5, */
    HAL_CLOCK_CG_PWM2                   = (6 + 32),    /* bit 6, */
    HAL_CLOCK_CG_PWM3                   = (7 + 32),    /* bit 7, */
    HAL_CLOCK_CG_SENSOR_CTRL_TOP_AO     = (8 + 32),    /* bit 8, */
    HAL_CLOCK_CG_USIM_BCLK              = (9 + 32),    /* bit 9, */
    HAL_CLOCK_CG_I2C0                   = (10 + 32),   /* bit 10, */
    HAL_CLOCK_CG_I2C1                   = (11 + 32),   /* bit 11, */
    HAL_CLOCK_CG_I2C2                   = (12 + 32),   /* bit 12, */
    HAL_CLOCK_CG_SENSOR_uP              = (13 + 32),   /* bit 13, */
    HAL_CLOCK_CG_SPM                    = (16 + 32),   /* bit 16, */
    HAL_CLOCK_CG_EFUSE                  = (18 + 32),   /* bit 18, */
    HAL_CLOCK_CG_SEJ                    = (19 + 32),   /* bit 19, */
    HAL_CLOCK_CG_AUXADC                 = (21 + 32),   /* bit 21, */
    HAL_CLOCK_CG_MD_XO                  = (22 + 32),   /* bit 22, XO_PDN_COND0_TO */
    HAL_CLOCK_CG_END                    = (23 + 32)
} hal_clock_cg_id;

/**
  * @}
  */

/**
 * @}
 * @}
 */
#endif

#ifdef __cplusplus
}
#endif

#endif /* __HAL_PLATFORM_H__ */

