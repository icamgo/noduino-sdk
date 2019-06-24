
#ifndef __HAL_UART_H__
#define __HAL_UART_H__

#include "hal_platform.h"






#ifdef __cplusplus
extern "C" {
#endif




/** @brief This enum defines baud rate of the UART frame. */
typedef enum {
    HAL_UART_BAUDRATE_110 = 0,           /**< Defines UART baudrate as 110 bps. */
    HAL_UART_BAUDRATE_300 = 1,           /**< Defines UART baudrate as 300 bps. */
    HAL_UART_BAUDRATE_1200 = 2,          /**< Defines UART baudrate as 1200 bps. */
    HAL_UART_BAUDRATE_2400 = 3,          /**< Defines UART baudrate as 2400 bps. */
    HAL_UART_BAUDRATE_4800 = 4,          /**< Defines UART baudrate as 4800 bps. */
    HAL_UART_BAUDRATE_9600 = 5,          /**< Defines UART baudrate as 9600 bps. */
    HAL_UART_BAUDRATE_19200 = 6,         /**< Defines UART baudrate as 19200 bps. */
    HAL_UART_BAUDRATE_38400 = 7,         /**< Defines UART baudrate as 38400 bps. */
    HAL_UART_BAUDRATE_57600 = 8,         /**< Defines UART baudrate as 57600 bps. */
    HAL_UART_BAUDRATE_115200 = 9,        /**< Defines UART baudrate as 115200 bps. */
    HAL_UART_BAUDRATE_230400 = 10,       /**< Defines UART baudrate as 230400 bps. */
    HAL_UART_BAUDRATE_460800 = 11,       /**< Defines UART baudrate as 460800 bps. */
    HAL_UART_BAUDRATE_921600 = 12,       /**< Defines UART baudrate as 921600 bps. */
#ifdef HAL_UART_FEATURE_3M_BAUDRATE
    HAL_UART_BAUDRATE_3000000 = 13,      /**< Defines UART baudrate as 3000000 bps. */
#endif
    HAL_UART_BAUDRATE_MAX                /**< Defines maximum enum value of UART baudrate. */
} hal_uart_baudrate_t;


/** @brief This enum defines word length of the UART frame. */
typedef enum {
    HAL_UART_WORD_LENGTH_5 = 0,           /**< Defines UART word length as 5 bits per frame. */
    HAL_UART_WORD_LENGTH_6 = 1,           /**< Defines UART word length as 6 bits per frame. */
    HAL_UART_WORD_LENGTH_7 = 2,           /**< Defines UART word length as 7 bits per frame. */
    HAL_UART_WORD_LENGTH_8 = 3            /**< Defines UART word length as 8 bits per frame. */
} hal_uart_word_length_t;


/** @brief This enum defines stop bit of the UART frame. */
typedef enum {
    HAL_UART_STOP_BIT_1 = 0,              /**< Defines UART stop bit as 1 bit per frame. */
    HAL_UART_STOP_BIT_2 = 1,              /**< Defines UART stop bit as 2 bits per frame. */
} hal_uart_stop_bit_t;


/** @brief This enum defines parity of the UART frame. */
typedef enum {
    HAL_UART_PARITY_NONE = 0,            /**< Defines UART parity as none. */
    HAL_UART_PARITY_ODD = 1,             /**< Defines UART parity as odd. */
    HAL_UART_PARITY_EVEN = 2             /**< Defines UART parity as even. */
} hal_uart_parity_t;


/** @brief This enum defines return status of the UART HAL public API. User should check return value after calling these APIs. */
typedef enum {
    HAL_UART_STATUS_ERROR_PARAMETER = -4,      /**< Invalid user input parameter. */
    HAL_UART_STATUS_ERROR_BUSY = -3,           /**< UART port is currently in use. */
    HAL_UART_STATUS_ERROR_UNINITIALIZED = -2,  /**< UART port has not been initialized. */
    HAL_UART_STATUS_ERROR = -1,                /**< UART driver detected a common error. */
    HAL_UART_STATUS_OK = 0                     /**< UART function executed successfully. */
} hal_uart_status_t;


/** @brief This enum defines the UART event when an interrupt occurs. */
typedef enum {
    HAL_UART_EVENT_TRANSACTION_ERROR = -1,          /**< Indicates if there is a transaction error when receiving data. */
    HAL_UART_EVENT_READY_TO_READ = 1,               /**< Indicates if there is enough data available in the RX buffer for the user to read from. */
    HAL_UART_EVENT_READY_TO_WRITE = 2               /**< Indicates if there is enough free space available in the TX buffer for the user to write into. */
} hal_uart_callback_event_t;

/** @brief This enum defines the UART flow control type. */
typedef enum {
    HAL_UART_FLOWCONTROL_NONE,                      /**< Defines UART doesn't use flow control. */
    HAL_UART_FLOWCONTROL_SOFTWARE,                  /**< Defines UART uses software flow control. */
    HAL_UART_FLOWCONTROL_HARDWARE,                  /**< Defines UART uses hardware flow control. */
} hal_uart_flowcontrol_t;

/** @brief This enum defines the UART software flow control setting. */
typedef struct {
    uint8_t xon;                            /**< Defines the xon character. */
    uint8_t xoff;                           /**< Defines the xoff character. */
    uint8_t escape_character;               /**< Defines the escape character. */
} hal_uart_sw_flowcontrol_config_t;

/**
  * @}
  */


/** @defgroup hal_uart_struct Struct
  * @{
  */


/** @brief This struct defines UART configure parameters. */
typedef struct {
    hal_uart_baudrate_t baudrate;              /**< This field represents the baudrate of the UART frame. */
    hal_uart_word_length_t word_length;        /**< This field represents the word length of the UART frame. */
    hal_uart_stop_bit_t stop_bit;              /**< This field represents the stop bit of the UART frame. */
    hal_uart_parity_t parity;                  /**< This field represents the parity of the UART frame. */
} hal_uart_config_t;


/** @brief This struct defines configuration parameters and TX/RX buffers for the VFIFO DMA associated with a specific UART channel. */
typedef struct {
    uint8_t *send_vfifo_buffer;                /**< This field represents the transmitting user allocated VFIFO buffer. It will only be used by the UART driver to send data, must be non-cacheable and aligned to 4bytes. */
    uint32_t send_vfifo_buffer_size;           /**< This field represents the size of the transmitting VFIFO buffer. */
    uint32_t send_vfifo_threshold_size;        /**< This field represents the threshold of the transmitting VFIFO buffer. VFIFO DMA will trigger an interrupt when the available bytes in VFIFO buffer are lower than the threshold. */
    uint8_t *receive_vfifo_buffer;             /**< This field represents the receiving user allocated VFIFO buffer. It will only be used by the UART driver for receiving data, and must be non-cacheable and align to 4bytes. */
    uint32_t receive_vfifo_buffer_size;        /**< This field represents size of the receiving VFIFO buffer. */
    uint32_t receive_vfifo_threshold_size;     /**< This field represents the threshold of the receiving VFIFO buffer. VFIFO DMA will trigger receive interrupt when available bytes in VFIFO buffer are more than the threshold. */
    uint32_t receive_vfifo_alert_size;         /**< This field represents the threshold size of free space left in the VFIFO buffer that activates the UART's flow control system. */
} hal_uart_dma_config_t;


typedef void (*hal_uart_callback_t)(hal_uart_callback_event_t event, void *user_data);

/**
 * @brief     This function initializes the UART hardware with basic functionality.
 * @param[in] uart_port   initializes the specified UART port number.
 * @param[in] uart_config   specifies configure parameters for UART hardware initialization.
 * @return
 *                #HAL_UART_STATUS_OK if OK. \n
 *                #HAL_UART_STATUS_ERROR_BUSY if the UART hardware has been initialized before. \n
 *                #HAL_UART_STATUS_ERROR_PARAMETER if parameter is invalid. \n
 */
hal_uart_status_t hal_uart_init(hal_uart_port_t uart_port, hal_uart_config_t *uart_config);


/**
 * @brief     This function deinitializes the UART hardware to its default status.
 * @param[in] uart_port   initializes the specified UART port number.
 * @return
 *                #HAL_UART_STATUS_OK if OK. \n
 *                #HAL_UART_STATUS_ERROR_PARAMETER if parameter is invalid. \n
 *                #HAL_UART_STATUS_ERROR_UNINITIALIZED if UART has not been initialized. \n
 * @note
 *            user must call this function when they don't use this UART port.
 */
hal_uart_status_t hal_uart_deinit(hal_uart_port_t uart_port);


/**
 * @brief     This function places one character at a time to the UART port in a polling mode.
 *               In this function, driver first poll the status of UART transmit in a loop until it's ready.
 *               Then driver places the character in the UART transmit register and returns from this function.
 * @param[in] uart_port   initializes the specified UART port number.
 * @param[in] byte   specifies the data to send.
 */
void hal_uart_put_char(hal_uart_port_t uart_port, char byte);


/**
 * @brief     This function gets one character from UART port in a polling mode.
 *               In this function, driver first polls the status of a UART receive operation in a loop until it's ready.
 *               Then driver get the character from UART receive register and return it.
 * @param[in] uart_port   initializes the specified UART port number.
 * @return
 *                 the characters received from the UART port.
 */
char hal_uart_get_char(hal_uart_port_t uart_port);


/**
 * @brief     This function gets one character from UART port in the unblocking polling mode.
 *               In this function, driver directly polls the status of a UART receive operation. If data is not ready, return error data (-1).
 *               If data is ready, get the character from the UART receive register and return it.
 * @param[in] uart_port   initializes the specified UART port number.
 * @return
 *            the character received from the UART port or error code to indicate that there is no valid character in the UART port.
 * @par       Example
 * @code
 *       uint32_t value = 0x0;
 *       value = hal_uart_get_char_unblocking(HAL_UART_0);
 *       if (value == 0xffffffff) {
 *           // Get error code, there is no valid character in the UART port.
 *       } else {
 *           // Get the valid character from the UART port.
 *           value &= 0xff;
 *       }
 * @endcode
 */
uint32_t hal_uart_get_char_unblocking(hal_uart_port_t uart_port);


/**
 * @brief     This function sends user data byte by byte in a polling mode.
 *               In this function, driver first poll status of UART transmit in a loop until it's ready.
 *               Then driver put the character into UART transmit register.
 *               Driver continues this operation until all user data is sent out,
 *               then it returns the transmitted bytes.
 * @param[in] uart_port   initializes the specified UART port number.
 * @param[in] data   specifies pointer to the user's data buffer.
 * @param[in] size   specifies size of the user's data buffer.
 * @return
 *                 bytes of transmitted data.
 */
uint32_t hal_uart_send_polling(hal_uart_port_t uart_port, const uint8_t *data, uint32_t size);





/**
 * @brief     This function receives data byte by byte in a polling mode.
 *               In this function, driver first polls the status of a UART receive operation in a loop until it's ready.
 *               Then the driver gets the character from the UART transmit register and continues this operation until all user data is received.
 *               The final output is to get the character from the UART receive register and return it.
 * @param[in] uart_port   initializes the specified UART port number.
 * @param[in] buffer   specifies pointer to the user's data buffer.
 * @param[in] size   specifies size of the user's data buffer.
 * @return
 *                 the characters received from the UART port.
 */
uint32_t hal_uart_receive_polling(hal_uart_port_t uart_port, uint8_t *buffer, uint32_t size);






#ifdef __cplusplus
}
#endif


/**
* @}
* @}
*/

#endif /* __HAL_UART_H__ */

