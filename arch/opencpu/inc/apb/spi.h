#ifndef __SPI_H
#define __SPI_H


#define HAL_SPI_MAXIMUM_POLLING_TRANSACTION_SIZE  32

/** @brief  The minimum clock frequency.
 */
#define  HAL_SPI_MASTER_CLOCK_MIN_FREQUENCY  30000

/** @brief  The maximum clock frequency.
 */
#define  HAL_SPI_MASTER_CLOCK_MAX_FREQUENCY  52000000

typedef enum {
    HAL_SPI_MASTER_SLAVE_0 = 0,                       /**< The SPI slave device is connected to the SPI master's CS0 pin. */
    HAL_SPI_MASTER_SLAVE_MAX                          /**< The total number of SPI master CS pins (invalid SPI master CS pin). */
} hal_spi_master_slave_port_t;

typedef enum {
    HAL_SPI_MASTER_CLOCK_POLARITY0 = 0,                     /**< Clock polarity is 0. */
    HAL_SPI_MASTER_CLOCK_POLARITY1 = 1                      /**< Clock polarity is 1. */
} hal_spi_master_clock_polarity_t;

typedef enum {
    HAL_SPI_MASTER_CLOCK_PHASE0 = 0,                         /**< Clock format is 0. */
    HAL_SPI_MASTER_CLOCK_PHASE1 = 1                          /**< Clock format is 1. */
} hal_spi_master_clock_phase_t;

typedef enum {
    HAL_SPI_MASTER_LSB_FIRST = 0,                       /**< Both send and receive data transfer LSB first. */
    HAL_SPI_MASTER_MSB_FIRST = 1                        /**< Both send and receive data transfer MSB first. */
} hal_spi_master_bit_order_t;

typedef struct {
    uint32_t clock_frequency;                                   /**< SPI master clock frequency setting. */
    hal_spi_master_slave_port_t slave_port;                     /**< SPI slave device selection. */
    hal_spi_master_bit_order_t bit_order;                       /**< SPI master bit order setting. */
    hal_spi_master_clock_polarity_t polarity;                   /**< SPI master clock polarity setting. */
    hal_spi_master_clock_phase_t phase;                         /**< SPI master clock phase setting. */
} hal_spi_master_config_t;
typedef struct {
    uint8_t *send_data;                               /**< Data buffer to send, this parameter cannot be NULL. */
    uint32_t send_length;                             /**< The number of bytes to send, no greater than 4. */
    uint8_t *receive_buffer;                          /**< Received data buffer, this parameter cannot be NULL. */
    uint32_t receive_length;                          /**< The valid number of bytes received with the number of bytes to send. */
} hal_spi_master_send_and_receive_config_t;
typedef enum {
    HAL_SPI_MASTER_EVENT_SEND_FINISHED      = 0,             /**< SPI master send operation completed. */
    HAL_SPI_MASTER_EVENT_RECEIVE_FINISHED   = 1              /**< SPI master receive operation completed. */
} hal_spi_master_callback_event_t;

typedef enum {
    HAL_SPI_MASTER_0 = 0,                              /**< SPI master port 0. */
    HAL_SPI_MASTER_1 = 1,                              /**< SPI master port 1. */
    HAL_SPI_MASTER_MAX                                 /**< The total number of SPI master ports (invalid SPI master port). */
} hal_spi_master_port_t;
#endif