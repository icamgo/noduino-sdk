#include "Arduino.h"
#include "sx126x.h"

#ifdef USE_SOFTSPI
#include "softspi.h"
#else
#include "spidrv.h"

#define SPI_M_USART1   	                                           \
{                                                                         \
  USART1,                       /* USART port                       */    \
  _USART_ROUTE_LOCATION_LOC0,   /* USART pins location number       */    \
  2000000,                      /* Bitrate                          */    \
  8,                            /* Frame length                     */    \
  0,                            /* Dummy tx value for rx only funcs */    \
  spidrvMaster,                 /* SPI mode                         */    \
  spidrvBitOrderMsbFirst,       /* Bit order on bus                 */    \
  spidrvClockMode0,             /* SPI clock/phase mode             */    \
  spidrvCsControlAuto,          /* CS controlled by the driver      */    \
  spidrvSlaveStartImmediate     /* Slave start transfers immediately*/    \
}

  //spidrvCsControlApplication,	/* CS controlled by the app         */    \
  //spidrvSlaveStartDelayed		\

SPIDRV_HandleData_t handle_data;
SPIDRV_Handle_t spi_hdl = &handle_data;

uint8_t spi_transfer(uint8_t data)
{
	uint8_t rx = 0;
	SPIDRV_MTransferSingleItemB(spi_hdl, data, &rx);
	return rx;
}
#endif

#define SX_1MS				(F_CPU/1000)		/* 14MHz, 1Tick = 1/14us, 14000tick = 1000us */
#define sx_delay(x)			do{for(int i=0;i<x*SX_1MS;i++) {asm volatile("nop");}}while(0)

#ifdef DEBUG_SX126X
#define INFO_S(param)			Serial.print(F(param))
#define INFO_HEX(param)			Serial.print(param,HEX)
#define INFOLN_HEX(param)		Serial.println(param,HEX)
#define INFO(param)				Serial.print(param)
#define INFOLN(param)			Serial.println(param)
#define FLUSHOUTPUT				Serial.flush();
#else
#define INFO_S(param)
#define INFO(param)
#define INFO_HEX(param)
#define INFOLN_HEX(param)
#define INFOLN(param)
#define FLUSHOUTPUT
#endif

SX126x::SX126x(int cs, int reset, int busy, int interrupt)
{
	_spi_cs = cs;
	_pin_reset = reset;
	_pin_busy = busy;
	_pin_dio1 = interrupt;

	_cad_on = false;

	tx_active = false;

	_ldro = true;

	_preamble_len = 8;

#ifdef USE_SOFTSPI
	pinMode(_spi_cs, OUTPUT);
#endif
	pinMode(_pin_reset, OUTPUT);
	pinMode(_pin_busy, INPUT);
	pinMode(_pin_dio1, INPUT);

	digitalWrite(_pin_reset, HIGH);

#ifdef USE_SOFTSPI
	spi_init(SW_CS, SW_SCK, SW_MOSI, SW_MISO);
#else
	SPIDRV_Init_t spi_init = SPI_M_USART1;
	SPIDRV_Init(spi_hdl, &spi_init);
#endif
}

int16_t SX126x::setup_v0(uint32_t freq_hz, int8_t dbm)
{
	_sf = SF10;
	_bw = BW500;
	_cr = CR46;
	_ldro = true;

	_tx_freq = freq_hz;

	if (dbm > 22)
		dbm = 22;

	if (dbm < -3)
		dbm = -3;

	_tx_power = dbm;

	set_stop_rx_timer_on_preamble(true);

	set_packet_type(SX126X_PACKET_TYPE_LORA);
	set_lora_symb_num_timeout(0);

	set_modulation_params(_sf, _bw, _cr, _ldro);
	set_rf_freq(_tx_freq);
	set_tx_power(_tx_power);

	// set_packet_params()
	pkt_params[2] = 0x00;	// Explicit header
	pkt_params[3] = 0xFF;	// pkt len
	write_op_cmd(SX126X_CMD_SET_PACKET_PARAMS, pkt_params, 6);
}

int16_t SX126x::setup_v1(uint32_t freq_hz, int8_t dbm)
{
	_sf = SF12;
	_bw = BW125;
	_cr = CR45;
	_ldro = true;

	_tx_freq = freq_hz;

	if (dbm > 22)
		dbm = 22;

	if (dbm < -3)
		dbm = -3;

	_tx_power = dbm;

	set_stop_rx_timer_on_preamble(false);

	set_packet_type(SX126X_PACKET_TYPE_LORA);
	set_lora_symb_num_timeout(0);

	set_modulation_params(_sf, _bw, _cr, _ldro);
	set_rf_freq(_tx_freq);
	set_tx_power(_tx_power);

	// set_packet_params()
	pkt_params[2] = 0x00;	// Explicit header
	pkt_params[3] = 0xFF;	// pkt len
	write_op_cmd(SX126X_CMD_SET_PACKET_PARAMS, pkt_params, 6);
}

int16_t SX126x::init()
{
	reset();

	pkt_params[0] = (_preamble_len>> 8) & 0xFF;
	pkt_params[1] = _preamble_len;

	pkt_params[2] = 0x00;	/* Explicit Header */
	pkt_params[3] = 0x30;	/* 48 Bytes payload len */

	pkt_params[4] = 0x01;	/* crc on */
	pkt_params[5] = 0x00;	/* no inverted iq */

	while (0x22 != get_status()) {
		set_standby(SX126X_STANDBY_RC);
		set_regulator_mode(SX126X_REGULATOR_DC_DC);
		delay(50);
	}

	get_status();
	workaround_ant_mismatch();

	/*
	 * ASR6500: 0x22
	 *  SX126x: 0x2A
	 */
	if (0x22 != get_status()) {
		INFOLN("SX126x error, maybe no SPI connection?");
		//return ERR_INVALID_MODE;
	}

	set_regulator_mode(SX126X_REGULATOR_DC_DC);
	//get_status();

	// convert from ms to SX126x time base
	set_dio3_as_tcxo_ctrl(SX126X_DIO3_OUTPUT_1_8, RADIO_TCXO_SETUP_TIME << 6);
	//set_dio3_as_tcxo_ctrl(SX126X_DIO3_OUTPUT_1_8, 320);				// 5ms

	//get_status();
	//delay(5);

	#if 1
	calibrate(SX126X_CALIBRATE_IMAGE_ON
		| SX126X_CALIBRATE_ADC_BULK_P_ON
		  | SX126X_CALIBRATE_ADC_BULK_N_ON
		  | SX126X_CALIBRATE_ADC_PULSE_ON
		  | SX126X_CALIBRATE_PLL_ON
		  | SX126X_CALIBRATE_RC13M_ON | SX126X_CALIBRATE_RC64K_ON);

	//get_status();
	#else
	calibrate(0x7f);

	get_dev_errors();
	clear_dev_errors();
	#endif

	set_dio2_as_rfswitch_ctrl(true);

	set_buffer_base_addr(0, 0);

	set_sync_word(0x1212);
	//set_sync_word(0x3444);	/* for public network */
	//set_sync_word(0x1424);	/* for private network */
}

void SX126x::workaround_ant_mismatch()
{
	/* 
	 * Better Resistance of the SX1262 Tx to Antenna Mismatch
	 * see DS_SX1261-2_V1.2 datasheet chapter 15.2
	 * RegTxClampConfig = @address 0x08D8
	 *
	 * The register modification must be done
	 * after a Power On Reset, or a wake-up
	 * from cold Start
	*/

	write_reg(0x08D8, read_reg(0x08D8) | 0x1E);
}

void SX126x::workaround_tx()
{
	/* 
	 * Before any packet transmission, bit #2 at address 0x0889 shall be
	 * set to:
	 *  •0 if the LoRa BW = 500 kHz
	 *  •1 for any other LoRa BW
	 *
	 * see DS_SX1261-2_V1.2 datasheet chapter 15.1
	 * 500KHz = @address 0x0889
	*/
	if (_bw == BW500) {
		write_reg(0x0889, read_reg(0x0889) & 0xFB);
	} else {
		write_reg(0x0889, read_reg(0x0889) & 0x04);
	}
}

int16_t SX126x::lora_config(uint8_t sf, uint8_t bw,
			   uint8_t cr, uint16_t preamble_len,
			   uint8_t pkt_len, bool crc_on, bool invert_iq)
{
	_sf = sf; _bw = bw; _cr = cr;

	//////////////////////////////////////////
	set_stop_rx_timer_on_preamble(false);
	set_lora_symb_num_timeout(0);

	set_packet_type(SX126X_PACKET_TYPE_LORA);

	set_modulation_params(_sf, _bw, _cr, _ldro);

#if 0
	pkt_params[0] = (preamble_len>> 8) & 0xFF;
	pkt_params[1] = preamble_len;

	if (pkt_len) {
		//fixed payload length
		pkt_params[2] = 0x01;
		pkt_params[3] = pkt_len;
	} else {
		pkt_params[2] = 0x00;
		pkt_params[3] = 0xFF;
	}

	if (crc_on)
		pkt_params[4] = 0x01;
	else
		pkt_params[4] = 0x00;

	if (invert_iq)
		pkt_params[5] = 0x01;
	else
		pkt_params[5] = 0x00;

	write_op_cmd(SX126X_CMD_SET_PACKET_PARAMS, pkt_params, 6);
#endif

/*
	config_dio_irq(SX126X_IRQ_ALL,	//all interrupts enabled
			(SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE, SX126X_IRQ_TIMEOUT),	//interrupts on DIO1
			SX126X_IRQ_NONE,	//interrupts on DIO2
			SX126X_IRQ_NONE);
*/

	set_rf_freq(_tx_freq);
	set_tx_power(_tx_power);

	set_modulation_params(_sf, _bw, _cr, _ldro);
}

/* ret:
 *  -1: rx_data_len > pkt_len
 *  -2: alloc failed
 *  -3: crc error
 *  -4: unknown interrupt
*/
int SX126x::get_rx_pkt(uint8_t *pkt, uint16_t len)
{
	int rx_len = 0;
	uint16_t irq = get_irq_status();

	if (false == (irq & SX126X_IRQ_CRC_ERR)) {

		if ((irq & SX126X_IRQ_RX_DONE) || (irq & SX126X_IRQ_TIMEOUT)) {

			/* ret:
			 *  -1: rx_data_len > pkt_len
			 *  -2: alloc failed
			*/
			rx_len = read_buf(pkt, len);

		} else {

			rx_len = -4;
		}

	} else {
		// crc error
		rx_len = -3;
	}

	clear_irq_status(SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERR | SX126X_IRQ_TIMEOUT);

	return rx_len;
}

int SX126x::rx(uint8_t *pkt, uint16_t len)
{
	int rx_len = 0;
	uint16_t irq = get_irq_status();

	if (irq & SX126X_IRQ_RX_DONE) {

		rx_len = read_buf(pkt, len);

		clear_irq_status(SX126X_IRQ_RX_DONE);
	}

	return rx_len;
}

void SX126x::enable_cad()
{
	_cad_on = true;
}

void SX126x::disable_cad()
{
	_cad_on = true;
}

int SX126x::send(uint8_t *data, uint8_t len, uint8_t mode)
{
	uint16_t irq;
	int rv = 1;

	if (tx_active == false) {

		tx_active = true;

		// set_packet_params()
		pkt_params[2] = 0x00;	// Explicit header
		pkt_params[3] = len;
		write_op_cmd(SX126X_CMD_SET_PACKET_PARAMS, pkt_params, 6);

		write_buf(data, len);

		workaround_tx();

		if (_cad_on) {
			carrier_sense();
		}

		config_dio_irq(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
						SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
						SX126X_IRQ_NONE,
						SX126X_IRQ_NONE);

		clear_irq_status(SX126X_IRQ_ALL);

		// timeout = 200ms
		set_tx(200);

		if (mode & SX126x_TXMODE_SYNC) {

			irq = get_irq_status();

			while ((!(irq & SX126X_IRQ_TX_DONE))
			       && (!(irq & SX126X_IRQ_TIMEOUT))) {
				irq = get_irq_status();
			}

			tx_active = false;

			if (irq == SX126X_IRQ_TX_DONE) {
				rv = 0;
			} else if (irq == SX126X_IRQ_TIMEOUT) {
				rv = 1;
			} else {
				rv = 2;
			}

		} else {
			rv = 0;
		}
	}

	if (rv == 0)
		INFOLN("TX OK");
	else
		INFOLN("TX failed");

	return rv;
}

void SX126x::clear_tx_active()
{
	tx_active = false;
}

bool SX126x::enter_rx(void)
{
	bool rv = false;

	if (tx_active == false) {

		config_dio_irq(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR,
						SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERR,
						SX126X_IRQ_NONE,
						SX126X_IRQ_NONE);

		clear_irq_status(SX126X_IRQ_ALL);

		set_rx(0xFFFFFF);

		rv = true;
	} else {

		rv = false;
	}

	return rv;
}

int SX126x::get_pkt_rssi()
{
	uint8_t buf[3];
	int rssi = 0;

	memset(buf, 0, 3);

	read_op_cmd(SX126X_CMD_GET_PACKET_STATUS, buf, 3);

	//(buf[1] < 128) ? (*snr = buf[1] >> 2) : (*snr = ((buf[1] - 256) >> 2));
	rssi = -buf[0] >> 1;
	return rssi;
}

void SX126x::reset(void)
{
	digitalWrite(_pin_reset, LOW);
	delay(50);
	digitalWrite(_pin_reset, HIGH);
	delay(20);
	while (digitalRead(_pin_busy)) ;
}

void SX126x::wakeup(void)
{
	get_status();
}

void SX126x::set_sleep()
{
	uint8_t cfg = SX126X_SLEEP_START_COLD | SX126X_SLEEP_RTC_OFF;
	write_op_cmd(SX126X_CMD_SET_SLEEP, &cfg, 0);

	//SPIDRV_DeInit(spi_hdl);
	//digitalWrite(_pin_reset, LOW);
}

void SX126x::set_standby(uint8_t mode)
{
	uint8_t data = mode;
	write_op_cmd(SX126X_CMD_SET_STANDBY, &data, 1);
}

uint8_t SX126x::get_status(void)
{
	uint8_t rv = 0xff, ret = 0;

#if 1
	ret = read_op_cmd(SX126X_CMD_GET_STATUS, &rv, 0);
#else
	while (digitalRead(_pin_busy)) ;

	#ifdef USE_SOFTSPI
	digitalWrite(_spi_cs, LOW);
	#endif

	spi_transfer(SX126X_CMD_GET_STATUS);
	ret = spi_transfer(0);

	#ifdef USE_SOFTSPI
	digitalWrite(_spi_cs, HIGH);
	#endif
#endif
	INFO("get_status: 0x");
	INFOLN_HEX(ret);
	return ret;
}

uint16_t SX126x::get_dev_errors(void)
{
	uint8_t error[2];

#if 1
	read_op_cmd(SX126X_CMD_GET_DEVICE_ERRORS, error, 2);
#else
	uint8_t rx[4] = {0};
	uint8_t cmd[4] = {0};
	cmd[0] = SX126X_CMD_GET_DEVICE_ERRORS;
	cmd[1] = 0; cmd[2] = 0; cmd[3] = 0;

	SPIDRV_MTransferB(spi_hdl, cmd, rx, 4);

	INFO("get_dev_errors: ");

	INFO_HEX(rx[2]);
	INFO(" ");

	error[0] = rx[2];
	error[1] = rx[3];
#endif

	INFO("0x");
	INFO_HEX(error[0]);
	INFOLN_HEX(error[1]);

	return (error[0] << 8) | error[1];
}

void SX126x::clear_dev_errors(void)
{
	uint8_t error[2] = {0, 0};

	write_op_cmd(SX126X_CMD_CLEAR_DEVICE_ERRORS, error, 2);
}

void SX126x::wait_on_busy(void)
{
	while (digitalRead(_pin_busy) == 1) ;
}

void SX126x::set_cad()
{
	write_op_cmd(SX126X_CMD_SET_CAD, NULL, 0);
}

void SX126x::set_cad_params(uint8_t sym_num, uint8_t det_pek, uint8_t det_min,
							uint8_t exit_mode, uint32_t timeout)
{
    uint8_t buf[6];

    buf[0] = sym_num;
    buf[1] = det_pek;
    buf[2] = det_min;
	buf[3] = exit_mode;
    buf[4] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[5] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[6] = (uint8_t)(timeout & 0xFF);

	write_op_cmd(SX126X_CMD_SET_CAD_PARAMS, buf, 7);
}

void SX126x::carrier_sense()
{
	int start = 0, end = 0;

	/*
	 * SF10: sym_num = 4, det_pek = 24, det_min = 10
	 * SF10: sym_num = 4, det_pek = 28, det_min = 10
	 */
	set_cad_params(0x2, 24, 10, SX126X_CAD_GOTO_STDBY, 0);

	config_dio_irq(SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_DETECTED,
					SX126X_IRQ_NONE,
					SX126X_IRQ_NONE,
					SX126X_IRQ_NONE);

	start = millis();
	end = start + 220;

	do {
		clear_irq_status(SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_DETECTED);

		set_cad();

		uint16_t irq = get_irq_status();
		while ((!(irq & SX126X_IRQ_CAD_DONE))
			   && (!(irq & SX126X_IRQ_CAD_DETECTED))) {
			irq = get_irq_status();
		}

		if ((irq & SX126X_IRQ_CAD_DETECTED) == false) {

			INFOLN("cad is not detected");
			return;

		} else if (irq & SX126X_IRQ_CAD_DONE) {
			INFOLN("cad is done");
		}

		start = millis();

	} while (start < end);
}

void SX126x::set_dio3_as_tcxo_ctrl(uint8_t volt, uint32_t timeout)
{
	uint8_t buf[4];

	buf[0] = volt & 0x07;
	buf[1] = (uint8_t) ((timeout >> 16) & 0xFF);
	buf[2] = (uint8_t) ((timeout >> 8) & 0xFF);
	buf[3] = (uint8_t) (timeout & 0xFF);

	write_op_cmd(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4);
}

void SX126x::calibrate(uint8_t calibParam)
{
	uint8_t data = calibParam;
	write_op_cmd(SX126X_CMD_CALIBRATE, &data, 1);
}

void SX126x::set_dio2_as_rfswitch_ctrl(uint8_t enable)
{
	uint8_t data = enable;
	write_op_cmd(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1);
}

void SX126x::set_rf_freq(uint32_t frequency)
{
	uint8_t buf[4];
	uint32_t freq = 0;

	calibrate_image(frequency);

	freq = (uint32_t) ((double)frequency / (double)FREQ_STEP);
	buf[0] = (uint8_t) ((freq >> 24) & 0xFF);
	buf[1] = (uint8_t) ((freq >> 16) & 0xFF);
	buf[2] = (uint8_t) ((freq >> 8) & 0xFF);
	buf[3] = (uint8_t) (freq & 0xFF);
	write_op_cmd(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);
}

void SX126x::calibrate_image(uint32_t frequency)
{
	uint8_t cal_freq[2];

	if (frequency > 900000000) {
		cal_freq[0] = 0xE1;
		cal_freq[1] = 0xE9;
	} else if (frequency > 850000000) {
		cal_freq[0] = 0xD7;
		cal_freq[1] = 0xD8;
	} else if (frequency > 770000000) {
		cal_freq[0] = 0xC1;
		cal_freq[1] = 0xC5;
	} else if (frequency > 460000000) {
		cal_freq[0] = 0x75;
		cal_freq[1] = 0x81;
	} else if (frequency > 425000000) {
		cal_freq[0] = 0x6B;
		cal_freq[1] = 0x6F;
	}
	write_op_cmd(SX126X_CMD_CALIBRATE_IMAGE, cal_freq, 2);
}

void SX126x::set_regulator_mode(uint8_t mode)
{
	uint8_t data = mode;
	write_op_cmd(SX126X_CMD_SET_REGULATOR_MODE, &data, 1);
}

void SX126x::set_buffer_base_addr(uint8_t tx_addr, uint8_t rx_addr)
{
	uint8_t buf[2];

	buf[0] = tx_addr;
	buf[1] = rx_addr;
	write_op_cmd(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);
}

void SX126x::set_sync_word(uint16_t syncw)
{
#if 0
	uint8_t buf[3];

	buf[0] = ((SX126X_REG_LORA_SYNCWORD_MSB & 0xFF00) >> 8);
	buf[1] = (SX126X_REG_LORA_SYNCWORD_MSB & 0x00FF);
	buf[2] = (syncw >> 8) & 0xFF;

	write_op_cmd(SX126X_CMD_WRITE_REGISTER, buf, 3);


	buf[0] = ((SX126X_REG_LORA_SYNCWORD_LSB & 0xFF00) >> 8);
	buf[1] = (SX126X_REG_LORA_SYNCWORD_LSB & 0x00FF);
	buf[2] = syncw & 0xFF;

	write_op_cmd(SX126X_CMD_WRITE_REGISTER, buf, 3);
#else
	write_reg(SX126X_REG_LORA_SYNCWORD_MSB, (syncw & 0xFF00) >> 8);
	write_reg(SX126X_REG_LORA_SYNCWORD_LSB, (syncw & 0xFF));
#endif
}

void SX126x::set_pa_config(uint8_t duty_cycle, uint8_t hp_max, uint8_t dev_sel,
			 uint8_t lut)
{
	uint8_t buf[4];

	buf[0] = duty_cycle;
	buf[1] = hp_max;
	buf[2] = dev_sel;
	buf[3] = lut;
	write_op_cmd(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}

void SX126x::set_over_current_protect(uint8_t value)
{
	uint8_t buf[3];

	buf[0] = ((SX126X_REG_OCP & 0xFF00) >> 8);
	buf[1] = (SX126X_REG_OCP & 0x00FF);
	buf[2] = value;
	write_op_cmd(SX126X_CMD_WRITE_REGISTER, buf, 3);
}

void SX126x::config_dio_irq(uint16_t irq_mask, uint16_t dio1_mask,
							 uint16_t dio2_mask, uint16_t dio3_mask)
{
	uint8_t buf[8];

	buf[0] = (uint8_t) ((irq_mask >> 8) & 0x00FF);
	buf[1] = (uint8_t) (irq_mask & 0x00FF);
	buf[2] = (uint8_t) ((dio1_mask >> 8) & 0x00FF);
	buf[3] = (uint8_t) (dio1_mask & 0x00FF);
	buf[4] = (uint8_t) ((dio2_mask >> 8) & 0x00FF);
	buf[5] = (uint8_t) (dio2_mask & 0x00FF);
	buf[6] = (uint8_t) ((dio3_mask >> 8) & 0x00FF);
	buf[7] = (uint8_t) (dio3_mask & 0x00FF);
	write_op_cmd(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}

void SX126x::set_stop_rx_timer_on_preamble(bool enable)
{
	uint8_t data = (uint8_t) enable;
	write_op_cmd(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1);
}

void SX126x::set_lora_symb_num_timeout(uint8_t symb_num)
{
	uint8_t data = symb_num;
	write_op_cmd(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1);
}

void SX126x::set_packet_type(uint8_t pkt_t)
{
	uint8_t data = pkt_t;
	write_op_cmd(SX126X_CMD_SET_PACKET_TYPE, &data, 1);
}

uint8_t SX126x::get_packet_type()
{
	uint8_t data = 0;
	read_op_cmd(SX126X_CMD_GET_PACKET_TYPE, &data, 1);
	return data;
}


void SX126x::set_modulation_params(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
	uint8_t data[4];
	//currently only LoRa supported
	data[0] = sf;
	data[1] = bw;
	data[2] = cr;
	data[3] = ldro;
	write_op_cmd(SX126X_CMD_SET_MODULATION_PARAMS, data, 4);
}

uint16_t SX126x::get_irq_status(void)
{
	uint8_t data[2];
	read_op_cmd(SX126X_CMD_GET_IRQ_STATUS, data, 2);
	return (data[0] << 8) | data[1];
}

void SX126x::clear_irq_status(uint16_t irq)
{
	uint8_t buf[2];

	buf[0] = (uint8_t) (((uint16_t) irq >> 8) & 0x00FF);
	buf[1] = (uint8_t) ((uint16_t) irq & 0x00FF);
	write_op_cmd(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);
}

void SX126x::set_rx(uint32_t timeout)
{
	uint8_t buf[3];

	buf[0] = (uint8_t) ((timeout >> 16) & 0xFF);
	buf[1] = (uint8_t) ((timeout >> 8) & 0xFF);
	buf[2] = (uint8_t) (timeout & 0xFF);
	write_op_cmd(SX126X_CMD_SET_RX, buf, 3);
}

void SX126x::set_tx(uint32_t timeoutInMs)
{
	uint8_t buf[3];
	uint32_t tout = (uint32_t) (timeoutInMs / 0.015625);
	buf[0] = (uint8_t) ((tout >> 16) & 0xFF);
	buf[1] = (uint8_t) ((tout >> 8) & 0xFF);
	buf[2] = (uint8_t) (tout & 0xFF);
	write_op_cmd(SX126X_CMD_SET_TX, buf, 3);
}

void SX126x::get_rxbuf_status(uint8_t *plen, uint8_t *rxbuf_start)
{
	uint8_t buf[2];

	read_op_cmd(SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 2);

	*plen = buf[0];
	*rxbuf_start = buf[1];
}

void SX126x::set_tx_power(int8_t dbm)
{
    uint8_t buf[2];

	// sx1262 or sx1268
	if (dbm > 22) {
		dbm = 22;
	} else if (dbm < -3) {
		dbm = -3;
	}

	if (dbm <= 14) {
		set_pa_config(0x02, 0x02, 0x00, 0x01);
	} else {
		set_pa_config(0x04, 0x07, 0x00, 0x01);
	}

	//set_over_current_protect(0x38);	// set max current to 140mA
	write_reg(SX126X_REG_OCP, 0x38); // current max 160mA for the whole device

    buf[0] = dbm;

    //if ( _crystal_select == 0) {
        // TCXO
        buf[1] = SX126X_PA_RAMP_200U;
    //} else {
        // XTAL
    //    buf[1] = RADIO_RAMP_20_US;
    //}

    write_op_cmd(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}

int SX126x::get_rssi()
{
    uint8_t buf;
    int rssi = 0;

    write_op_cmd(SX126X_CMD_GET_RSSI_INST, &buf, 1);
	rssi = -buf >> 1;

    return rssi;
}

void SX126x::test_tx()
{
	write_op_cmd(SX126X_CMD_SET_TX_CONTINUOUS_WAVE, NULL, 0);
}

void SX126x::test_tx_preamble()
{
	write_op_cmd(SX126X_CMD_SET_TX_INFINITE_PREAMBLE, NULL, 0);
}

/*
 * Reture value:
 *  >= 0: rxdata_len
 *    -1: rxdata_len > buf_len
 *    -2: alloc failed
*/
int SX126x::read_buf(uint8_t *data, uint8_t d_len)
{
	uint8_t offset = 0;
	uint8_t len = 0;

	get_rxbuf_status(&len, &offset);

	if (len > d_len) {
		return -1;
	}

	while (digitalRead(_pin_busy)) ;

#ifdef USE_SOFTSPI
	digitalWrite(_spi_cs, LOW);

	spi_transfer(SX126X_CMD_READ_BUFFER);
	spi_transfer(offset);
	spi_transfer(SX126X_CMD_NOP);

	for (uint16_t i = 0; i < len; i++) {
		data[i] = spi_transfer(SX126X_CMD_NOP);
	}

	digitalWrite(_spi_cs, HIGH);
#else

	uint8_t *ptx = (uint8_t *)malloc(len+3);

	if (ptx == NULL) {
		INFOLN("alloc failed");
		return -2;
	}

	memset(ptx, 0, len+3);

	ptx[0] = SX126X_CMD_READ_BUFFER;
	ptx[1] = offset;								/* offset */
	ptx[2] = SX126X_CMD_NOP;

	SPIDRV_MTransferB(spi_hdl, ptx, ptx, len+3);

	memcpy(data, ptx+3, len);

#ifdef DEBUG_SX126X_SPI
	for (uint16_t i = 0; i < len; i++) {
		INFO_HEX(ptx[i+3]);
		INFO(" ");
	}
	INFOLN("");
#endif

	free(ptx);
	ptx = NULL;

#endif

	while (digitalRead(_pin_busy)) ;

	return len;
}

uint8_t SX126x::write_buf(uint8_t *data, uint8_t len)
{
#ifdef DEBUG_SX126X_SPI
	INFO("SPI write: CMD=0x");
	INFO_HEX(SX126X_CMD_WRITE_BUFFER);
	INFO(" TX: ");
#endif

#ifdef USE_SOFTSPI
	digitalWrite(_spi_cs, LOW);

	spi_transfer(SX126X_CMD_WRITE_BUFFER);
	spi_transfer(0);	//offset in tx fifo

	for (uint16_t i = 0; i < len; i++) {
	#ifdef DEBUG_SX126X_SPI
		INFO_HEX(data[i]);
		INFO(" ");
	#endif
		spi_transfer(data[i]);
	}
	#ifdef DEBUG_SX126X_SPI
	INFOLN("");
	#endif

	digitalWrite(_spi_cs, HIGH);
#else

	uint8_t *ptx = (uint8_t *)malloc(len+2);

	if (ptx == NULL) {
		INFOLN("alloc failed");
		return;
	}

	memset(ptx, 0, len+2);

	ptx[0] = SX126X_CMD_WRITE_BUFFER;
	ptx[1] = 0;								/* offset */

	memcpy(ptx+2, data, len);

	SPIDRV_MTransmitB(spi_hdl, ptx, len+2);

#ifdef DEBUG_SX126X_SPI
	for (uint16_t i = 0; i < len; i++) {
		INFO_HEX(ptx[i+2]);
		INFO(" ");
	}
	INFOLN("");
#endif

	free(ptx);
	ptx = NULL;
#endif

	while (digitalRead(_pin_busy)) ;

	return 0;
}

uint8_t SX126x::read_reg(uint16_t addr)
{
	uint8_t ret = 0;
	read_reg(addr, &ret, 1);
	return ret;
}

void SX126x::read_reg(uint16_t addr, uint8_t *data, uint8_t size)
{
	// TODO timeout
	while (digitalRead(_pin_busy)) ;

#ifdef USE_SOFTSPI
	digitalWrite(_spi_cs, LOW);

	spi_transfer(SX126X_CMD_READ_REGISTER);

    spi_transfer((addr & 0xFF00) >> 8);
    spi_transfer(addr & 0x00FF);
	spi_transfer(0);

    for (int i = 0; i < size; i++) {
		data[i] = spi_transfer(0);
    }

	digitalWrite(_spi_cs, HIGH);
#else
	uint8_t *ptx = (uint8_t *)malloc(size+4);

	if (ptx == NULL) {
		INFOLN("alloc failed");
		return;
	}

	memset(ptx, 0, size+4);

	ptx[0] = SX126X_CMD_READ_REGISTER;
	ptx[1] = (addr >> 8) & 0xff;
	ptx[2] = addr & 0xff;
	ptx[3] = 0;

	SPIDRV_MTransferB(spi_hdl, ptx, ptx, size+4);

	memcpy(data, ptx+4, size);

	free(ptx);
	ptx = NULL;
#endif

	while (digitalRead(_pin_busy)) ;
}

void SX126x::write_reg(uint16_t addr, uint8_t data)
{
    write_reg(addr, &data, 1);
}

void SX126x::write_reg(uint16_t addr, uint8_t *data, uint8_t size)
{
	// TODO timeout
	while (digitalRead(_pin_busy)) ;

#ifdef USE_SOFTSPI
	digitalWrite(_spi_cs, LOW);

	spi_transfer(SX126X_CMD_WRITE_REGISTER);

    spi_transfer((addr & 0xFF00) >> 8);
    spi_transfer(addr & 0x00FF);

    for (int i = 0; i < size; i++) {
		spi_transfer(data[i]);
    }

	digitalWrite(_spi_cs, HIGH);
#else

	uint8_t *ptx = (uint8_t *)malloc(size+3);

	if (ptx == NULL) {
		INFOLN("alloc failed");
		return;
	}

	memset(ptx, 0, size+3);

	ptx[0] = SX126X_CMD_READ_REGISTER;
	ptx[1] = (addr >> 8) & 0xff;
	ptx[2] = addr & 0xff;

	memcpy(ptx+3, data, size);

	SPIDRV_MTransmitB(spi_hdl, ptx, size+3);

	free(ptx);
	ptx = NULL;
#endif

	while (digitalRead(_pin_busy)) ;
}

void SX126x::write_op_cmd(uint8_t cmd, uint8_t *data, uint8_t len)
{
	while (digitalRead(_pin_busy)) ;

#ifdef USE_SOFTSPI
	// start transfer
	digitalWrite(_spi_cs, LOW);

	spi_transfer(cmd);

#ifdef DEBUG_SX126X_SPI
	INFO("SPI write: CMD=0x");
	INFO_HEX(cmd);
	INFO(" TX: ");
#endif

	for (uint8_t n = 0; n < len; n++) {

		spi_transfer(data[n]);

#ifdef DEBUG_SX126X_SPI
		INFO_HEX(data[n]);
		INFO(" ");
#endif
	}

#ifdef DEBUG_SX126X_SPI
	INFOLN(" ");
#endif

	// stop transfer
	digitalWrite(_spi_cs, HIGH);
#else
	uint8_t tx[12] = {0};

	tx[0] = cmd;

	if (len > 0 && len <= 11) {
		memcpy(tx+1, data, len);
	}

	SPIDRV_MTransmitB(spi_hdl, tx, len+1);

#ifdef DEBUG_SX126X_SPI
	INFO("SPI write: CMD=0x");
	INFO_HEX(cmd);
	INFO(" TX: ");

	for (uint8_t n = 0; n < len; n++) {
		INFO_HEX(data[n]);
		INFO(" ");
	}
	INFOLN(" ");
#endif

	sx_delay(5);

#endif

	while (digitalRead(_pin_busy)) ;
}

/*
 * Max len is 6
 * Return the status of 2nd NOP
 */
uint8_t SX126x::read_op_cmd(uint8_t cmd, uint8_t *data, uint8_t len)
{
	while (digitalRead(_pin_busy)) ;

#ifdef USE_SOFTSPI
	uint8_t rx[2] = {0};

	// start transfer
	digitalWrite(_spi_cs, LOW);

	spi_transfer(cmd);
	spi_transfer(0);

#ifdef DEBUG_SX126X_SPI
	INFO("SPI read: CMD=0x");
	INFO_HEX(cmd);
	INFO(" RX: ");
#endif

	for (uint8_t i = 0; i < len; i++) {

		data[i] = spi_transfer(0);

#ifdef DEBUG_SX126X_SPI
		INFO_HEX(data[i]);
		INFO(" ");
#endif
	}

#ifdef DEBUG_SX126X_SPI
	INFOLN(" ");
#endif

	// stop transfer
	digitalWrite(_spi_cs, HIGH);
#else
	uint8_t tx[8] = {0};
	uint8_t rx[8] = {0};

	tx[0] = cmd;

	SPIDRV_MTransferB(spi_hdl, tx, rx, len+2);

	if (len > 0 && len <= 6) {
		memcpy(data, rx+2, len);
	}

#ifdef DEBUG_SX126X_SPI
	INFO("SPI read: CMD=0x");
	INFO_HEX(cmd);
	INFO(" RX: ");

	for (uint8_t i = 0; i < len; i++) {
		INFO_HEX(data[i]);
		INFO(" ");
	}
	INFOLN(" ");
#endif

#endif

	while (digitalRead(_pin_busy)) ;

	return rx[1];
}
