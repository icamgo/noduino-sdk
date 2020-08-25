#include "Arduino.h"
#include <SPI.h>
#include "SX126x.h"

SX126x::SX126x(int cs, int reset, int busy, int interrupt)
{
	SX126x_SPI_SELECT = cs;
	SX126x_RESET = reset;
	SX126x_BUSY = busy;
	SX126x_INT0 = interrupt;

	txActive = false;

	pinMode(SX126x_SPI_SELECT, OUTPUT);
	pinMode(SX126x_RESET, OUTPUT);
	pinMode(SX126x_BUSY, INPUT);
	pinMode(SX126x_INT0, INPUT);

	SPI.begin();
}

int16_t SX126x::begin(uint32_t freq_hz, int8_t dbm)
{
	if (dbm > 22)
		dbm = 22;
	if (dbm < -3)
		dbm = -3;

	_tx_power = dbm;

	reset();

	if (0x2A != get_status()) {
		Serial.println("SX126x error, maybe no SPI connection?");
		return ERR_INVALID_MODE;
	}
	SetStandby(SX126X_STANDBY_RC);

	get_status();
	get_dev_errors();

	SetPacketType(SX126X_PACKET_TYPE_LORA);

	get_status();
	get_dev_errors();

	// convert from ms to SX126x time base
	SetDio3AsTcxoCtrl(SX126X_DIO3_OUTPUT_3_3, RADIO_TCXO_SETUP_TIME << 6);

	get_status();
	get_dev_errors();

	SetRfFrequency(freq_hz);

	get_status();
	get_dev_errors();

	set_pa_config(0x04, 0x07, 0x00, 0x01);


	get_status();
	get_dev_errors();

	Calibrate(SX126X_CALIBRATE_IMAGE_ON
		| SX126X_CALIBRATE_ADC_BULK_P_ON
		  | SX126X_CALIBRATE_ADC_BULK_N_ON
		  | SX126X_CALIBRATE_ADC_PULSE_ON
		  | SX126X_CALIBRATE_PLL_ON
		  | SX126X_CALIBRATE_RC13M_ON | SX126X_CALIBRATE_RC64K_ON);

	get_status();
	get_dev_errors();

	SetDio2AsRfSwitchCtrl(true);

	get_status();
	get_dev_errors();

	SetStandby(SX126X_STANDBY_RC);

	get_dev_errors();

	SetRegulatorMode(SX126X_REGULATOR_DC_DC);

	get_dev_errors();

	SetBufferBaseAddress(0, 0);

	get_dev_errors();

	SetOvercurrentProtection(0x38);	// set max current to 140mA 

	get_dev_errors();

	//SetPowerConfig(dbm, SX126X_PA_RAMP_200U);	//0 fuer Empfaenger

	config_dio_irq(SX126X_IRQ_ALL,	//all interrupts enabled
			(SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT),	//interrupts on DIO1
			SX126X_IRQ_NONE,	//interrupts on DIO2
			SX126X_IRQ_NONE);	//interrupts on DIO3

	get_dev_errors();

	SetSyncWord(0x1212);

	get_dev_errors();

}

int16_t SX126x::LoRaConfig(uint8_t sf, uint8_t bw,
			   uint8_t cr, uint16_t preambleLength,
			   uint8_t payloadLen, bool crcOn, bool invertIrq)
{
	uint8_t ldro = 1;		//LowDataRateOptimize

	_sf = sf; _bw = bw; _cr = cr;

	SetStopRxTimerOnPreambleDetect(false);
	SetLoRaSymbNumTimeout(0);

	SetPacketType(SX126X_PACKET_TYPE_LORA);

	set_modulation_params(sf, bw, cr, ldro);

	PacketParams[0] = (preambleLength >> 8) & 0xFF;
	PacketParams[1] = preambleLength;
	if (payloadLen) {
		//fixed payload length
		PacketParams[2] = 0x01;
		PacketParams[3] = payloadLen;
	} else {
		PacketParams[2] = 0x00;
		PacketParams[3] = 0xFF;
	}

	if (crcOn)
		PacketParams[4] = 0x01;
	else
		PacketParams[4] = 0x00;

	if (invertIrq)
		PacketParams[5] = 0x01;
	else
		PacketParams[5] = 0x00;

	write_cmd(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6);

	config_dio_irq(SX126X_IRQ_ALL,	//all interrupts enabled
			(SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE, SX126X_IRQ_TIMEOUT),	//interrupts on DIO1
			SX126X_IRQ_NONE,	//interrupts on DIO2
			SX126X_IRQ_NONE);

	//receive state no receive timeoout
	//SetRx(0xFFFFFF);
}

uint8_t SX126x::Receive(uint8_t * pData, uint16_t len)
{
	uint8_t rxLen = 0;
	uint16_t irqRegs = GetIrqStatus();

	if (irqRegs & SX126X_IRQ_RX_DONE) {
		clear_irq_status(SX126X_IRQ_RX_DONE);
		ReadBuffer(pData, &rxLen, len);
	}

	return rxLen;
}

bool SX126x::send(uint8_t *data, uint8_t len, uint8_t mode)
{
	uint16_t irq;
	bool rv = false;

	if (txActive == false) {

		txActive = true;

		set_tx_power(_tx_power);

		config_dio_irq(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
						SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
						SX126X_IRQ_NONE,
						SX126X_IRQ_NONE);

		set_modulation_params(_sf, _bw, _cr, 1);


		// set_packet_params()
		PacketParams[2] = 0x00;	//Variable length packet (explicit header)
		PacketParams[3] = len;
		write_cmd(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6);

		clear_irq_status(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);

		write_buf(data, len);

		//SetTx(0);
		SetTx(100);		// timeout = 100ms

		if (mode & SX126x_TXMODE_SYNC) {
			irq = GetIrqStatus();
			while ((!(irq & SX126X_IRQ_TX_DONE))
			       && (!(irq & SX126X_IRQ_TIMEOUT))) {
				irq = GetIrqStatus();
			}
			txActive = false;

			SetRx(0xFFFFFF);

			if (irq != SX126X_IRQ_TIMEOUT)
				rv = true;
		} else {
			rv = true;
		}
	}

	return rv;
}

bool SX126x::ReceiveMode(void)
{
	uint16_t irq;
	bool rv = false;

	if (txActive == false) {
		rv = true;
	} else {
		irq = GetIrqStatus();
		if (irq & (SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT)) {
			SetRx(0xFFFFFF);
			txActive = false;
			rv = true;
		}
	}

	return rv;
}

void SX126x::ReceiveStatus(uint8_t * rssi, uint8_t * snr)
{
	uint8_t buf[3];

	read_cmd(SX126X_CMD_GET_PACKET_STATUS, buf, 3);

	(buf[1] < 128) ? (*snr = buf[1] >> 2) : (*snr = ((buf[1] - 256) >> 2));
	*rssi = -buf[0] >> 1;
}

void SX126x::SetTxPower(int8_t dbm)
{
	SetPowerConfig(dbm, SX126X_PA_RAMP_200U);
}

void SX126x::reset(void)
{
	delay(10);
	digitalWrite(SX126x_RESET, 0);
	delay(20);
	digitalWrite(SX126x_RESET, 1);
	delay(10);
	while (digitalRead(SX126x_BUSY)) ;
}

void SX126x::Wakeup(void)
{
	get_status();
}

void SX126x::SetStandby(uint8_t mode)
{
	uint8_t data = mode;
	write_cmd(SX126X_CMD_SET_STANDBY, &data, 1);
}

uint8_t SX126x::get_status(void)
{
	uint8_t rv = 0;
	read_cmd(SX126X_CMD_GET_STATUS, &rv, 1);
	Serial.print("get_status: 0x");
	Serial.println(rv, HEX);
	return rv;
}

uint16_t SX126x::get_dev_errors(void)
{
	uint16_t error;

	read_cmd(SX126X_CMD_GET_DEVICE_ERRORS, (uint8_t *)&error, 2);

	Serial.print("get_dev_errors: 0x");
	Serial.println(error, HEX);

	return error;
}

void SX126x::WaitOnBusy(void)
{
	while (digitalRead(SX126x_BUSY) == 1) ;
}

void SX126x::SetDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t timeout)
{
	uint8_t buf[4];

	buf[0] = tcxoVoltage & 0x07;
	buf[1] = (uint8_t) ((timeout >> 16) & 0xFF);
	buf[2] = (uint8_t) ((timeout >> 8) & 0xFF);
	buf[3] = (uint8_t) (timeout & 0xFF);

	write_cmd(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4);
}

void SX126x::Calibrate(uint8_t calibParam)
{
	uint8_t data = calibParam;
	write_cmd(SX126X_CMD_CALIBRATE, &data, 1);
}

void SX126x::SetDio2AsRfSwitchCtrl(uint8_t enable)
{
	uint8_t data = enable;
	write_cmd(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1);
}

void SX126x::SetRfFrequency(uint32_t frequency)
{
	uint8_t buf[4];
	uint32_t freq = 0;

	CalibrateImage(frequency);

	freq = (uint32_t) ((double)frequency / (double)FREQ_STEP);
	buf[0] = (uint8_t) ((freq >> 24) & 0xFF);
	buf[1] = (uint8_t) ((freq >> 16) & 0xFF);
	buf[2] = (uint8_t) ((freq >> 8) & 0xFF);
	buf[3] = (uint8_t) (freq & 0xFF);
	write_cmd(SX126X_CMD_SET_RF_FREQUENCY, buf, 4);
}

void SX126x::CalibrateImage(uint32_t frequency)
{
	uint8_t calFreq[2];

	if (frequency > 900000000) {
		calFreq[0] = 0xE1;
		calFreq[1] = 0xE9;
	} else if (frequency > 850000000) {
		calFreq[0] = 0xD7;
		calFreq[1] = 0xD8;
	} else if (frequency > 770000000) {
		calFreq[0] = 0xC1;
		calFreq[1] = 0xC5;
	} else if (frequency > 460000000) {
		calFreq[0] = 0x75;
		calFreq[1] = 0x81;
	} else if (frequency > 425000000) {
		calFreq[0] = 0x6B;
		calFreq[1] = 0x6F;
	}
	write_cmd(SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2);
}

void SX126x::SetRegulatorMode(uint8_t mode)
{
	uint8_t data = mode;
	write_cmd(SX126X_CMD_SET_REGULATOR_MODE, &data, 1);
}

void SX126x::SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
	uint8_t buf[2];

	buf[0] = txBaseAddress;
	buf[1] = rxBaseAddress;
	write_cmd(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);
}

void SX126x::SetSyncWord(uint16_t syncw)
{
	uint8_t buf[3];

	buf[0] = ((SX126X_REG_LORA_SYNC_WORD_MSB & 0xFF00) >> 8);
	buf[1] = (SX126X_REG_LORA_SYNC_WORD_MSB & 0x00FF);
	buf[2] = (syncw >> 8) & 0xFF;

	write_cmd(SX126X_CMD_WRITE_REGISTER, buf, 3);


	buf[0] = ((SX126X_REG_LORA_SYNC_WORD_LSB & 0xFF00) >> 8);
	buf[1] = (SX126X_REG_LORA_SYNC_WORD_LSB & 0x00FF);
	buf[2] = syncw & 0xFF;

	write_cmd(SX126X_CMD_WRITE_REGISTER, buf, 3);
}

void SX126x::SetPowerConfig(int8_t power, uint8_t rampTime)
{
	uint8_t buf[2];

	if (power > 22) {
		power = 22;
	} else if (power < -3) {
		power = -3;
	}

	buf[0] = power;
	buf[1] = (uint8_t) rampTime;
	write_cmd(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}

void SX126x::set_pa_config(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel,
			 uint8_t paLut)
{
	uint8_t buf[4];

	buf[0] = paDutyCycle;
	buf[1] = hpMax;
	buf[2] = deviceSel;
	buf[3] = paLut;
	write_cmd(SX126X_CMD_SET_PA_CONFIG, buf, 4);
}

void SX126x::SetOvercurrentProtection(uint8_t value)
{
	uint8_t buf[3];

	buf[0] = ((SX126X_REG_OCP_CONFIGURATION & 0xFF00) >> 8);
	buf[1] = (SX126X_REG_OCP_CONFIGURATION & 0x00FF);
	buf[2] = value;
	write_cmd(SX126X_CMD_WRITE_REGISTER, buf, 3);
}

void SX126x::config_dio_irq(uint16_t irqMask, uint16_t dio1Mask,
			     uint16_t dio2Mask, uint16_t dio3Mask)
{
	uint8_t buf[8];

	buf[0] = (uint8_t) ((irqMask >> 8) & 0x00FF);
	buf[1] = (uint8_t) (irqMask & 0x00FF);
	buf[2] = (uint8_t) ((dio1Mask >> 8) & 0x00FF);
	buf[3] = (uint8_t) (dio1Mask & 0x00FF);
	buf[4] = (uint8_t) ((dio2Mask >> 8) & 0x00FF);
	buf[5] = (uint8_t) (dio2Mask & 0x00FF);
	buf[6] = (uint8_t) ((dio3Mask >> 8) & 0x00FF);
	buf[7] = (uint8_t) (dio3Mask & 0x00FF);
	write_cmd(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}

void SX126x::SetStopRxTimerOnPreambleDetect(bool enable)
{
	uint8_t data = (uint8_t) enable;
	write_cmd(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1);
}

void SX126x::SetLoRaSymbNumTimeout(uint8_t SymbNum)
{
	uint8_t data = SymbNum;
	write_cmd(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1);
}

void SX126x::SetPacketType(uint8_t packetType)
{
	uint8_t data = packetType;
	write_cmd(SX126X_CMD_SET_PACKET_TYPE, &data, 1);
}

void SX126x::set_modulation_params(uint8_t sf, uint8_t bw,
				 uint8_t cr,
				 uint8_t lowDataRateOptimize)
{
	uint8_t data[4];
	//currently only LoRa supported
	data[0] = sf;
	data[1] = bw;
	data[2] = cr;
	data[3] = lowDataRateOptimize;
	write_cmd(SX126X_CMD_SET_MODULATION_PARAMS, data, 4);
}

uint16_t SX126x::GetIrqStatus(void)
{
	uint8_t data[2];
	read_cmd(SX126X_CMD_GET_IRQ_STATUS, data, 2);
	return (data[0] << 8) | data[1];
}

void SX126x::clear_irq_status(uint16_t irq)
{
	uint8_t buf[2];

	buf[0] = (uint8_t) (((uint16_t) irq >> 8) & 0x00FF);
	buf[1] = (uint8_t) ((uint16_t) irq & 0x00FF);
	write_cmd(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2);
}

void SX126x::SetRx(uint32_t timeout)
{
	uint8_t buf[3];

	buf[0] = (uint8_t) ((timeout >> 16) & 0xFF);
	buf[1] = (uint8_t) ((timeout >> 8) & 0xFF);
	buf[2] = (uint8_t) (timeout & 0xFF);
	write_cmd(SX126X_CMD_SET_RX, buf, 3);
}

void SX126x::SetTx(uint32_t timeoutInMs)
{
	uint8_t buf[3];
	uint32_t tout = (uint32_t) (timeoutInMs / 0.015625);
	buf[0] = (uint8_t) ((tout >> 16) & 0xFF);
	buf[1] = (uint8_t) ((tout >> 8) & 0xFF);
	buf[2] = (uint8_t) (tout & 0xFF);
	write_cmd(SX126X_CMD_SET_TX, buf, 3);
}

void SX126x::GetRxBufferStatus(uint8_t * payloadLength,
			       uint8_t * rxStartBufferPointer)
{
	uint8_t buf[2];

	read_cmd(SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 2);

	*payloadLength = buf[0];
	*rxStartBufferPointer = buf[1];
}

uint8_t SX126x::ReadBuffer(uint8_t * rxData, uint8_t * rxDataLen,
			   uint8_t maxLen)
{
	uint8_t offset = 0;

	GetRxBufferStatus(rxDataLen, &offset);
	if (*rxDataLen > maxLen) {
		return 1;
	}

	while (digitalRead(SX126x_BUSY)) ;

	digitalWrite(SX126x_SPI_SELECT, LOW);
	SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
	SPI.transfer(SX126X_CMD_READ_BUFFER);
	SPI.transfer(offset);
	SPI.transfer(SX126X_CMD_NOP);
	for (uint16_t i = 0; i < *rxDataLen; i++) {
		rxData[i] = SPI.transfer(SX126X_CMD_NOP);
	}
	digitalWrite(SX126x_SPI_SELECT, HIGH);

	while (digitalRead(SX126x_BUSY)) ;

	return 0;
}

uint8_t SX126x::write_buf(uint8_t *data, uint8_t len)
{
	//Serial.print("SPI write: CMD=0x");
	//Serial.print(SX126X_CMD_WRITE_BUFFER, HEX);
	//Serial.print(" DataOut: ");
	digitalWrite(SX126x_SPI_SELECT, LOW);
	SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

	SPI.transfer(SX126X_CMD_WRITE_BUFFER);
	SPI.transfer(0);	//offset in tx fifo

	//Serial.print(" 0 ");
	for (uint16_t i = 0; i < len; i++) {
		//Serial.print(data[i]);
		//Serial.print(" ");
		SPI.transfer(data[i]);
	}

	digitalWrite(SX126x_SPI_SELECT, HIGH);
	//Serial.println("");

	while (digitalRead(SX126x_BUSY)) ;

	return 0;
}

void SX126x::write_cmd(uint8_t cmd, uint8_t *data, uint8_t len,
			     bool waitForBusy)
{
	SPItransfer(cmd, true, data, NULL, len, waitForBusy);
}

void SX126x::read_cmd(uint8_t cmd, uint8_t *data, uint8_t len,
			    bool waitForBusy)
{
	SPItransfer(cmd, false, NULL, data, len, waitForBusy);
}

void SX126x::SPItransfer(uint8_t cmd, bool write, uint8_t * dataOut,
			 uint8_t * dataIn, uint8_t len, bool waitForBusy)
{

	// ensure BUSY is low (state meachine ready)
	// TODO timeout
	while (digitalRead(SX126x_BUSY)) ;

	// start transfer
	digitalWrite(SX126x_SPI_SELECT, LOW);
	SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

	// send command byte
	SPI.transfer(cmd);

	// send/receive all bytes
	if (write) {
		Serial.print("SPI write: CMD=0x");
		Serial.print(cmd, HEX);
		Serial.print(" DataOut: ");
		for (uint8_t n = 0; n < len; n++) {
			uint8_t in = SPI.transfer(dataOut[n]);
			Serial.print(dataOut[n], HEX);
			Serial.print(" ");
		}
		Serial.println();
	} else {
		Serial.print("SPI read:  CMD=0x");
		Serial.print(cmd, HEX);
		// skip the first byte for read-type commands (status-only)
		uint8_t in = SPI.transfer(SX126X_CMD_NOP);

		//Serial.println((SX126X_CMD_NOP, HEX));
		Serial.print(" DataIn: ");

		for (uint8_t n = 0; n < len; n++) {
			dataIn[n] = SPI.transfer(SX126X_CMD_NOP);
			//Serial.println((SX126X_CMD_NOP, HEX));
			Serial.print(dataIn[n], HEX);
			Serial.print(" ");
		}
		//Serial.println();
	}

	// stop transfer
	SPI.endTransaction();
	digitalWrite(SX126x_SPI_SELECT, HIGH);

	// wait for BUSY to go high and then low
	// TODO timeout
	if (waitForBusy) {
		delayMicroseconds(1);
		while (digitalRead(SX126x_BUSY)) ;
	}
}

void SX126x::write_reg(uint16_t addr, uint8_t data)
{
    write_reg(addr, &data, 1);
}

void SX126x::write_reg(uint16_t addr, uint8_t *data, uint8_t size)
{
	// TODO timeout
	while (digitalRead(SX126x_BUSY)) ;

	digitalWrite(SX126x_SPI_SELECT, LOW);
	SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

	SPI.transfer(SX126X_CMD_WRITE_REGISTER);

    SPI.transfer((addr & 0xFF00) >> 8);
    SPI.transfer(addr & 0x00FF);

    for (int i = 0; i < size; i++) {
		SPI.transfer(data[i]);
    }

	SPI.endTransaction();
	digitalWrite(SX126x_SPI_SELECT, HIGH);
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

	write_reg(SX126X_REG_OCP_CONFIGURATION, 0x38); // current max 160mA for the whole device

    buf[0] = dbm;

    //if ( _crystal_select == 0) {
        // TCXO
        buf[1] = SX126X_PA_RAMP_200U;
    //} else {
        // XTAL
    //    buf[1] = RADIO_RAMP_20_US;
    //}

    write_cmd(SX126X_CMD_SET_TX_PARAMS, buf, 2);
}
