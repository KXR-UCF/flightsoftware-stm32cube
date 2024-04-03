#include <AD7124.h>
#include <stm32f4xx_hal.h>

int8_t ad7124_noCheckReadRegister(struct ad7124_dev *dev,
		struct ad7124_registerData *p_reg) {
	int8_t flag = 0, i = 0, check8 = 0;
	uint8_t txBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t rxBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	// Build Command word
	txBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD
			| AD7124_COMM_REG_RA(p_reg->addr);

	/*
	 * If this is an AD7124_DATA register read, and the DATA_STATUS bit is set
	 * in ADC_CONTROL, need to read 4, not 3 bytes for DATA with flag
	 */
	if ((p_reg->addr == AD7124_DATA_REG)
			&& (dev->regs[AD7124_ADC_Control].value
					& AD7124_ADC_CTRL_REG_DATA_STATUS))
		check8 = 1;

	uint16_t length = (
			(dev->useCRC != AD7124_DISABLE_CRC) ? p_reg->size + 1 : p_reg->size);

	HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_RESET);
	flag = (HAL_SPI_TransmitReceive(dev->hspi, &txBuf[0], &rxBuf[0], length,
	HAL_MAX_DELAY) != HAL_OK);
	HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_SET);

	if (flag)
		return 1;

	// Check the CRC
	if (dev->useCRC == AD7124_USE_CRC) {
		txBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD
				| AD7124_COMM_REG_RA(p_reg->addr);
		for (i = 1; i < p_reg->size + 2; ++i)
			txBuf[i] = rxBuf[i];
		flag = ad7124_computeCrc8(txBuf, p_reg->size + 2);
	}

	if (check8 != 0) {
		/* readRegister checksum failed. */
		return -2;
	}

	if (flag)
		return 1;

	// Build the result
	p_reg->value = 0;
	for (i = 1; i < p_reg->size + 1; i++) {
		p_reg->value <<= 8;
		p_reg->value += rxBuf[i];
	}

	return 0;
}

int8_t ad7124_noCheckWriteRegister(struct ad7124_dev *dev,
		struct ad7124_registerData p_reg) {
	int32_t regValue = 0;
	int8_t flag = 0, i = 0, crc8 = 0;
	uint8_t txBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	// Build Command word
	txBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD
			| AD7124_COMM_REG_RA(p_reg.addr);

	// Fill the write buffer
	regValue = p_reg.value;
	for (i = 0; i < p_reg.size; i++) {
		txBuf[p_reg.size - i] = regValue & 0xFF;
		regValue >>= 8;
	}

	// Compute the CRC
	if (dev->useCRC != AD7124_DISABLE_CRC) {
		crc8 = ad7124_computeCrc8(txBuf, p_reg.size + 1);
		txBuf[p_reg.size + 1] = crc8;
	}

	uint16_t length = ((
			(dev->useCRC != AD7124_DISABLE_CRC) ? p_reg.size + 1 : p_reg.size)
			+ 1);

	HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_RESET);
	flag =
			(HAL_SPI_Transmit(dev->hspi, txBuf, length, HAL_MAX_DELAY) != HAL_OK);
	HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_SET);

	return flag;
}

int8_t ad7124_readRegister(struct ad7124_dev *dev,
		struct ad7124_registerData *p_reg) {
	int8_t flag = 0;

	if (p_reg->addr != AD7124_ERR_REG && dev->isReady) {
		flag = ad7124_waitForSpiReady(dev, dev->responseTimeout);
		if (flag)
			return 1;
	}

	return ad7124_noCheckReadRegister(dev, p_reg);
}

int8_t ad7124_writeRegister(struct ad7124_dev *dev,
		struct ad7124_registerData p_reg) {
	int8_t flag = 0;

	if (dev->isReady) {
		flag = ad7124_waitForSpiReady(dev, dev->responseTimeout);
		if (flag)
			return 1;
	}

	return ad7124_noCheckWriteRegister(dev, p_reg);
}

int8_t ad7124_reset(struct ad7124_dev *dev) {
	int8_t flag = 0;
	uint8_t txBuf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_RESET);
	flag = (HAL_SPI_Transmit(dev->hspi, txBuf, 8, HAL_MAX_DELAY) != HAL_OK);
	HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_SET);

	if (flag)
		return 1;

	/* Read POR bit to clear */
	flag = ad7124_waitToPowerOn(dev, dev->responseTimeout);
	if (flag)
		return 1;

	// Recommened 4ms delay
	HAL_Delay(4);

	return 0;
}

// Waits until the device can accept read and write user actions
int8_t ad7124_waitForSpiReady(struct ad7124_dev *dev, uint32_t timeout) {
	struct ad7124_registerData *regs;
	int8_t flag;
	int8_t ready = 0;

	regs = dev->regs;

	while (!ready && --timeout) {
		// read the value of the error registers
		flag = ad7124_readRegister(dev, &regs[AD7124_Error]);
		if (flag)
			return 1;

		ready = (regs[AD7124_Error].value & AD7124_ERR_REG_SPI_IGNORE_ERR) == 0;
	}

	if (!timeout)
		return 1;

	return 0;
}

// Waits until the device finishes the power-on reset operations
int8_t ad7124_waitToPowerOn(struct ad7124_dev *dev, uint32_t timeout) {
	struct ad7124_registerData *regs;
	int32_t flag = 0;
	int8_t powered_on = 0;

	regs = dev->regs;

	while (!powered_on && --timeout) {
		flag = ad7124_readRegister(dev, &regs[AD7124_Status]);

		if (flag)
			return 1;

		// Check the POR_FLAG bit in the flag Register
		powered_on = (regs[AD7124_Status].value & AD7124_STATUS_REG_POR_FLAG)
				== 0;
	}

	if (!(timeout || powered_on))
		return 1;
	return 0;
}

// Waits until a new conversion result is available
int8_t ad7124_WaitForConvReady(struct ad7124_dev *dev, uint32_t timeout) {
	struct ad7124_registerData *regs;
	int8_t flag, ready = 0;

	regs = dev->regs;

	while (!ready && --timeout) {
		// Read d the value of the flag register
		flag = ad7124_readRegister(dev, &regs[AD7124_Status]);
		if (flag)
			return 1;
		// Check the RDY bit in the flag register
		ready = (regs[AD7124_Status].value & AD7124_STATUS_REG_RDY == 0);
	}

	if (!timeout)
		return 1;

	return 0;
}

// Reads the conversion result from the device
int8_t ad7124_readData(struct ad7124_dev *dev, int32_t *p_data) {
	struct ad7124_registerData *regs;
	int8_t flag;

	regs = dev->regs;

	// Read the value of the data register
	flag = ad7124_readRegister(dev, &regs[AD7124_Data]);

	*p_data = regs[AD7124_Data].value;

	return flag;
}

// Computes the CRC checksum for a data buffer
int8_t ad7124_computeCrc8(uint8_t *p_buf, uint8_t buf_size) {
	uint8_t i = 0;
	uint8_t crc = 0;

	while (buf_size) {
		for (i = 0x80; i != 0; i >>= 1) {
			uint8_t cmp1 = (crc & 0x80) != 0;
			uint8_t cmp2 = (*p_buf & i) != 0;
			if (cmp1 != cmp2) {
				/* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
			} else {
				crc <<= 1;
			}
		}
		p_buf++;
		buf_size--;
	}

	return crc;
}

// Updates the CRC settings.
void ad7124_updateCrcSetting(struct ad7124_dev *dev) {
	struct ad7124_registerData *regs;

	if (!dev)
		return;

	regs = dev->regs;

	/* Get CRC State. */
	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_CRC_ERR_EN)
		dev->useCRC = AD7124_USE_CRC;
	else
		dev->useCRC = AD7124_DISABLE_CRC;
}

// Updates the device SPI interface settings
void ad7124_updateDevSpiSettings(struct ad7124_dev *dev) {
	struct ad7124_registerData *regs;

	if (!dev)
		return;

	regs = dev->regs;

	if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_IGNORE_ERR_EN)
		dev->isReady = 1;
	else
		dev->isReady = 0;
}

// ADC Fucntions

int8_t ad7124_setAdcControl(struct ad7124_dev *dev,
		enum ad7124_mode operating_mode, enum ad7124_powerMode power_mode,
		uint8_t ref_en, uint8_t clk_sel) {
	struct ad7124_registerData *r = &dev->regs[AD7124_ADC_Control];

	r->value = AD7124_ADC_CTRL_REG_MODE(operating_mode) |
	AD7124_ADC_CTRL_REG_POWER_MODE(power_mode) |
	AD7124_ADC_CTRL_REG_CLK_SEL(clk_sel) |
	(ref_en ? AD7124_ADC_CTRL_REG_REF_EN : 0) |
	AD7124_ADC_CTRL_REG_DOUT_RDY_DEL;

	return ad7124_writeRegister(dev, *r);
}

int8_t ad7124_setConfig(struct ad7124_dev *dev, uint8_t cfg,
		enum ad7124_reference_source ref, enum ad7124_PgaSel pga,
		int8_t bipolar, enum ad7124_BurnoutCurrent burnout) {
	if (cfg < 8) {
		struct ad7124_registerData *r;

		cfg += AD7124_Config_0;
		r = &dev->regs[cfg];

		r->value = AD7124_CFG_REG_REF_SEL(ref) |
		AD7124_CFG_REG_PGA(pga) |
		(bipolar ? AD7124_CFG_REG_BIPOLAR : 0) |
		AD7124_CFG_REG_BURNOUT(burnout) |
		AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_REF_BUFM |
		AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM;
		return ad7124_writeRegister(dev, *r);
	}
	return -1;
}

int8_t ad7124_setChannel(struct ad7124_dev *dev, uint8_t ch, uint8_t cfg,
		enum ad7124_analog_input ainp, enum ad7124_analog_input ainm,
		uint8_t enable) {
	if ((ch < 16) && (cfg < 8)) {
		struct ad7124_registerData *r;

		ch += AD7124_Channel_0;
		r = &dev->regs[ch];

		r->value = AD7124_CH_MAP_REG_SETUP(cfg) | AD7124_CH_MAP_REG_AINP(ainp)
				| AD7124_CH_MAP_REG_AINM(ainm)
				| (enable ? AD7124_CH_MAP_REG_CH_ENABLE : 0);

		return ad7124_writeRegister(dev, *r);
	}
	return -1;
}

int8_t ad7124_getRegister(struct ad7124_dev *dev, enum ad7124_registers id) {
	int8_t flag = 0;
	if (id < AD7124_Status && id >= 57)
		return -1;

	flag = ad7124_readRegister(dev, &dev->regs[id]);
	if (flag != 0) {
		return -1;
	}

	return dev->regs[id].value;
}

uint8_t ad7124_currentChannel(struct ad7124_dev *dev) {
	int8_t flag = ad7124_getRegister(dev, AD7124_Status);

	if (flag < 0) {
		return 1;
	}
	return (uint8_t) (flag & AD7124_STATUS_REG_CH_ACTIVE(15));
}

uint8_t ad7124_enableChannel(struct ad7124_dev *dev, uint8_t ch, uint8_t enable) {
	if (ch > 16)
		return -1;

	struct ad7124_registerData *r;
	uint8_t flag, chan = AD7124_Channel_0;
	chan += ch;
	r = &dev->regs[chan];
	flag = ad7124_readRegister(dev, r);
	if (flag < 0) {
		return 7;
	}

	if (enable) {

		r->value |= AD7124_CH_MAP_REG_CH_ENABLE;
	} else {

		r->value &= ~AD7124_CH_MAP_REG_CH_ENABLE;
	}

	return ad7124_writeRegister(dev, *r);
}

int32_t ad7124_getData(struct ad7124_dev *dev) {
	int32_t value;
	int8_t ret;

	ret = ad7124_readData(dev, &value);
	if (ret < 0) {
		return ret;
	}
	return (int32_t) value;
}

int8_t ad7124_startSingleConversion(struct ad7124_dev *dev, uint8_t ch) {
	if (ch < 16) {
		int ret;

		ret = ad7124_enableChannel(dev, ch, 1);
		if (ret < 0) {
			return ret;
		}
		return ad7124_setMode(dev, AD7124_SINGLE);
	}
	return -1;
}

int32_t ad7124_adcRead(struct ad7124_dev *dev, uint8_t ch) {
	HAL_Delay(50);
	int8_t flag = 0;
	uint8_t cur_ch = ad7124_currentChannel(dev);

	if (ch != cur_ch) {

		// disable previous channel if different
		flag = ad7124_enableChannel(dev, cur_ch, 0);
		if (flag != 0)
			return 1;
	}

	flag = ad7124_startSingleConversion(dev, ch);
	if (flag != 0) {
		return flag;
	}

	flag = ad7124_waitEndOfConversion(dev);
	if (flag != 0)
		return flag;

	return ad7124_getData(dev);
}

int8_t ad7124_waitEndOfConversion(struct ad7124_dev *dev) {
	int8_t ret;
	uint8_t ready = 0;
	uint32_t timeout = 4000;

	do {

		/* Read the value of the Status Register */
		ret = ad7124_readRegister(dev, &dev->regs[AD7124_STATUS_REG]);
		if (ret != 0) {

			return 4;
		}

		/* Check the RDY bit in the Status Register */
		ready = (dev->regs[AD7124_STATUS_REG].value &
		AD7124_STATUS_REG_RDY) == 0;

		HAL_Delay(4);
	} while (!ready && --timeout);

	return (timeout == 0) ? 5 : 0;
}

float ad7124_toVoltage(int32_t value, int16_t gain, float vref, uint8_t bipolar) {
	float voltage = (float) value;

	if (bipolar) {
		voltage = voltage / (float) 0x7FFFFFU - 1;
	} else {
		voltage = voltage / (float) 0xFFFFFFU;
	}

	voltage = voltage * vref / (float) gain;
	return voltage;
}

int8_t ad7124_setMode(struct ad7124_dev *dev, enum ad7124_mode operating_mode) {
	struct ad7124_registerData *r;
	r = &dev->regs[AD7124_ADC_Control];
	r->value &= ~AD7124_ADC_CTRL_REG_MODE(0x0F); // clear mode
	r->value |= AD7124_ADC_CTRL_REG_MODE(operating_mode);
	return ad7124_writeRegister(dev, dev->regs[AD7124_ADC_Control]);
}

// Initializes the AD7124.
int8_t ad7124_init(struct ad7124_dev *device, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *ncs_GPIOx, uint16_t ncs_GPIO_pin) {
	struct ad7124_registerData ad7124_init_regs_default[AD7124_REG_NO] = { {
			0x00, 0x00, 1, 2 }, /* AD7124_Status */
	/*
	 * 7:6 - 11 : Full power mode
	 * 8 - 1: Internale ref enabled
	 * 1:0 - 00 Internal 614.4 clock
	 */
	{ 0x01, 0x05C0, 2, 1 }, /* AD7124_ADC_Control */
	{ 0x02, 0x0000, 3, 2 }, /* AD7124_Data */
	{ 0x03, 0x0000, 3, 1 }, /* AD7124_IOCon1 */
	{ 0x04, 0x0000, 2, 1 }, /* AD7124_IOCon2 */
	{ 0x05, 0x02, 1, 2 }, /* AD7124_ID */
	{ 0x06, 0x0000, 3, 2 }, /* AD7124_Error */
	{ 0x07, 0x0000, 3, 1 }, /* AD7124_Error_En */
	{ 0x08, 0x00, 1, 2 }, /* AD7124_Mclk_Count */
	{ 0x09, 0x0001, 2, 1 }, /* AD7124_Channel_0 */
	{ 0x0A, 0x0043, 2, 1 }, /* AD7124_Channel_1 */
	{ 0x0B, 0x0043, 2, 1 }, /* AD7124_Channel_2 */
	{ 0x0C, 0x00C7, 2, 1 }, /* AD7124_Channel_3 */
	{ 0x0D, 0x0085, 2, 1 }, /* AD7124_Channel_4 */
	{ 0x0E, 0x014B, 2, 1 }, /* AD7124_Channel_5 */
	{ 0x0F, 0x00C7, 2, 1 }, /* AD7124_Channel_6 */
	{ 0x10, 0x01CF, 2, 1 }, /* AD7124_Channel_7 */
	{ 0x11, 0x0109, 2, 1 }, /* AD7124_Channel_8 */
	{ 0x12, 0x0031, 2, 1 }, /* AD7124_Channel_9 */
	{ 0x13, 0x014B, 2, 1 }, /* AD7124_Channel_10 */
	{ 0x14, 0x0071, 2, 1 }, /* AD7124_Channel_11 */
	{ 0x15, 0x018D, 2, 1 }, /* AD7124_Channel_12 */
	{ 0x16, 0x00B1, 2, 1 }, /* AD7124_Channel_13 */
	{ 0x17, 0x01CF, 2, 1 }, /* AD7124_Channel_14 */
	{ 0x18, 0x00F1, 2, 1 }, /* AD7124_Channel_15 */
	{ 0x19, 0x0980, 2, 1 }, /* AD7124_Config_0 */
	{ 0x1A, 0x0981, 2, 1 }, /* AD7124_Config_1 */
	{ 0x1B, 0x0982, 2, 1 }, /* AD7124_Config_2 */
	{ 0x1C, 0x0983, 2, 1 }, /* AD7124_Config_3 */
	{ 0x1D, 0x0984, 2, 1 }, /* AD7124_Config_4 */
	{ 0x1E, 0x0985, 2, 1 }, /* AD7124_Config_5 */
	{ 0x1F, 0x0986, 2, 1 }, /* AD7124_Config_6 */
	{ 0x20, 0x0987, 2, 1 }, /* AD7124_Config_7 */
	/*
	 * 10:0 - 1 - FS = 1 from filter 0 to 7
	 */
	{ 0x21, 0x060001, 3, 1 }, /* AD7124_Filter_0 */
	{ 0x22, 0x060001, 3, 1 }, /* AD7124_Filter_1 */
	{ 0x23, 0x060001, 3, 1 }, /* AD7124_Filter_2 */
	{ 0x24, 0x060001, 3, 1 }, /* AD7124_Filter_3 */
	{ 0x25, 0x060001, 3, 1 }, /* AD7124_Filter_4 */
	{ 0x26, 0x060001, 3, 1 }, /* AD7124_Filter_5 */
	{ 0x27, 0x060001, 3, 1 }, /* AD7124_Filter_6 */
	{ 0x28, 0x060001, 3, 1 }, /* AD7124_Filter_7 */
	{ 0x29, 0x800000, 3, 1 }, /* AD7124_Offset_0 */
	{ 0x2A, 0x800000, 3, 1 }, /* AD7124_Offset_1 */
	{ 0x2B, 0x800000, 3, 1 }, /* AD7124_Offset_2 */
	{ 0x2C, 0x800000, 3, 1 }, /* AD7124_Offset_3 */
	{ 0x2D, 0x800000, 3, 1 }, /* AD7124_Offset_4 */
	{ 0x2E, 0x800000, 3, 1 }, /* AD7124_Offset_5 */
	{ 0x2F, 0x800000, 3, 1 }, /* AD7124_Offset_6 */
	{ 0x30, 0x800000, 3, 1 }, /* AD7124_Offset_7 */
	{ 0x31, 0x500000, 3, 1 }, /* AD7124_Gain_0 */
	{ 0x32, 0x500000, 3, 1 }, /* AD7124_Gain_1 */
	{ 0x33, 0x500000, 3, 1 }, /* AD7124_Gain_2 */
	{ 0x34, 0x500000, 3, 1 }, /* AD7124_Gain_3 */
	{ 0x35, 0x500000, 3, 1 }, /* AD7124_Gain_4 */
	{ 0x36, 0x500000, 3, 1 }, /* AD7124_Gain_5 */
	{ 0x37, 0x500000, 3, 1 }, /* AD7124_Gain_6 */
	{ 0x38, 0x500000, 3, 1 }, /* AD7124_Gain_7 */
	};

	int8_t flag = 0;
	uint8_t i = 0;

	// Fill all registers with default values
	device->regs = &ad7124_init_regs_default;
	device->responseTimeout = 1000;
	device->isReady = 0;
	device->useCRC = 0;

	// SPI Communication Data
	device->hspi = hspi;
	device->ncs_GPIOx = ncs_GPIOx;
	device->ncs_GPIO_pin = ncs_GPIO_pin;
	device->active_device = AD7124_8_ID;

	// Reset the device
	flag = ad7124_reset(device);

	// Initialize ADC
	ad7124_setAdcControl(device, AD7124_STANDBY, AD7124_LOW_POWER, 0, 0);

	// Read the ID register
	flag = ad7124_readRegister(device, &device->regs[AD7124_ID_REG]);

	if (device->active_device == ID_AD7124_4) {
		if (!(device->regs[AD7124_ID_REG].value == AD7124_4_ID))
			return 1;
	} else if (device->active_device == ID_AD7124_8) {
		if (!(device->regs[AD7124_ID_REG].value == AD7124_8_ID))
			return 1;
	}

	//	Thermocouple mode
	flag = ad7124_setAdcControl(device, AD7124_CONTINUOUS, AD7124_HIGH_POWER, 1,
			0);
	// Sets up the 8 channels
	for (i = 0; i < AD7124_MAX_SETUPS; i++) {
		flag = ad7124_setConfig(device, i, EXTERNAL_REFIN1, Pga128, 1, BurnoutOff);
		if (flag)
			return 1;

		flag = ad7124_setChannel(device, i, i, 2*i, 2*i+1, 0);
		if (flag)
			return 1;
	}


	if (flag)
		return 1;

	// Reads Calbiration Data
	for (uint8_t i = 0; i < 8; i++) {
		ad7124_readRegister(device, &device->regs[AD7124_Offset_0 + i]);
		ad7124_readRegister(device, &device->regs[AD7124_Gain_0 + i]);
	}

	return 0;
}
