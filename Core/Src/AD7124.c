
#include <stm32f4xx_hal.h>
#include "AD7124.h"



int8_t ad7124_noCheckReadRegister(struct ad7124_dev *dev, struct ad7124_registerData *p_reg)
{
    int8_t flag, i = 0, add_status_length = 0;
    uint8_t txBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t rxBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // Build Command word
    txBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(p_reg->addr);

    /*
     * If this is an AD7124_DATA register read, and the DATA_STATUS bit is set
     * in ADC_CONTROL, need to read 4, not 3 bytes for DATA with flag
     */
    if ((p_reg->addr == AD7124_DATA_REG) &&
        (dev->regs[AD7124_ADC_Control].value & AD7124_ADC_CTRL_REG_DATA_STATUS))
        add_status_length = 1;

    uint16_t length = (((dev->useCRC != AD7124_DISABLE_CRC) ? p_reg->size + 1 : p_reg->size) + 1 + add_status_length);

    HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_RESET);
    flag = (HAL_SPI_TransmitReceive(dev->hspi, &txBuf[0], &rxBuf[0], length, HAL_MAX_DELAY) != HAL_OK);
    HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_SET);

    if (flag)
        return flag;

    // Check the CRC
    if (dev->useCRC == AD7124_USE_CRC)
    {
    	txBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(p_reg->addr);
        for (i = 1; i < p_reg->size + 2 + add_status_length; ++i)
        	txBuf[i] = rxBuf[i];
        flag = ad7124_computeCrc8(txBuf, p_reg->size + 2 + add_status_length);
    }

    if (flag)
        return flag;

    /*
     * if reading Data with 4 bytes, need to copy the flag byte to the flag
     * register struct value member
     */

    if (add_status_length)
        dev->regs[AD7124_Status].value = rxBuf[p_reg->size + 1];

    // Build the result
    p_reg->value = 0;
    for (i = 1; i < p_reg->size + 1; i++)
    {
        p_reg->value <<= 8;
        p_reg->value += rxBuf[i];
    }

    return 0;
}

int8_t ad7124_noCheckWriteRegister(struct ad7124_dev *dev, struct ad7124_registerData p_reg)
{
    int32_t reg_value = 0;
    int8_t flag, i = 0, add_status_length = 0;
    uint8_t txBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // Build Command word
    txBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(p_reg.addr);

    // Fill the write buffer
    reg_value = p_reg.value;
    for (i = 0; i < p_reg.size; i++)
    {
        txBuf[p_reg.size - i] = reg_value & 0xFF;
        reg_value >>= 8;
    }

    // Compute the CRC
    if (dev->useCRC != AD7124_DISABLE_CRC)
    {
        flag = ad7124_computeCrc8(txBuf, p_reg.size + 1);
        txBuf[p_reg.size + 1] = flag;
    }

    uint16_t length = (((dev->useCRC != AD7124_DISABLE_CRC) ? p_reg.size + 1 : p_reg.size) + 1 + add_status_length);

    HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_RESET);
    flag = (HAL_SPI_Transmit(dev->hspi, txBuf, length , HAL_MAX_DELAY) != HAL_OK);
    HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_SET);

    return flag;
}

int8_t ad7124_readRegister(struct ad7124_dev *dev, struct ad7124_registerData *p_reg)
{
    int8_t flag;

    if (p_reg->addr != AD7124_ERR_REG && dev->isReady)
    {
        flag = ad7124_waitForSpiReady(dev,
                                      dev->responseTimeout);
        if (flag)
            return flag;
    }

    return ad7124_noCheckReadRegister(dev, p_reg);
}

int8_t ad7124_readRegister2(struct ad7124_dev *dev, uint32_t reg, uint32_t *readval)
{
    int8_t flag;

    flag = ad7124_readRegister(dev, &dev->regs[reg]);
    if (flag)
        return flag;

    *readval = dev->regs[reg].value;

    return 0;
}

int8_t ad7124_writeRegister(struct ad7124_dev *dev,
                            struct ad7124_registerData p_reg)
{
    int8_t flag;

    if (dev->isReady)
    {
        flag = ad7124_waitForSpiReady(dev,
                                         dev->responseTimeout);
        if (flag)
            return flag;
    }

    return ad7124_noCheckWriteRegister(dev,
                                       p_reg);
}

int8_t ad7124_writeRegister2(struct ad7124_dev *dev,
                             uint32_t reg,
                             uint32_t writeval)
{
    dev->regs[reg].value = writeval;

    return ad7124_writeRegister(dev, dev->regs[reg]);
}

int8_t ad7124_reset(struct ad7124_dev *dev)
{
    int8_t flag = 0;
    uint8_t txBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_RESET);
    flag = (HAL_SPI_Transmit(dev->hspi, txBuf, 8, HAL_MAX_DELAY) != HAL_OK);
    HAL_GPIO_WritePin(dev->ncs_GPIOx, dev->ncs_GPIO_pin, GPIO_PIN_SET);

    if (flag)
        return flag;

    /* CRC is disabled after reset */
    dev->useCRC = AD7124_DISABLE_CRC;

    /* Read POR bit to clear */
    flag = ad7124_waitToPowerOn(dev, dev->responseTimeout);
    if (flag)
        return flag;

    // Recommened 4ms delay
    HAL_Delay(4);

    return 0;
}

// Waits until the device can accept read and write user actions
int8_t ad7124_waitForSpiReady(struct ad7124_dev *dev, uint32_t timeout)
{
    struct ad7124_registerData *regs;
    int8_t flag;
    int8_t ready;

    regs = dev->regs;

    while (!ready && --timeout)
    {
        // read the value of the error registers
        flag = ad7124_readRegister(dev, &regs[AD7124_Error]);
        if (flag)
            return flag;

        ready = (regs[AD7124_Error].value & AD7124_ERR_REG_SPI_IGNORE_ERR) == 0;
    }

    if (!timeout)
        return 1;

    return 0;
}

// Waits until the device finishes the power-on reset operations
int8_t ad7124_waitToPowerOn(struct ad7124_dev *dev, uint32_t timeout)
{
    struct ad7124_registerData *regs;
    int32_t flag;
    int8_t powered_on;

    regs = dev->regs;

    while (!powered_on && --timeout)
    {
        flag = ad7124_readRegister(dev, &regs[AD7124_Status]);

        if (flag)
            return flag;

        // Check the POR_FLAG bit in the flag Register
        powered_on = (regs[AD7124_Status].value & AD7124_STATUS_REG_POR_FLAG) == 0;
    }

    if (!(timeout || powered_on))
        return 1;
    return 0;
}

// Waits until a new conversion result is available
int8_t ad7124_WaitForConvReady(struct ad7124_dev *dev, uint32_t timeout)
{
    struct ad7124_registerData *regs;
    int8_t flag, ready = 0;

    regs = dev->regs;

    while (!ready && --timeout)
    {
        // Read d the value of the flag register
        flag = ad7124_readRegister(dev, &regs[AD7124_Status]);
        if (flag)
            return flag;
        // Check the RDY bit in the flag register
        ready = (regs[AD7124_Status].value & AD7124_STATUS_REG_RDY == 0);
    }

    if (!timeout)
        return 1;

    return 0;
}

// Reads the conversion result from the device
int8_t ad7124_readData(struct ad7124_dev *dev, int32_t *p_data)
{
    struct ad7124_registerData *regs;
    int8_t flag;

    regs = dev->regs;

    // Read the value of the data register
    flag = ad7124_readRegister(dev, &regs[AD7124_Data]);

    *p_data = regs[AD7124_Data].value;

    return flag;
}

// Get the ID of the channel of the latest conversion.
int8_t ad7124_getReadChanId(struct ad7124_dev *dev, uint32_t *status)
{
    int8_t flag;
    uint32_t reg_temp;

    flag = ad7124_readRegister2(dev, AD7124_STATUS_REG, &reg_temp);
    if (flag)
        return flag;

    *status = reg_temp & AD7124_STATUS_REG_CH_ACTIVE(0xF);

    return 0;
}

// Computes the CRC checksum for a data buffer
int8_t ad7124_computeCrc8(uint8_t* p_buf, uint8_t buf_size)
{
    uint8_t i = 0;
    uint8_t crc = 0;

    while (buf_size)
    {
        for (i = 0x80; i != 0; i >>= 1)
        {
            uint8_t cmp1 = (crc & 0x80) != 0;
            uint8_t cmp2 = (*p_buf & i) != 0;
            if (cmp1 != cmp2)
            {
                /* MSB of CRC register XOR input Bit from Data */
                crc <<= 1;
                crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
            }
            else
            {
                crc <<= 1;
            }
        }
        p_buf++;
        buf_size--;
    }

    return crc;
}

// Updates the CRC settings.
void ad7124_updateCrcSetting(struct ad7124_dev *dev)
{
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
void ad7124_updateDevSpiSettings(struct ad7124_dev *dev)
{
    struct ad7124_registerData *regs;

    if (!dev)
        return;

    regs = dev->regs;

    if (regs[AD7124_Error_En].value & AD7124_ERREN_REG_SPI_IGNORE_ERR_EN)
        dev->isReady = 1;
    else
        dev->isReady = 0;
}

// Get the AD7124 reference clock
int8_t ad7124_fclkGet(struct ad7124_dev *dev, float *f_clk)
{
    int8_t flag;
    const float f_clk_fp = 614400,
                f_clk_mp = 153600,
                f_clk_lp = 76800;
    uint32_t reg_temp;

    flag = ad7124_readRegister2(dev, AD7124_ADC_Control, &reg_temp);
    if (flag)
        return flag;

    switch (dev->power_mode)
    {
    case 0:
        *f_clk = f_clk_lp;
        break;
    case 1:
        *f_clk = f_clk_mp;
        break;
    case 2:
    case 3:
        *f_clk = f_clk_fp;
        break;
    default:
        return flag;
    }

    return 0;
}

// Get the filter coefficient for the sample rate.
int8_t ad7124_fltCoeffGet(struct ad7124_dev *dev, int16_t chn_num, uint16_t *flt_coff)
{
    uint16_t power_mode;
    int8_t flag;
    uint32_t reg_temp;

    flag = ad7124_readRegister2(dev, AD7124_ADC_Control, &reg_temp);
    if (flag)
        return flag;

    power_mode = dev->power_mode;

    flag = ad7124_readRegister2(dev, (AD7124_Filter_0 + chn_num), &reg_temp);
    if (flag)
        return flag;

    *flt_coff = 32;
    if (reg_temp & AD7124_FILT_REG_SINGLE_CYCLE)
    {
        if ((reg_temp & AD7124_FILT_REG_FILTER(7)) ==
            AD7124_FILT_REG_FILTER(0))
            *flt_coff *= 4;
        if ((reg_temp & AD7124_FILT_REG_FILTER(7)) ==
            AD7124_FILT_REG_FILTER(2))
            *flt_coff *= 3;
    }
    if ((reg_temp & AD7124_FILT_REG_FILTER(7)) ==
        AD7124_FILT_REG_FILTER(4))
    {
        if (power_mode == 0)
            *flt_coff *= 11;
        else
            *flt_coff *= 19;
    }
    if ((reg_temp & AD7124_FILT_REG_FILTER(7)) ==
        AD7124_FILT_REG_FILTER(5))
    {
        if (power_mode == 0)
            *flt_coff *= 10;
        else
            *flt_coff *= 18;
    }

    return 0;
}

// Calculate ODR of the device.
float ad7124_getOdr(struct ad7124_dev *dev, int16_t chn_num)
{
    float f_clk;
    uint16_t fs_value, flt_coff;
    int8_t flag;
    uint32_t reg_temp;

    flag = ad7124_fclkGet(dev, &f_clk);
    if (flag)
        return flag;

    flag = ad7124_readRegister2(dev,
                                (AD7124_Filter_0 + chn_num),
                                &reg_temp);
    if (flag)
        return flag;

    fs_value = reg_temp & AD7124_FILT_REG_FS(0x7FF);

    if ((reg_temp & AD7124_FILT_REG_FILTER(7)) ==
        AD7124_FILT_REG_FILTER(7))
    {
        switch ((reg_temp & AD7124_FILT_REG_POST_FILTER(7)) >> 17)
        {
        case 2:
            return 27.27;
        case 3:
            return 25;
        case 5:
            return 20;
        case 6:
            return 16.7;
        default:
            return -1;
        }
    }

    flag = ad7124_fltcoffGet(dev, chn_num, &flt_coff);
    if (flag)
        return flag;

    return (f_clk / (float)(flt_coff * fs_value));
}

// Set ODR of the device
int8_t ad7124_setOdr(struct ad7124_dev *dev, float odr, int16_t chn_num)
{
    float f_clk;
    uint16_t flt_coff, fs_value;
    int8_t flag;
    uint32_t reg_temp;

    flag = ad7124_fclkGet(dev, &f_clk);
    if (flag)
        return flag;

    flag = ad7124_fltcoffGet(dev, chn_num, &flt_coff);
    if (flag)
        return flag;

    fs_value = (uint16_t)(f_clk / (flt_coff * odr));
    if (fs_value == 0)
        fs_value = 1;
    if (fs_value > 2047)
        fs_value = 2047;

    flag = ad7124_readRegister2(dev,
                                (AD7124_Filter_0 + chn_num),
                                &reg_temp);
    if (flag)
        return flag;

    reg_temp &= ~AD7124_FILT_REG_FS(0x7FF);
    reg_temp |= AD7124_FILT_REG_FS(fs_value);

    return ad7124_writeRegister2(dev, (AD7124_Filter_0 + chn_num), reg_temp);
}

// SPI internal register write to device using a mask.
int8_t ad7124_regWriteMsk(struct ad7124_dev *dev, uint32_t reg_addr, uint32_t data, uint32_t mask)
{
    int8_t flag;
    uint32_t reg_data;

    flag = ad7124_readRegister2(dev, reg_addr, &reg_data);
    if (flag)
        return flag;

    reg_data &= ~mask;
    reg_data |= data;

    return ad7124_writeRegister2(dev, reg_addr, reg_data);
}

// Set ADC Mode
int8_t ad7124_setAdcMode(struct ad7124_dev *device, enum ad7124_mode adc_mode)
{
    int8_t flag;

    if (!device || adc_mode >= ADC_MAX_MODES)
        return 1;

    flag = ad7124_regWriteMsk(device, 0x01 ,AD7124_ADC_CTRL_REG_MODE(0X0F), AD7124_ADC_CTRL_REG_MODE(adc_mode));
    if (flag)
        return flag;

    device->mode = adc_mode;

    return 0;
}

// Enable/disable channel.
//int8_t ad7124_setChannelStatus(struct ad7124_dev *device,
//                               uint8_t chn_num,
//                               uint8_t channel_status)
//{
//    int8_t flag;
//
//    if (channel_status)
//        channel_status = AD7124_CH_MAP_REG_CH_ENABLE;
//    else
//        channel_status = 0x0U;
//
//    flag = ad7124_regWriteMsk(device,
//                              AD7124_CH0_MAP_REG + chn_num,
//                              AD7124_CH_MAP_REG_CH_ENABLE,
//                              AD7124_CH_MAP_REG_CH_ENABLE);
//    if (flag)
//        return flag;
//
//    device->chan_map[chn_num].channel_enable = channel_status;
//
//    return 0;
//}

// Set Analog Inputs to channel.
int8_t ad7124_connectAnalogInput(struct ad7124_dev *device,
                                 uint8_t chn_num,
                                 struct ad7124_analog_inputs analog_input)
{
    int8_t flag;

    /* Select the Positive Analog Input */
    flag = ad7124_regWriteMsk(device,
                              AD7124_CH0_MAP_REG + chn_num,
                              AD7124_CH_MAP_REG_AINP(analog_input.ainp),
                              AD7124_CH_MAP_REG_AINP(analog_input.ainp));
    if (flag)
        return flag;

    /* Select the Negative Analog Input */
    flag = ad7124_regWriteMsk(device,
                              AD7124_CH0_MAP_REG + chn_num,
                              AD7124_CH_MAP_REG_AINM(analog_input.ainm),
                              AD7124_CH_MAP_REG_AINM(analog_input.ainm));
    if (flag)
        return flag;

    device->chan_map[chn_num].ain.ainp =
        analog_input.ainp;
    device->chan_map[chn_num].ain.ainm =
        analog_input.ainm;

    return 0;
}

// Assign Setup to Channel.
int8_t ad7124_assignSetup(struct ad7124_dev *device,
                          uint8_t chn_num,
                          uint8_t setup)
{
    int8_t flag;

    /* Assign setup to the Channel Register. */
    flag = ad7124_regWriteMsk(device,
                              AD7124_CH0_MAP_REG + chn_num,
                              AD7124_CH_MAP_REG_SETUP(setup),
                              AD7124_CH_MAP_REG_SETUP(setup));
    if (flag)
        return (flag);

    device->chan_map[chn_num].setup_sel = setup;

    return 0;
}

// Set Polarity
int8_t ad7124_setPolarity(struct ad7124_dev *device,
                          uint8_t bipolar,
                          uint8_t setup_id)
{
    int8_t flag;
    uint32_t reg_data;

    if (bipolar)
        reg_data = AD7124_CFG_REG_BIPOLAR;
    else
        reg_data = 0x0U;

    flag = ad7124_regWriteMsk(device,
                              AD7124_CFG0_REG + setup_id,
                              reg_data,
                              AD7124_CFG_REG_BIPOLAR);
    if (flag)
        return flag;

    device->setups[setup_id].bi_unipolar = bipolar;

    return 0;
}

// Select the reference source.
int8_t ad7124_setReferenceSource(struct ad7124_dev *device,
                                 enum ad7124_reference_source ref_source,
                                 uint8_t setup_id,
                                 uint16_t ref_en)
{
    int8_t flag;

    if (!device || ref_source >= MAX_REF_SOURCES)
        return 1;

    flag = ad7124_regWriteMsk(device,
                              AD7124_CFG0_REG + setup_id,
                              AD7124_CFG_REG_REF_SEL(ref_source),
                              AD7124_CFG_REG_REF_SEL(ref_source));
    if (flag)
        return flag;

    device->setups[setup_id].ref_source = ref_source;

    if (ref_en)
        ref_en = AD7124_ADC_CTRL_REG_REF_EN;
    else
        ref_en = 0x0U;

    /* Enable the REF_EN Bit in case of Internal reference */
    if (ref_source == INTERNAL_REF)
    {
        flag = ad7124_regWriteMsk(device,
                                  AD7124_ADC_CTRL_REG,
                                  ref_en,
                                  AD7124_ADC_CTRL_REG_REF_EN);
        if (flag)
            return flag;
    }

    device->ref_en = ref_en;

    return 0;
}

// Enable Input Buffer
int8_t ad7124_enableBuffers(struct ad7124_dev *device,
                            uint8_t inbuf_en,
                            uint8_t refbuf_en,
                            uint8_t setup_id)
{
    int8_t flag;
    uint32_t reg_val;

    if (inbuf_en)
        /* Enable input buffer for the chosen set up. */
        reg_val = (AD7124_CFG_REG_AIN_BUFP |
                   AD7124_CFG_REG_AINN_BUFM);
    else
        reg_val = 0;

    flag = ad7124_regWriteMsk(device,
                              AD7124_CFG0_REG + setup_id,
                              reg_val,
							  AD7124_CFG_REG_AIN_BUFP |
							  AD7124_CFG_REG_AINN_BUFM);
    if (flag)
        return flag;

    if (refbuf_en)
        /* Enable reference buffer for the chosen set up */
        reg_val = (AD7124_CFG_REG_REF_BUFP |
                   AD7124_CFG_REG_REF_BUFM);
    else
        reg_val = 0;

    flag = ad7124_regWriteMsk(device,
                              AD7124_CFG0_REG + setup_id,
                              reg_val,
							  AD7124_CFG_REG_AIN_BUFP |
							  AD7124_CFG_REG_AINN_BUFM);
    if (flag)
        return flag;

    device->setups[setup_id].ain_buff = inbuf_en;
    device->setups[setup_id].ref_buff = refbuf_en;

    return 0;
}

int8_t ad7124_setAdcControl(struct ad7124_dev *dev, enum ad7124_mode operating_mode, enum ad7124_power_mode power_mode, uint8_t ref_en, uint8_t clk_sel)
{
	struct ad7124_registerData *r = &dev->regs[AD7124_ADC_Control];

    r->value = AD7124_ADC_CTRL_REG_MODE(operating_mode) |
               AD7124_ADC_CTRL_REG_POWER_MODE(power_mode) |
               AD7124_ADC_CTRL_REG_CLK_SEL(clk_sel) |
               (ref_en ? AD7124_ADC_CTRL_REG_REF_EN : 0) |
               AD7124_ADC_CTRL_REG_DOUT_RDY_DEL;

    return ad7124_writeRegister(dev, *r);
}

int8_t ad7124_setConfig(struct ad7124_dev *dev, uint8_t cfg, enum ad7124_reference_source ref, enum ad7124_PgaSel pga, int8_t bipolar, enum ad7124_BurnoutCurrent burnout)
{
    if (cfg < 8)
    {
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

int8_t ad7124_setChannel(struct ad7124_dev *dev, uint8_t ch, uint8_t cfg, enum ad7124_analog_input ainp, enum ad7124_analog_input ainm, uint8_t enable)
{
    if ((ch < 16) && (cfg < 8))
    {
    	struct ad7124_registerData *r;

        ch += AD7124_Channel_0;
        r = &dev->regs[ch];

        r->value = AD7124_CH_MAP_REG_SETUP(cfg) |
                   AD7124_CH_MAP_REG_AINP(ainp) |
                   AD7124_CH_MAP_REG_AINM(ainm) |
                   (enable ? AD7124_CH_MAP_REG_CH_ENABLE : 0);

        return ad7124_writeRegister(dev, *r);
    }
    return -1;
}

int8_t ad7124_getRegister(struct ad7124_dev *dev,enum ad7124_registers id){
	int8_t flag = 0;
	if(id < AD7124_Status && id >= 57)
		return -1;

	flag = ad7124_readRegister(dev, &dev->regs[id]);
	if(flag != 0){
		return -1;
	}

	return dev->regs[id].value;

}

uint8_t ad7124_currentChannel(struct ad7124_dev *dev){
	int8_t flag = ad7124_getRegister(dev, AD7124_Status);

	if(flag < 0){
		return flag;
	}
	return (uint8_t) (flag & AD7124_STATUS_REG_CH_ACTIVE(15));
}

uint8_t ad7124_enableChannel(struct ad7124_dev *dev,uint8_t ch, uint8_t enable){
	if(ch > 16)
		return -1;

	struct ad7124_registerData * r;
	uint8_t flag;

	ch += ADC_CHANNEL_0;
	r = &dev->regs[ch];
	flag = ad7124_readRegister(dev, r);
	if (flag < 0) {
      return flag;
    }

    if (enable) {

      r->value |= AD7124_CH_MAP_REG_CH_ENABLE;

    }
    else {

      r->value &= ~AD7124_CH_MAP_REG_CH_ENABLE;
    }

    return ad7124_writeRegister(dev, dev->regs[ch]);
}


int8_t ad7124_read(struct ad7124_dev *dev, uint8_t ch){
	int8_t flag = 0;
	uint8_t cur_ch = ad7124_currentChannel(dev);

	if(ch != cur_ch){

		//diable previous channel if different
		flag = ad7124_enableChannel(dev, cur_ch, 0);
		if(flag < 0)
			return flag;
	}

	flag = ad7124_startSingleConversion(dev, ch);
	if(flag < 0){
		return flag;
	}

	flag = waitEndOfConversion();
}

int8_t ad7124_startSingleConversion(struct ad7124_dev *dev, uint8_t ch){
	  if (ch < 16) {
	    int ret;

	    ret = ad7124_enableChannel(dev, ch, 1);
	    if (ret < 0) {
	      return ret;
	    }
	    return ad7124_setMode(dev AD7124_SINGLE);
	  }
	  return -1;
}

int8_t ad7124_waitEndOfConversion(struct ad7124_dev *dev,uint32_t timeout){
	  int8_t ret;
	  uint8_t ready = 0;
	  uint8_t timeout_en;

	  timeout_en = (timeout > 0);

	  do {

	    /* Read the value of the Status Register */
	    ret = readRegister (&reg[Status]);
	    if (ret < 0) {

	      return ret;
	    }

	    /* Check the RDY bit in the Status Register */
	    ready = (reg[Status].value &
	             AD7124_STATUS_REG_RDY) == 0;

	    if (timeout) {

	      drv.delay (1);
	      timeout--;
	    }
	  }
	  while (!ready && timeout);

	  return timeout_en && (timeout == 0) ? AD7124_TIMEOUT : ready;

}

int8_t ad7124_setMode(struct ad7124_dev *dev, ad7124_mode operating_mode){
	r = &dev->regs[AD7124_ADC_Control];
	r->value &= ~AD7124_ADC_CTRL_REG_MODE (0x0F); // clear mode
	r->value |= AD7124_ADC_CTRL_REG_MODE (mode);
	return writeRegister (ADC_Control);

}
// Initializes the AD7124.
int8_t ad7124_init(struct ad7124_dev *device, SPI_HandleTypeDef *hspi, GPIO_TypeDef *ncs_GPIOx, uint16_t ncs_GPIO_pin, enum ad7124_device_type device_type, struct ad7124_registerData *ad7124_init_regs_default)
{
    int8_t flag;
    uint8_t i;

    device->regs = ad7124_init_regs_default;
    device->responseTimeout = 1000;
    device->isReady = 0;
    device->useCRC = 0;

    /* Initialize the SPI communication. */
    device->hspi = hspi;
    device->ncs_GPIOx = ncs_GPIOx;
    device->ncs_GPIO_pin = ncs_GPIO_pin;
    device->active_device = device_type;

    /*  Reset the device interface.*/
    flag = ad7124_reset(device);

    /* Initialize ADC */
    ad7124_setAdcControl(device, AD7124_STANDBY, AD7124_LOW_POWER, 0, 0);

    /* Read ID register to identify the part. */
    flag = ad7124_readRegister(device, &device->regs[AD7124_ID_REG]);

    if (device->active_device == ID_AD7124_4)
    {
        if (!(device->regs[AD7124_ID_REG].value == AD7124_4_ID))
            return 1;
    }
    else if (device->active_device == ID_AD7124_8)
    {
        if (!(device->regs[AD7124_ID_REG].value == AD7124_8_ID))
            return 1;
    }

    // Sets up the 8 channels
    for (i = 0; i < AD7124_MAX_SETUPS; i++)
    {
        flag = ad7124_setConfig(device,i,INTERNAL_REF,Pga1,1,BurnoutOff);
        if (flag)
            return 1;

        flag = ad7124_setChannel(device,i,i,i,AD7124_AVSS,0);
        if (flag)
            return 1;
    }

    flag = ad7124_setAdcControl(device, AD7124_STANDBY, AD7124_HIGH_POWER, 1, 0);

    if (flag)
        return 1;

    // Reads Calbiration Data
    for (uint8_t i = 0; i < 8; i++)
    {
        ad7124_readRegister(device,&device->regs[AD7124_Offset_0+i]);
        ad7124_readRegister(device,&device->regs[AD7124_Gain_0+i]);
    }

    return 0;
}

