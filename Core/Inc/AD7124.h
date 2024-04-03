#ifndef AD7124_H_
#define AD7124_H_

#include <stm32f4xx.h>

#define AD7124_RW 1 /* Read and Write */
#define AD7124_R 2  /* Read only */
#define AD7124_W 3  /* Write only */

#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define SPI2_CS0_Pin GPIO_PIN_10
#define SPI2_CS0_Pin_GPIO_Port GPIOD

/* Total Number of Setups */
#define AD7124_MAX_SETUPS 8
/* Maximum number of channels */
#define AD7124_MAX_CHANNELS 16
/* Device IDs */
#define AD7124_4_ID 0x14
#define AD7124_8_ID 0x16

/* AD7124 Register Map */
#define AD7124_COMM_REG 0x00
#define AD7124_STATUS_REG 0x00
#define AD7124_ADC_CTRL_REG 0x01
#define AD7124_DATA_REG 0x02
#define AD7124_IO_CTRL1_REG 0x03
#define AD7124_IO_CTRL2_REG 0x04
#define AD7124_ID_REG 0x05
#define AD7124_ERR_REG 0x06
#define AD7124_ERREN_REG 0x07
#define AD7124_CH0_MAP_REG 0x09
#define AD7124_CH1_MAP_REG 0x0A
#define AD7124_CH2_MAP_REG 0x0B
#define AD7124_CH3_MAP_REG 0x0C
#define AD7124_CH4_MAP_REG 0x0D
#define AD7124_CH5_MAP_REG 0x0E
#define AD7124_CH6_MAP_REG 0x0F
#define AD7124_CH7_MAP_REG 0x10
#define AD7124_CH8_MAP_REG 0x11
#define AD7124_CH9_MAP_REG 0x12
#define AD7124_CH10_MAP_REG 0x13
#define AD7124_CH11_MAP_REG 0x14
#define AD7124_CH12_MAP_REG 0x15
#define AD7124_CH13_MAP_REG 0x16
#define AD7124_CH14_MAP_REG 0x17
#define AD7124_CH15_MAP_REG 0x18
#define AD7124_CFG0_REG 0x19
#define AD7124_CFG1_REG 0x1A
#define AD7124_CFG2_REG 0x1B
#define AD7124_CFG3_REG 0x1C
#define AD7124_CFG4_REG 0x1D
#define AD7124_CFG5_REG 0x1E
#define AD7124_CFG6_REG 0x1F
#define AD7124_CFG7_REG 0x20
#define AD7124_FILT0_REG 0x21
#define AD7124_FILT1_REG 0x22
#define AD7124_FILT2_REG 0x23
#define AD7124_FILT3_REG 0x24
#define AD7124_FILT4_REG 0x25
#define AD7124_FILT5_REG 0x26
#define AD7124_FILT6_REG 0x27
#define AD7124_FILT7_REG 0x28
#define AD7124_OFFS0_REG 0x29
#define AD7124_OFFS1_REG 0x2A
#define AD7124_OFFS2_REG 0x2B
#define AD7124_OFFS3_REG 0x2C
#define AD7124_OFFS4_REG 0x2D
#define AD7124_OFFS5_REG 0x2E
#define AD7124_OFFS6_REG 0x2F
#define AD7124_OFFS7_REG 0x30
#define AD7124_GAIN0_REG 0x31
#define AD7124_GAIN1_REG 0x32
#define AD7124_GAIN2_REG 0x33
#define AD7124_GAIN3_REG 0x34
#define AD7124_GAIN4_REG 0x35
#define AD7124_GAIN5_REG 0x36
#define AD7124_GAIN6_REG 0x37
#define AD7124_GAIN7_REG 0x38

/* Communication Register bits */
#define AD7124_COMM_REG_WEN (0 << 7)
#define AD7124_COMM_REG_WR (0 << 6)
#define AD7124_COMM_REG_RD (1 << 6)
#define AD7124_COMM_REG_RA(x) ((x) & 0x3F)

/* Status Register bits */
#define AD7124_STATUS_REG_RDY (1 << 7)
#define AD7124_STATUS_REG_ERROR_FLAG (1 << 6)
#define AD7124_STATUS_REG_POR_FLAG (1 << 4)
#define AD7124_STATUS_REG_CH_ACTIVE(x) ((x) & 0xF)

/* ADC_Control Register bits */
#define AD7124_ADC_CTRL_REG_DOUT_RDY_DEL (1 << 12)
#define AD7124_ADC_CTRL_REG_CONT_READ (1 << 11)
#define AD7124_ADC_CTRL_REG_DATA_STATUS (1 << 10)
#define AD7124_ADC_CTRL_REG_CS_EN (1 << 9)
#define AD7124_ADC_CTRL_REG_REF_EN (1 << 8)
#define AD7124_ADC_CTRL_REG_POWER_MODE(x) (((x) & 0x3) << 6)
#define AD7124_ADC_CTRL_REG_MODE(x) (((x) & 0xF) << 2)
#define AD7124_ADC_CTRL_REG_CLK_SEL(x) (((x) & 0x3) << 0)

/* IO_Control_1 Register bits */
#define AD7124_IO_CTRL1_REG_GPIO_DAT2 (1 << 23)
#define AD7124_IO_CTRL1_REG_GPIO_DAT1 (1 << 22)
#define AD7124_IO_CTRL1_REG_GPIO_CTRL2 (1 << 19)
#define AD7124_IO_CTRL1_REG_GPIO_CTRL1 (1 << 18)
#define AD7124_IO_CTRL1_REG_PDSW (1 << 15)
#define AD7124_IO_CTRL1_REG_IOUT1(x) (((x) & 0x7) << 11)
#define AD7124_IO_CTRL1_REG_IOUT0(x) (((x) & 0x7) << 8)
#define AD7124_IO_CTRL1_REG_IOUT_CH1(x) (((x) & 0xF) << 4)
#define AD7124_IO_CTRL1_REG_IOUT_CH0(x) (((x) & 0xF) << 0)

/* IO_Control_1 AD7124-8 specific bits */
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT4 (1 << 23)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT3 (1 << 22)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT2 (1 << 21)
#define AD7124_8_IO_CTRL1_REG_GPIO_DAT1 (1 << 20)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL4 (1 << 19)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL3 (1 << 18)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL2 (1 << 17)
#define AD7124_8_IO_CTRL1_REG_GPIO_CTRL1 (1 << 16)

/* IO_Control_2 Register bits */
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS7 (1 << 15)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS6 (1 << 14)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS5 (1 << 11)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS4 (1 << 10)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS3 (1 << 5)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS2 (1 << 4)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS1 (1 << 1)
#define AD7124_IO_CTRL2_REG_GPIO_VBIAS0 (1 << 0)

/* IO_Control_2 AD7124-8 specific bits */
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS15 (1 << 15)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS14 (1 << 14)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS13 (1 << 13)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS12 (1 << 12)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS11 (1 << 11)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS10 (1 << 10)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS9 (1 << 9)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS8 (1 << 8)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7 (1 << 7)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS6 (1 << 6)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS5 (1 << 5)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS4 (1 << 4)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS3 (1 << 3)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS2 (1 << 2)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS1 (1 << 1)
#define AD7124_8_IO_CTRL2_REG_GPIO_VBIAS0 (1 << 0)

/* ID Register bits */
#define AD7124_ID_REG_DEVICE_ID(x) (((x) & 0xF) << 4)
#define AD7124_ID_REG_SILICON_REV(x) (((x) & 0xF) << 0)

/* Error Register bits */
#define AD7124_ERR_REG_LDO_CAP_ERR (1 << 19)
#define AD7124_ERR_REG_ADC_CAL_ERR (1 << 18)
#define AD7124_ERR_REG_ADC_CONV_ERR (1 << 17)
#define AD7124_ERR_REG_ADC_SAT_ERR (1 << 16)
#define AD7124_ERR_REG_AINP_OV_ERR (1 << 15)
#define AD7124_ERR_REG_AINP_UV_ERR (1 << 14)
#define AD7124_ERR_REG_AINM_OV_ERR (1 << 13)
#define AD7124_ERR_REG_AINM_UV_ERR (1 << 12)
#define AD7124_ERR_REG_REF_DET_ERR (1 << 11)
#define AD7124_ERR_REG_DLDO_PSM_ERR (1 << 9)
#define AD7124_ERR_REG_ALDO_PSM_ERR (1 << 7)
#define AD7124_ERR_REG_SPI_IGNORE_ERR (1 << 6)
#define AD7124_ERR_REG_SPI_SLCK_CNT_ERR (1 << 5)
#define AD7124_ERR_REG_SPI_READ_ERR (1 << 4)
#define AD7124_ERR_REG_SPI_WRITE_ERR (1 << 3)
#define AD7124_ERR_REG_SPI_CRC_ERR (1 << 2)
#define AD7124_ERR_REG_MM_CRC_ERR (1 << 1)
#define AD7124_ERR_REG_ROM_CRC_ERR (1 << 0)

/* Error_En Register bits */
#define AD7124_ERREN_REG_MCLK_CNT_EN (1 << 22)
#define AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN (1 << 21)
#define AD7124_ERREN_REG_LDO_CAP_CHK(x) (((x) & 0x3) << 19)
#define AD7124_ERREN_REG_ADC_CAL_ERR_EN (1 << 18)
#define AD7124_ERREN_REG_ADC_CONV_ERR_EN (1 << 17)
#define AD7124_ERREN_REG_ADC_SAT_ERR_EN (1 << 16)
#define AD7124_ERREN_REG_AINP_OV_ERR_EN (1 << 15)
#define AD7124_ERREN_REG_AINP_UV_ERR_EN (1 << 14)
#define AD7124_ERREN_REG_AINM_OV_ERR_EN (1 << 13)
#define AD7124_ERREN_REG_AINM_UV_ERR_EN (1 << 12)
#define AD7124_ERREN_REG_REF_DET_ERR_EN (1 << 11)
#define AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN (1 << 10)
#define AD7124_ERREN_REG_DLDO_PSM_ERR_ERR (1 << 9)
#define AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN (1 << 8)
#define AD7124_ERREN_REG_ALDO_PSM_ERR_EN (1 << 7)
#define AD7124_ERREN_REG_SPI_IGNORE_ERR_EN (1 << 6)
#define AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN (1 << 5)
#define AD7124_ERREN_REG_SPI_READ_ERR_EN (1 << 4)
#define AD7124_ERREN_REG_SPI_WRITE_ERR_EN (1 << 3)
#define AD7124_ERREN_REG_SPI_CRC_ERR_EN (1 << 2)
#define AD7124_ERREN_REG_MM_CRC_ERR_EN (1 << 1)
#define AD7124_ERREN_REG_ROM_CRC_ERR_EN (1 << 0)

/* Channel Registers 0-15 bits */
#define AD7124_CH_MAP_REG_CH_ENABLE (1 << 15)
#define AD7124_CH_MAP_REG_SETUP(x) (((x) & 0x7) << 12)
#define AD7124_CH_MAP_REG_AINP(x) (((x) & 0x1F) << 5)
#define AD7124_CH_MAP_REG_AINM(x) (((x) & 0x1F) << 0)

/* Configuration Registers 0-7 bits */
#define AD7124_CFG_REG_BIPOLAR (1 << 11)
#define AD7124_CFG_REG_BURNOUT(x) (((x) & 0x3) << 9)
#define AD7124_CFG_REG_REF_BUFP (1 << 8)
#define AD7124_CFG_REG_REF_BUFM (1 << 7)
#define AD7124_CFG_REG_AIN_BUFP (1 << 6)
#define AD7124_CFG_REG_AINN_BUFM (1 << 5)
#define AD7124_CFG_REG_REF_SEL(x) ((x) & 0x3) << 3
#define AD7124_CFG_REG_PGA(x) (((x) & 0x7) << 0)

/* Filter Register 0-7 bits */
#define AD7124_FILT_REG_FILTER(x) (((x) & 0x7) << 21)
#define AD7124_FILT_REG_REJ60 (1 << 20)
#define AD7124_FILT_REG_POST_FILTER(x) (((x) & 0x7) << 17)
#define AD7124_FILT_REG_SINGLE_CYCLE (1 << 16)
#define AD7124_FILT_REG_FS(x) (((x) & 0x7FF) << 0)

#define AD7124_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */
#define AD7124_DISABLE_CRC 0
#define AD7124_USE_CRC 1

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/**
 * @enum	ad7124_device_type
 * @details AD7124 Device definitions
 **/
enum ad7124_device_type {
	ID_AD7124_4, ID_AD7124_8
};

/**
 * @enum  ad7124_mode
 * @brief ADC Modes of Operation
 **/
enum ad7124_mode {
	AD7124_CONTINUOUS,
	AD7124_SINGLE,
	AD7124_STANDBY,
	AD7124_POWER_DOWN,
	AD7124_IDLE,
	AD7124_IN_ZERO_SCALE_OFF,
	AD7124_IN_FULL_SCALE_GAIN,
	AD7124_SYS_ZERO_SCALE_OFF,
	AD7124_SYS_ZERO_SCALE_GAIN,
	ADC_MAX_MODES
};

/**
 * @enum ad7124_analog_input
 * @brief ADC input sources for each channel.
 **/
enum ad7124_analog_input {
	AD7124_AIN0,
	AD7124_AIN1,
	AD7124_AIN2,
	AD7124_AIN3,
	AD7124_AIN4,
	AD7124_AIN5,
	AD7124_AIN6,
	AD7124_AIN7,
	AD7124_AIN8,
	AD7124_AIN9,
	AD7124_AIN10,
	AD7124_AIN11,
	AD7124_AIN12,
	AD7124_AIN13,
	AD7124_AIN14,
	AD7124_AIN15,
	AD7124_TEMP_SENSOR,
	AD7124_AVSS,
	AD7124_IN_REF,
	AD7124_DGND,
	AD7124_AVDD_AVSS_P,
	AD7124_AVDD_AVSS_M,
	AD7124_IOVDD_DGND_P,
	AD7124_IOVDD_DGND_M,
	AD7124_ALDO_AVSS_P,
	AD7124_ALDO_AVSS_M,
	AD7124_DLDO_DGND_P,
	AD7124_DLDO_DGND_M,
	AD7124_V_20MV_P,
	AD7124_V_20MV_M
};

/**
 * @struct ad7124_analog_inputs
 * @brief Positive and negative analog inputs
 **/
struct ad7124_analog_inputs {
	enum ad7124_analog_input ainp;
	enum ad7124_analog_input ainm;
};

/**
 * @struct ad7124_channel_map
 * @brief Channel mapping
 **/
struct ad7124_channel_map {
	uint8_t channel_enable;
	uint8_t setup_sel;
	struct ad7124_analog_inputs ain;
};

/**
 * @enum ad7124_reference_source
 * @brief Type of ADC Reference
 **/
enum ad7124_reference_source {
	/* External Reference REFIN1+/-*/
	EXTERNAL_REFIN1,
	/* External Reference REFIN2+/-*/
	EXTERNAL_REFIN2,
	/* Internal 2.5V Reference */
	INTERNAL_REF,
	/* AVDD - AVSS */
	AVDD_AVSS,
	/* Maximum Reference Sources */
	MAX_REF_SOURCES
};

/**
 * @struct ad7124_channel_setup
 * @brief Channel setup
 **/
struct ad7124_channel_setup {
	uint8_t bi_unipolar;
	uint8_t ref_buff;
	uint8_t ain_buff;
	enum ad7124_reference_source ref_source;
};

/**
 * @enum ad7124_powerMode
 * @brief Power modes
 **/
enum ad7124_powerMode {
	AD7124_LOW_POWER, AD7124_MID_POWER, AD7124_HIGH_POWER
};

/* Device register info */
struct ad7124_registerData {
	int32_t addr;
	int32_t value;
	int32_t size;
	int32_t rw;
};

/* AD7124 registers list */
enum ad7124_registers {
	AD7124_Status,
	AD7124_ADC_Control,
	AD7124_Data,
	AD7124_IOCon1,
	AD7124_IOCon2,
	AD7124_ID,
	AD7124_Error,
	AD7124_Error_En,
	AD7124_Mclk_Count,
	AD7124_Channel_0,
	AD7124_Channel_1,
	AD7124_Channel_2,
	AD7124_Channel_3,
	AD7124_Channel_4,
	AD7124_Channel_5,
	AD7124_Channel_6,
	AD7124_Channel_7,
	AD7124_Channel_8,
	AD7124_Channel_9,
	AD7124_Channel_10,
	AD7124_Channel_11,
	AD7124_Channel_12,
	AD7124_Channel_13,
	AD7124_Channel_14,
	AD7124_Channel_15,
	AD7124_Config_0,
	AD7124_Config_1,
	AD7124_Config_2,
	AD7124_Config_3,
	AD7124_Config_4,
	AD7124_Config_5,
	AD7124_Config_6,
	AD7124_Config_7,
	AD7124_Filter_0,
	AD7124_Filter_1,
	AD7124_Filter_2,
	AD7124_Filter_3,
	AD7124_Filter_4,
	AD7124_Filter_5,
	AD7124_Filter_6,
	AD7124_Filter_7,
	AD7124_Offset_0,
	AD7124_Offset_1,
	AD7124_Offset_2,
	AD7124_Offset_3,
	AD7124_Offset_4,
	AD7124_Offset_5,
	AD7124_Offset_6,
	AD7124_Offset_7,
	AD7124_Gain_0,
	AD7124_Gain_1,
	AD7124_Gain_2,
	AD7124_Gain_3,
	AD7124_Gain_4,
	AD7124_Gain_5,
	AD7124_Gain_6,
	AD7124_Gain_7,
	AD7124_REG_NO
};

enum ad7124_PgaSel {
	Pga1 = 0, /**< Gain 1, Input Range When VREF = 2.5 V: ±2.5 V */
	Pga2, /**< Gain 2, Input Range When VREF = 2.5 V: ±1.25 V */
	Pga4, /**< Gain 4, Input Range When VREF = 2.5 V: ± 625 mV */
	Pga8, /**< Gain 8, Input Range When VREF = 2.5 V: ±312.5 mV */
	Pga16, /**< Gain 16, Input Range When VREF = 2.5 V: ±156.25 mV */
	Pga32, /**< Gain 32, Input Range When VREF = 2.5 V: ±78.125 mV */
	Pga64, /**< Gain 64, Input Range When VREF = 2.5 V: ±39.06 mV */
	Pga128 /**< Gain 128, Input Range When VREF = 2.5 V: ±19.53 mV */
};

enum ad7124_BurnoutCurrent {
	BurnoutOff = 0, /**< burnout current source off (default). */
	Burnout500nA, /**< burnout current source on, 0.5 μA. */
	Burnout2uA, /**< burnout current source on, 2 μA. */
	Burnout4uA /**< burnout current source on, 4 μA. */
};

//struct ad7124_registerData ad7124_init_regs_default[AD7124_REG_NO] = {
//    {0x00, 0x00, 1, 2}, /* AD7124_Status */
//    /*
//     * 7:6 - 11 : Full power mode
//     * 8 - 1: Internale ref enabled
//     * 1:0 - 00 Internal 614.4 clock
//     */
//    {0x01, 0x05C0, 2, 1}, /* AD7124_ADC_Control */
//    {0x02, 0x0000, 3, 2}, /* AD7124_Data */
//    {0x03, 0x0000, 3, 1}, /* AD7124_IOCon1 */
//    {0x04, 0x0000, 2, 1}, /* AD7124_IOCon2 */
//    {0x05, 0x02, 1, 2},   /* AD7124_ID */
//    {0x06, 0x0000, 3, 2}, /* AD7124_Error */
//    {0x07, 0x0000, 3, 1}, /* AD7124_Error_En */
//    {0x08, 0x00, 1, 2},   /* AD7124_Mclk_Count */
//    {0x09, 0x0001, 2, 1}, /* AD7124_Channel_0 */
//    {0x0A, 0x0043, 2, 1}, /* AD7124_Channel_1 */
//    {0x0B, 0x0043, 2, 1}, /* AD7124_Channel_2 */
//    {0x0C, 0x00C7, 2, 1}, /* AD7124_Channel_3 */
//    {0x0D, 0x0085, 2, 1}, /* AD7124_Channel_4 */
//    {0x0E, 0x014B, 2, 1}, /* AD7124_Channel_5 */
//    {0x0F, 0x00C7, 2, 1}, /* AD7124_Channel_6 */
//    {0x10, 0x01CF, 2, 1}, /* AD7124_Channel_7 */
//    {0x11, 0x0109, 2, 1}, /* AD7124_Channel_8 */
//    {0x12, 0x0031, 2, 1}, /* AD7124_Channel_9 */
//    {0x13, 0x014B, 2, 1}, /* AD7124_Channel_10 */
//    {0x14, 0x0071, 2, 1}, /* AD7124_Channel_11 */
//    {0x15, 0x018D, 2, 1}, /* AD7124_Channel_12 */
//    {0x16, 0x00B1, 2, 1}, /* AD7124_Channel_13 */
//    {0x17, 0x01CF, 2, 1}, /* AD7124_Channel_14 */
//    {0x18, 0x00F1, 2, 1}, /* AD7124_Channel_15 */
//    {0x19, 0x0980, 2, 1}, /* AD7124_Config_0 */
//    {0x1A, 0x0981, 2, 1}, /* AD7124_Config_1 */
//    {0x1B, 0x0982, 2, 1}, /* AD7124_Config_2 */
//    {0x1C, 0x0983, 2, 1}, /* AD7124_Config_3 */
//    {0x1D, 0x0984, 2, 1}, /* AD7124_Config_4 */
//    {0x1E, 0x0985, 2, 1}, /* AD7124_Config_5 */
//    {0x1F, 0x0986, 2, 1}, /* AD7124_Config_6 */
//    {0x20, 0x0987, 2, 1}, /* AD7124_Config_7 */
//    /*
//     * 10:0 - 1 - FS = 1 from filter 0 to 7
//     */
//    {0x21, 0x060001, 3, 1}, /* AD7124_Filter_0 */
//    {0x22, 0x060001, 3, 1}, /* AD7124_Filter_1 */
//    {0x23, 0x060001, 3, 1}, /* AD7124_Filter_2 */
//    {0x24, 0x060001, 3, 1}, /* AD7124_Filter_3 */
//    {0x25, 0x060001, 3, 1}, /* AD7124_Filter_4 */
//    {0x26, 0x060001, 3, 1}, /* AD7124_Filter_5 */
//    {0x27, 0x060001, 3, 1}, /* AD7124_Filter_6 */
//    {0x28, 0x060001, 3, 1}, /* AD7124_Filter_7 */
//    {0x29, 0x800000, 3, 1}, /* AD7124_Offset_0 */
//    {0x2A, 0x800000, 3, 1}, /* AD7124_Offset_1 */
//    {0x2B, 0x800000, 3, 1}, /* AD7124_Offset_2 */
//    {0x2C, 0x800000, 3, 1}, /* AD7124_Offset_3 */
//    {0x2D, 0x800000, 3, 1}, /* AD7124_Offset_4 */
//    {0x2E, 0x800000, 3, 1}, /* AD7124_Offset_5 */
//    {0x2F, 0x800000, 3, 1}, /* AD7124_Offset_6 */
//    {0x30, 0x800000, 3, 1}, /* AD7124_Offset_7 */
//    {0x31, 0x500000, 3, 1}, /* AD7124_Gain_0 */
//    {0x32, 0x500000, 3, 1}, /* AD7124_Gain_1 */
//    {0x33, 0x500000, 3, 1}, /* AD7124_Gain_2 */
//    {0x34, 0x500000, 3, 1}, /* AD7124_Gain_3 */
//    {0x35, 0x500000, 3, 1}, /* AD7124_Gain_4 */
//    {0x36, 0x500000, 3, 1}, /* AD7124_Gain_5 */
//    {0x37, 0x500000, 3, 1}, /* AD7124_Gain_6 */
//    {0x38, 0x500000, 3, 1}, /* AD7124_Gain_7 */
//};

/**
 * The structure describes the device and is used with the ad7124 driver.
 * @brief Device Structure
 **/
struct ad7124_dev {
	/* SPI Settings*/
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *ncs_GPIOx;
	uint16_t ncs_GPIO_pin;
	/* Device Settings */
	struct ad7124_registerData *regs;
	int8_t useCRC;
	int8_t isReady;
	int32_t responseTimeout;

	// AFTER HERE - Idk what to do with
	enum ad7124_mode mode;
	/* Active Device */
	enum ad7124_device_type active_device;
	/* Reference enable */
	uint8_t ref_en;
	/* Power modes */
	enum ad7124_powerMode power_mode;
	/* Setups */
	struct ad7124_channel_setup setups[AD7124_MAX_SETUPS];
	/* Channel Mapping*/
	struct ad7124_channel_map chan_map[AD7124_MAX_CHANNELS];
};

int8_t ad7124_noCheckReadRegister(struct ad7124_dev *dev,
		struct ad7124_registerData *p_reg);
int8_t ad7124_noCheckWriteRegister(struct ad7124_dev *dev,
		struct ad7124_registerData p_reg);
int8_t ad7124_readRegister(struct ad7124_dev *dev,
		struct ad7124_registerData *p_reg);
int8_t ad7124_writeRegister(struct ad7124_dev *dev,
		struct ad7124_registerData p_reg);
int8_t ad7124_reset(struct ad7124_dev *dev);
int8_t ad7124_waitForSpiReady(struct ad7124_dev *dev, uint32_t timeout);
int8_t ad7124_waitToPowerOn(struct ad7124_dev *dev, uint32_t timeout);
int8_t ad7124_WaitForConvReady(struct ad7124_dev *dev, uint32_t timeout);
int8_t ad7124_readData(struct ad7124_dev *dev, int32_t *p_data);
int8_t ad7124_computeCrc8(uint8_t *p_buf, uint8_t buf_size);
void ad7124_updateCrcSetting(struct ad7124_dev *dev);
void ad7124_updateDevSpiSettings(struct ad7124_dev *dev);
int8_t ad7124_setAdcControl(struct ad7124_dev *dev,
		enum ad7124_mode operating_mode, enum ad7124_powerMode power_mode,
		uint8_t ref_en, uint8_t clk_sel);
int8_t ad7124_setConfig(struct ad7124_dev *dev, uint8_t cfg,
		enum ad7124_reference_source ref, enum ad7124_PgaSel pga,
		int8_t bipolar, enum ad7124_BurnoutCurrent burnout);
int8_t ad7124_setChannel(struct ad7124_dev *dev, uint8_t ch, uint8_t cfg,
		enum ad7124_analog_input ainp, enum ad7124_analog_input ainm,
		uint8_t enable);
int8_t ad7124_getRegister(struct ad7124_dev *dev, enum ad7124_registers id);
uint8_t ad7124_currentChannel(struct ad7124_dev *dev);
uint8_t ad7124_enableChannel(struct ad7124_dev *dev, uint8_t ch, uint8_t enable);
int8_t ad7124_readData(struct ad7124_dev *dev, int32_t *pData);
int32_t ad7124_getData(struct ad7124_dev *dev);
int32_t ad7124_adcRead(struct ad7124_dev *dev, uint8_t ch);
int8_t ad7124_startSingleConversion(struct ad7124_dev *dev, uint8_t ch);
int8_t ad7124_waitEndOfConversion(struct ad7124_dev *dev);
float ad7124_toVoltage(int32_t value, int16_t gain, float vref, uint8_t bipolar);
int8_t ad7124_setMode(struct ad7124_dev *dev, enum ad7124_mode operating_mode);
int8_t ad7124_init(struct ad7124_dev *device, SPI_HandleTypeDef *hspi,
		GPIO_TypeDef *ncs_GPIOx, uint16_t ncs_GPIO_pin);

#endif // !AD7124_H_
