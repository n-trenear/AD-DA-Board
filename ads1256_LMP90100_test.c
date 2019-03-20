/*
 * ads1256_LMP90100_test.c:
 *	Created on: 20/3/2019
 *	Author: Nathan Trenear
 *
 */

/*
             define from bcm2835.h                       define from Board DVK511
                 3.3V | | 5V               ->                 3.3V | | 5V
    RPI_V2_GPIO_P1_03 | | 5V               ->                  SDA | | 5V
    RPI_V2_GPIO_P1_05 | | GND              ->                  SCL | | GND
       RPI_GPIO_P1_07 | | RPI_GPIO_P1_08   ->                  IO7 | | TX
                  GND | | RPI_GPIO_P1_10   ->                  GND | | RX
       RPI_GPIO_P1_11 | | RPI_GPIO_P1_12   ->                  IO0 | | IO1
    RPI_V2_GPIO_P1_13 | | GND              ->                  IO2 | | GND
       RPI_GPIO_P1_15 | | RPI_GPIO_P1_16   ->                  IO3 | | IO4
                  VCC | | RPI_GPIO_P1_18   ->                  VCC | | IO5
       RPI_GPIO_P1_19 | | GND              ->                 MOSI | | GND
       RPI_GPIO_P1_21 | | RPI_GPIO_P1_22   ->                 MISO | | IO6
       RPI_GPIO_P1_23 | | RPI_GPIO_P1_24   ->                  SCK | | CE0
                  GND | | RPI_GPIO_P1_26   ->                  GND | | CE1
*/

#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <time.h>

//CS      -----   SPICS
//DIN     -----   MOSI
//DOUT  -----   MISO
//SCLK   -----   SCLK
//DRDY  -----   ctl_IO     data  starting
//RST     -----   ctl_IO     reset

//#define  DRDY  RPI_V2_GPIO_P1_11         //P0
#define  DRDY  17
//#define  RST  RPI_V2_GPIO_P1_12     //P1
#define  RST  18
//#define	SPICS	RPI_V2_GPIO_P1_15	//P3
#define  SPICS  22

#define CS_1() bcm2835_gpio_write(SPICS,HIGH)
#define CS_0()  bcm2835_gpio_write(SPICS,LOW)

#define DRDY_IS_LOW()	((bcm2835_gpio_lev(DRDY)==0))

#define RST_1() 	bcm2835_gpio_write(RST,HIGH)
#define RST_0() 	bcm2835_gpio_write(RST,LOW)

#define MISO  9
#define MOSI  10
#define SPICS2  27
#define LMP_CS_1()  bcm2835_gpio_write(SPICS2,HIGH)
#define LMP_CS_0()  bcm2835_gpio_write(SPICS2,LOW)
#define LMP_CS_IS_LOW() (bcm2835_gpio_lev(SPICS2) == 0)
#define LMP_CS_IS_HIGH() (bcm2835_gpio_lev(SPICS2) == 1)
#define LMP_DRDY_IS_LOW()  (bcm2835_gpio_lev(MISO)==0)
#define LMP_DRDY_IS_HIGH() (bcm2835_gpio_lev(MISO)==1)

/* Unsigned integer types  */
#define uint8_t unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned long

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

/* gain channel� */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* GAIN   1 */
	ADS1256_GAIN_2			= (1),	/*GAIN   2 */
	ADS1256_GAIN_4			= (2),	/*GAIN   4 */
	ADS1256_GAIN_8			= (3),	/*GAIN   8 */
	ADS1256_GAIN_16			= (4),	/* GAIN  16 */
	ADS1256_GAIN_32			= (5),	/*GAIN    32 */
	ADS1256_GAIN_64			= (6),	/*GAIN    64 */
}ADS1256_GAIN_E;

/* Sampling speed choice*/
/*
	11110000 = 30,000SPS (default)
	11100000 = 15,000SPS
	11010000 = 7,500SPS
	11000000 = 3,750SPS
	10110000 = 2,000SPS
	10100001 = 1,000SPS
	10010010 = 500SPS
	10000010 = 100SPS
	01110010 = 60SPS
	01100011 = 50SPS
	01010011 = 30SPS
	01000011 = 25SPS
	00110011 = 15SPS
	00100011 = 10SPS
	00010011 = 5SPS
	00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	ADS1256_GAIN_E Gain;		/* GAIN  */
	ADS1256_DRATE_E DataRate;	/* DATA output  speed*/
	int32_t AdcNow[8];			/* ADC  Conversion value */
	uint8_t Channel;			/* The current channel*/
	uint8_t ScanMode;	/*Scanning mode,   0  Single-ended input  8 channel�� 1 Differential input  4 channel*/
}ADS1256_VAR_T;

/*Register definition�� Table 23. Register Map --- ADS1256 datasheet Page 30*/
enum
{
	/*Register address, followed by reset the default values */
	REG_STATUS = 0,	// x1H
	REG_MUX    = 1, // 01H
	REG_ADCON  = 2, // 20H
	REG_DRATE  = 3, // F0H
	REG_IO     = 4, // E0H
	REG_OFC0   = 5, // xxH
	REG_OFC1   = 6, // xxH
	REG_OFC2   = 7, // xxH
	REG_FSC0   = 8, // xxH
	REG_FSC1   = 9, // xxH
	REG_FSC2   = 10, // xxH
};

/* Command definition�� TTable 24. Command Definitions --- ADS1256 datasheet Page 34 */
enum
{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};

ADS1256_VAR_T g_tADS1256;
static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
	0xF0,		/*reset the default values  */
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};

void  bsp_DelayUS(uint64_t micros);
void ADS1256_StartScan(uint8_t _ucScanMode);
static void ADS1256_Send8Bit(uint8_t _data);
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);
static void ADS1256_DelayDATA(void);
static uint8_t ADS1256_Recive8Bit(void);
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);
static uint8_t ADS1256_ReadReg(uint8_t _RegID);
static void ADS1256_WriteCmd(uint8_t _cmd);
uint8_t ADS1256_ReadChipID(void);
static void ADS1256_SetChannal(uint8_t _ch);
static void ADS1256_SetDiffChannal(uint8_t _ch);
static void ADS1256_WaitDRDY(void);
static int32_t ADS1256_ReadData(void);

int32_t ADS1256_GetAdc(uint8_t _ch);
void ADS1256_ISR(void);
uint8_t ADS1256_Scan(void);

void  bsp_DelayUS(uint64_t micros)
{
		bcm2835_delayMicroseconds (micros);
}
/*
*********************************************************************************************************
*	name: ADS1256_StartScan
*	function: Configuration DRDY PIN for external interrupt is triggered
*	parameter: _ucDiffMode : 0  Single-ended input  8 channel�� 1 Differential input  4 channe
*	The return value: NULL
*********************************************************************************************************
*/
void ADS1256_StartScan(uint8_t _ucScanMode)
{
	g_tADS1256.ScanMode = _ucScanMode;
	/* ��ʼɨ��ǰ, �������������� */
	{
		uint8_t ch_num;

		g_tADS1256.Channel = 0;

		for (ch_num = 0; ch_num < 8; ch_num++)
		{
			g_tADS1256.AdcNow[ch_num] = 0;
		}
	}

}

/*
*********************************************************************************************************
*	name: ADS1256_Send8Bit
*	function: SPI bus to send 8 bit data
*	parameter: _data:  data
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_Send8Bit(uint8_t _data)
{

	bsp_DelayUS(2);
	bcm2835_spi_transfer(_data);
}

/*
*********************************************************************************************************
*	name: ADS1256_CfgADC
*	function: The configuration parameters of ADC, gain and data rate
*	parameter: _gain:gain 1-64
*                      _drate:  data  rate
*	The return value: NULL
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
	g_tADS1256.Gain = _gain;
	g_tADS1256.DataRate = _drate;

	ADS1256_WaitDRDY();

	{
		uint8_t buf[4];		/* Storage ads1256 register configuration parameters */

		/*Status register define
			Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)

			Bit 3 ORDER: Data Output Bit Order
				0 = Most Significant Bit First (default)
				1 = Least Significant Bit First
			Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
			byte first. The ORDER bit only controls the bit order of the output data within the byte.

			Bit 2 ACAL : Auto-Calibration
				0 = Auto-Calibration Disabled (default)
				1 = Auto-Calibration Enabled
			When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
			the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
			values.

			Bit 1 BUFEN: Analog Input Buffer Enable
				0 = Buffer Disabled (default)
				1 = Buffer Enabled

			Bit 0 DRDY :  Data Ready (Read Only)
				This bit duplicates the state of the DRDY pin.

			ACAL=1  enable  calibration
		*/
		//buf[0] = (0 << 3) | (1 << 2) | (1 << 1);//enable the internal buffer
        buf[0] = (0 << 3) | (1 << 2) | (0 << 1);  // The internal buffer is prohibited

        //ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

		buf[1] = 0x08;

		/*	ADCON: A/D Control Register (Address 02h)
			Bit 7 Reserved, always 0 (Read Only)
			Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
				00 = Clock Out OFF
				01 = Clock Out Frequency = fCLKIN (default)
				10 = Clock Out Frequency = fCLKIN/2
				11 = Clock Out Frequency = fCLKIN/4
				When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.

			Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
				00 = Sensor Detect OFF (default)
				01 = Sensor Detect Current = 0.5 �� A
				10 = Sensor Detect Current = 2 �� A
				11 = Sensor Detect Current = 10�� A
				The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
				ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.

			Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
				000 = 1 (default)
				001 = 2
				010 = 4
				011 = 8
				100 = 16
				101 = 32
				110 = 64
				111 = 64
		*/
		buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
		//ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/*choose 1: gain 1 ;input 5V/
		buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;

		CS_0();	/* SPIƬѡ = 0 */
		ADS1256_Send8Bit(CMD_WREG | 0);	/* Write command register, send the register address */
		ADS1256_Send8Bit(0x03);			/* Register number 4,Initialize the number  -1*/

		ADS1256_Send8Bit(buf[0]);	/* Set the status register */
		ADS1256_Send8Bit(buf[1]);	/* Set the input channel parameters */
		ADS1256_Send8Bit(buf[2]);	/* Set the ADCON control register,gain */
		ADS1256_Send8Bit(buf[3]);	/* Set the output rate */

		CS_1();	/* SPI  cs = 1 */
	}

	bsp_DelayUS(50);
}


/*
*********************************************************************************************************
*	name: ADS1256_DelayDATA
*	function: delay
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
	/*
		Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
		min  50   CLK = 50 * 0.13uS = 6.5uS
	*/
	bsp_DelayUS(10);	/* The minimum time delay 6.5us */
}




/*
*********************************************************************************************************
*	name: ADS1256_Recive8Bit
*	function: SPI bus receive function
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void)
{
	uint8_t read = 0;
	read = bcm2835_spi_transfer(0xff);
	return read;
}

/*
*********************************************************************************************************
*	name: ADS1256_WriteReg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
	CS_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_WREG | _RegID);	/*Write command register */
	ADS1256_Send8Bit(0x00);		/*Write the register number */

	ADS1256_Send8Bit(_RegValue);	/*send register value */
	CS_1();	/* SPI   cs = 1 */
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadReg
*	function: Read  the corresponding register
*	parameter: _RegID: register  ID
*	The return value: read register value
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
	uint8_t read;

	CS_0();	/* SPI  cs  = 0 */
	ADS1256_Send8Bit(CMD_RREG | _RegID);	/* Write command register */
	ADS1256_Send8Bit(0x00);	/* Write the register number */

	ADS1256_DelayDATA();	/*delay time */

	read = ADS1256_Recive8Bit();	/* Read the register values */
	CS_1();	/* SPI   cs  = 1 */

	return read;
}

/*
*********************************************************************************************************
*	name: ADS1256_WriteCmd
*	function: Sending a single byte order
*	parameter: _cmd : command
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
	CS_0();	/* SPI   cs = 0 */
	ADS1256_Send8Bit(_cmd);
	CS_1();	/* SPI  cs  = 1 */
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadChipID
*	function: Read the chip ID
*	parameter: _cmd : NULL
*	The return value: four high status register
*********************************************************************************************************
*/
uint8_t ADS1256_ReadChipID(void)
{
	uint8_t id;

	ADS1256_WaitDRDY();
	id = ADS1256_ReadReg(REG_STATUS);
	return (id >> 4);
}

/*
*********************************************************************************************************
*	name: ADS1256_SetChannal
*	function: Configuration channel number
*	parameter:  _ch:  channel number  0--7
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_SetChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t care��)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t care��)
	*/
	if (_ch > 7)
	{
		return;
	}
	ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN connection AINCOM */
}

/*
*********************************************************************************************************
*	name: ADS1256_SetDiffChannal
*	function: The configuration difference channel
*	parameter:  _ch:  channel number  0--3
*	The return value:  four high status register
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannal(uint8_t _ch)
{
	/*
	Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
		0000 = AIN0 (default)
		0001 = AIN1
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t care��)

		NOTE: When using an ADS1255 make sure to only select the available inputs.

	Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
		0000 = AIN0
		0001 = AIN1 (default)
		0010 = AIN2 (ADS1256 only)
		0011 = AIN3 (ADS1256 only)
		0100 = AIN4 (ADS1256 only)
		0101 = AIN5 (ADS1256 only)
		0110 = AIN6 (ADS1256 only)
		0111 = AIN7 (ADS1256 only)
		1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t care��)
	*/
	if (_ch == 0)
	{
		ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* DiffChannal  AIN0�� AIN1 */
	}
	else if (_ch == 1)
	{
		ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/*DiffChannal   AIN2�� AIN3 */
	}
	else if (_ch == 2)
	{
		ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/*DiffChannal    AIN4�� AIN5 */
	}
	else if (_ch == 3)
	{
		ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/*DiffChannal   AIN6�� AIN7 */
	}
}

/*
*********************************************************************************************************
*	name: ADS1256_WaitDRDY
*	function: delay time  wait for automatic calibration
*	parameter:  NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void)
{
	uint32_t ch_num;

	for (ch_num = 0; ch_num < 400000; ch_num++)
	{
		if (DRDY_IS_LOW())
		{
			break;
		}
	}
	if (ch_num >= 400000)
	{
		printf("ADS1256_WaitDRDY() Time Out ...\r\n");
	}
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadData
*	function: read ADC value
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
	uint32_t read = 0;
    static uint8_t buf[3];

	CS_0();	/* SPI   cs = 0 */

	ADS1256_Send8Bit(CMD_RDATA);	/* read ADC command  */

	ADS1256_DelayDATA();	/*delay time  */

	/*Read the sample results 24bit*/
    buf[0] = ADS1256_Recive8Bit();
    buf[1] = ADS1256_Recive8Bit();
    buf[2] = ADS1256_Recive8Bit();

    read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
    read |= ((uint32_t)buf[1] << 8);  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
    read |= buf[2];

	CS_1();	/* SPIƬѡ = 1 */

	/* Extend a signed number*/
    if (read & 0x800000)
    {
	    read |= 0xFF000000;
    }

	return (int32_t)read;
}


/*
*********************************************************************************************************
*	name: ADS1256_GetAdc
*	function: read ADC value
*	parameter:  channel number 0--7
*	The return value:  ADC vaule (signed number)
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(uint8_t _ch)
{
	int32_t iTemp;

	if (_ch > 7)
	{
		return 0;
	}

	iTemp = g_tADS1256.AdcNow[_ch];

	return iTemp;
}

/*
*********************************************************************************************************
*	name: ADS1256_ISR
*	function: Collection procedures
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
void ADS1256_ISR(void)
{
	if (g_tADS1256.ScanMode == 0)	/*  0  Single-ended input  8 channel�� 1 Differential input  4 channe */
	{

		ADS1256_SetChannal(g_tADS1256.Channel);	/*Switch channel mode */
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[7] = ADS1256_ReadData();
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();
		}

		if (++g_tADS1256.Channel >= 8)
		{
			g_tADS1256.Channel = 0;
		}
	}
	else	/*DiffChannal*/
	{

		ADS1256_SetDiffChannal(g_tADS1256.Channel);	/* change DiffChannal */
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_SYNC);
		bsp_DelayUS(5);

		ADS1256_WriteCmd(CMD_WAKEUP);
		bsp_DelayUS(25);

		if (g_tADS1256.Channel == 0)
		{
			g_tADS1256.AdcNow[3] = ADS1256_ReadData();
		}
		else
		{
			g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();
		}

		if (++g_tADS1256.Channel >= 4)
		{
			g_tADS1256.Channel = 0;
		}
	}
}

/*
*********************************************************************************************************
*	name: ADS1256_Scan
*	function:
*	parameter:NULL
*	The return value:  1
*********************************************************************************************************
*/
uint8_t ADS1256_Scan(void)
{
	if (DRDY_IS_LOW())
	{
		ADS1256_ISR();
		return 1;
	}

	return 0;
}
/*
*********************************************************************************************************
*	name: Voltage_Convert
*	function:  Voltage value conversion function
*	parameter: Vref : The reference voltage 3.3V or 5V
*			   voltage : output DAC value
*	The return value:  NULL
*********************************************************************************************************
*/
uint16_t Voltage_Convert(float Vref, float voltage)
{
	uint16_t _D_;
	_D_ = (uint16_t)(65536 * voltage / Vref);

	return _D_;
}
/*
*********************************************************************************************************
*	name: Voltage_Convert
*	function:  Voltage value conversion function
*	parameter: Vref : The reference voltage 3.3V or 5V
*			   voltage : output DAC value
*	The return value:  NULL
*********************************************************************************************************
*/
void storeVolt(int32_t Vin){
		FILE * fp;
  // store voltage and time
  time_t t = time(NULL) + 36000; //current time in seconds adding 10 hours
  struct tm tm = *localtime(&t);

  fp = fopen ("VoltageReadings.csv", "a+");

  fprintf(fp, "%d-%d-%d %d:%d:%d,%ld.%03ld\n", tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900,
    tm.tm_hour, tm.tm_min, tm.tm_sec, Vin / 1000000, (Vin%1000000)/1000);

  fclose(fp);
}
/*
*********************************************************************************************************
*	name: LMP90100_ReadChannel
*	function: Write the corresponding register
*	parameter: NULL
*
*	The return value: ADC channel scanned
*********************************************************************************************************
*/
static int  LMP90100_ReadChannel(void)
{
	uint8_t buf[4];
	uint8_t ans;

	buf[0] = 0x10;  // Set Upper nibble of register to read
	buf[1] = 0x01;  // Upper nibble is 1
	buf[2] = 0x89;  // Read 0x19 register
	buf[3] = 0x00;  // Transmit zero.. Response will be written to this value
	bcm2835_spi_transfern(buf,4);  // Transmit set value of buf, write response into buf for each byte sent.
	ans = buf[3] & 0x07;  // Read 3 LSBs of 0x19 register
	return ans;
}

/*
*********************************************************************************************************
*	name: LMP90100_ReadADC
*	function:  Read the ADC registers 1A,1B,1C (24bit) and calculate temperature
*	parameter: NULL
*
*	The return value: Temperature
*********************************************************************************************************
*/
static float LMP90100_ReadADC(void)
{
    float  temp_calc;
	uint8_t buf[6];
	int32_t adc;

	buf[0] = 0x10;
	buf[1] = 0x01;
	buf[2] = 0xCA;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x00;
	bcm2835_spi_transfern(buf,6);

	adc =  (((uint32_t) buf[3]) << 16);
	adc += (((uint32_t) buf[4]) <<  8);
    adc += (((uint32_t) buf[5]) <<  0);

    temp_calc = (((float)adc) /16777216*4015 - 100)/0.3925;

    return temp_calc;
}
/*
*********************************************************************************************************
*	name: LMP90100_DRDY
*	function: Detect ADC ready pulse on MISO line and initate ADC read and ADC channel number read
*	parameter: NULL
*
*	The return value: 1 on success else return 0
*********************************************************************************************************
*/

static unsigned int LMP90100_DRDY (void)
{
	int Channel;
	float Temp_Reading;
	int result;
    static int ctr = 0;


	 if (LMP_DRDY_IS_LOW() && LMP_CS_IS_LOW())
	  {
			printf("%s\n", "sort of working");
        if (ctr > 1)
        {

          Temp_Reading = LMP90100_ReadADC();
          Channel = LMP90100_ReadChannel();
          printf("Ch:%02X Temp: %3.1f \n",Channel,Temp_Reading);
					printf("%s\n", "Not working");

          ctr = 0;
          result = 1;
        }
        else
        {
        	ctr = 0;
        	result = 0;
        }

	  }

	 if (LMP_DRDY_IS_HIGH() && LMP_CS_IS_LOW() && (ctr == 0))
	  {
		ctr++;

	  }

	 if (LMP_DRDY_IS_HIGH() && LMP_CS_IS_LOW() && (ctr >= 1))
	  {

		ctr++;
		//printf("Detect %02X\r",ctr);

	   }

	 if (LMP_DRDY_IS_HIGH() && LMP_CS_IS_HIGH() && (ctr >= 1))
	 {
		 ctr = 0;
	 }

	//bsp_DelayUS(5000);
	return result;
}
/*
*********************************************************************************************************
*	name: main
*	function:
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/

int  main()
{
  unsigned int cs_state = 1;
  uint8_t id;
  int32_t adc[8];
	int32_t volt[8];
	int32_t Vin;
	uint8_t i;
	uint8_t ch_num;
	uint8_t buf[3];
    if (!bcm2835_init())
        return 1;
  uint8_t setup_buf[16];

    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);   //default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                //default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8192);//default

    bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);//
    bcm2835_gpio_write(SPICS, HIGH);
    bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);

  ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_15SPS);
  ADS1256_StartScan(0);
	ch_num = 8; // number of channels.

      LMP_CS_0();
      CS_1();
      setup_buf[0] = 0x10;
      setup_buf[1] = 0x01;
      setup_buf[2] = 0x02;
      setup_buf[3] = 0x0A;   //Set current source to 1mA
      setup_buf[4] = 0x0F;
      setup_buf[5] = 0x98;   //Set continuous scan on CH0 - CH3 only.
      setup_buf[6] = 0x10;
      setup_buf[7] = 0x02;
      setup_buf[8] = 0x01;
      setup_buf[9] = 0x60;
      setup_buf[10] = 0x03;
      setup_buf[11] = 0x60;
      setup_buf[12] = 0x05;
      setup_buf[13] = 0x60;
      setup_buf[14] = 0x07;
      setup_buf[15] = 0x60;

      bcm2835_spi_transfern(setup_buf,16);
      LMP_CS_1();
      CS_0();

		while(1){

	    while((ADS1256_Scan() == 0));

			for (i = 0; i < ch_num; i++)
			{
				adc[i] = ADS1256_GetAdc(i);
	              	 volt[i] = (adc[i] * 100) / 167;
			}

			Vin = (volt[7] - volt[6]) / 8 * ((1000 + 100000) / 1000); /* uV  */

			if (Vin < 0){
				Vin = -Vin;
				printf("-%ld.%03ld %03ld V \r\n", Vin / 1000000, (Vin%1000000)/1000, Vin%1000);
        storeVolt(Vin);
			}
			else{
				printf("%ld.%03ld %03ld V \r\n", Vin / 1000000, (Vin%1000000)/1000, Vin%1000);
				storeVolt(Vin);
			}

			printf("\33[%dA", 1);

			bsp_DelayUS(3000);
			CS_1();

      //if (LMP90100_DRDY())
    	if (cs_state == 1)
    	{
         LMP_CS_0();
    	   cs_state = 0;
    	}

    	if (LMP90100_DRDY())
    	{
    		if (cs_state == 0)
    		{
          LMP_CS_1();
    			cs_state = 1;
    		}
			}
    		bsp_DelayUS(3000);
			  CS_0();
				LMP_CS_1();
		}
    bcm2835_spi_end();
    bcm2835_close();

    return 0;
}
