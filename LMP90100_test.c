/*
 * main.c
 *
 *  Created on: 04/02/2019
 *      Author: Shane Goodwin
*/

/*
             define from bcm2835.h
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

//CS    -----   SPICS
//DIN     -----   MOSI
//DOUT  -----   MISO
//SCLK   -----   SCLK
//DRDY  -----   ctl_IO     data  starting
//RST     -----   ctl_IO     reset

#define MISO  9
#define MOSI  10
#define DRDY  17
#define SPICS  22
#define CS_1()  bcm2835_gpio_write(SPICS,HIGH)
#define CS_0()  bcm2835_gpio_write(SPICS,LOW)
#define CS_IS_LOW() (bcm2835_gpio_lev(SPICS) == 0)
#define CS_IS_HIGH() (bcm2835_gpio_lev(SPICS) == 1)
#define DRDY_IS_LOW()  (bcm2835_gpio_lev(MISO)==0)
#define DRDY_IS_HIGH() (bcm2835_gpio_lev(MISO)==1)
#define CMD_WREG 0x10
#define CMD_RREG 0x90

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

void  bsp_DelayUS(uint64_t micros)
{
		bcm2835_delayMicroseconds (micros);
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


	 if (DRDY_IS_LOW() && CS_IS_LOW())
	  {
        if (ctr > 1)
        {

          Temp_Reading = LMP90100_ReadADC();
          Channel = LMP90100_ReadChannel();
          printf("Ch:%02X Temp: %3.1f \r",Channel,Temp_Reading);
          ctr = 0;
          result = 1;
        }
        else
        {
        	ctr = 0;
        	result = 0;
        }

	  }

	 if (DRDY_IS_HIGH() && CS_IS_LOW() && (ctr == 0))
	  {
		ctr++;

	  }

	 if (DRDY_IS_HIGH() && CS_IS_LOW() && (ctr >= 1))
	  {

		ctr++;
		//printf("Detect %02X\r",ctr);

	   }

	 if (DRDY_IS_HIGH() && CS_IS_HIGH() && (ctr >= 1))
	 {
		 ctr = 0;
	 }

	//bsp_DelayUS(5000);
	return result;
}

/*
*********************************************************************************************************
*	name:  main
*	Setup SPI device then collect Temperature and ADC channel and print values on screen.
*	parameter: NULL
*
*	The return value: 0
*********************************************************************************************************
*/
int  main()
{

    unsigned int cs_state = 1;

	uint8_t setup_buf[16];
    if (!bcm2835_init())
        return 1;

		bcm2835_spi_begin();
		bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);   //default
		bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                //default
		bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_8192);//default

    bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP);//
    bcm2835_gpio_write(SPICS, HIGH);


	//Todo:  Create Setup function
    CS_0();
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
    CS_1();

    while(1)
	{


  //if (LMP90100_DRDY())
	if (cs_state == 1)
	{

	   CS_0();
	   cs_state = 0;
	}

	if (LMP90100_DRDY())
	{
		if (cs_state == 0)
		{
			CS_1();
			cs_state = 1;
		}
		bsp_DelayUS(3000);

	}
	}
    bcm2835_spi_end();
    bcm2835_close();

    return 0;
}
