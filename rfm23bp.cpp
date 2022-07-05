	/*
 * rfm23.c
 *
 *  Created on: 16.03.2022
 *      Author: Daniel Steiner
 */

/**
	Code is for STM32F407xx
	You must set a Timer like TIM2 for delay_us function.
**/

#include "main.h"
#include "xtea.h"
#include "rfm23bp.h"
#include <cassert>
#include "usbd_cdc_if.h"
#include <math.h>
#include "cmsis_os2.h"

XTEA *xtea;

// interrupt status register
volatile static uint8_t RFM23_ISR1 = 0x00;
volatile static uint8_t RFM23_ISR2 = 0x00;

#define RF22_SPI_WRITE_MASK 0x80

// status variable (only internal usage)
volatile static uint8_t RFM23_STATUS = 0x00;
#define RFM23_STATUS_INTERRUPT		0x00 | (1 << 0)

SPI_HandleTypeDef hspi;

RFM23::RFM23()
{}



bool RFM23::init()
{
	  /* USER CODE BEGIN SPI1_Init 0 */

	  /* USER CODE END SPI1_Init 0 */

	  /* USER CODE BEGIN SPI1_Init 1 */

	  /* USER CODE END SPI1_Init 1 */
	  /* SPI1 parameter configuration*/
	  hspi.Instance = SPI1;
	  hspi.Init.Mode = SPI_MODE_MASTER;
	  hspi.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi.Init.NSS = SPI_NSS_SOFT;
	  hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	  hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi.Init.CRCPolynomial = 10;

	  if (HAL_SPI_Init(&hspi) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN SPI1_Init 2 */

	  /* USER CODE END SPI1_Init 2 */
	  delay_us(500);

	  this->swReset();

	  while(!(this->read_register(RH_RF22_REG_04_INTERRUPT_STATUS2) & RH_RF22_ICHIPRDY))
	  		;

	if(this->read_register(RH_RF22_REG_00_DEVICE_TYPE) != 0x8)
		return false;

	this->swReset();




	this->write_register(RH_RF22_REG_05_INTERRUPT_ENABLE1, RH_RF22_ENTXFFAEM | RH_RF22_ENRXFFAFULL | RH_RF22_ENPKSENT | RH_RF22_ENPKVALID | RH_RF22_ENCRCERROR | RH_RF22_ENFFERR);
	this->write_register(RH_RF22_REG_06_INTERRUPT_ENABLE2, RH_RF22_ENPREAVAL);

	this->mode_ready();

	this->write_register(RH_RF22_REG_7D_TX_FIFO_CONTROL2, RH_RF22_TXFFAEM_THRESHOLD);
	this->write_register(RH_RF22_REG_7E_RX_FIFO_CONTROL, RH_RF22_RXFFAFULL_THRESHOLD);

	this->write_register(RH_RF22_REG_30_DATA_ACCESS_CONTROL, RH_RF22_ENPACRX | RH_RF22_ENPACTX | RH_RF22_ENCRC | (RH_RF22_CRC_CRC_16_IBM & RH_RF22_CRC));
	this->write_register(RH_RF22_REG_32_HEADER_CONTROL1, RH_RF22_BCEN_HEADER3 | RH_RF22_HDCH_HEADER3);
	this->write_register(RH_RF22_REG_33_HEADER_CONTROL2, RH_RF22_HDLEN_4 | RH_RF22_SYNCLEN_2);

	this->setPreambleLength(8);
	uint8_t syncWords[] = {0x2d,0xd4};

	//;

	this->setSyncWords(syncWords, sizeof(syncWords));
	this->setPromiscuous(false);
	this->setFrequency(434.00, 0.05);

	// 57.6kbps

	uint8_t modem_reg20[6] = {0x45,0x01,0xD7,0xDC,0x07,0x6E};
	uint8_t modem_reg2c[3] = {0x40, 0x0A, 0x2D};
	uint8_t modem_reg6e[5] = {0x0E,0xBF,0x0C,0x23,0x2E};


	this->write_register(0x1C,0x06);	// IF Bandwidth
	this->write_register(0x1F,0x03);	// AFC Timing
	this->write_burst(RH_RF22_REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE, modem_reg20, 6);
	this->write_burst(RH_RF22_REG_2C_OOK_COUNTER_VALUE_1, modem_reg2c, 3);
	this->write_register(0x58, 0x80); 	// (trclk = 0, dtmod = 2, eninv = 0, fd[8] = 0, modtyp = 3)
	this->write_register(0x69, 0x60); 	// AGC on
	this->write_burst(RH_RF22_REG_6E_TX_DATA_RATE1, modem_reg6e, 5);


	this->write_register(RH_RF22_REG_0B_GPIO_CONFIGURATION0, 0x12);
	this->write_register(RH_RF22_REG_0C_GPIO_CONFIGURATION1, 0x15);

	this->setTxPower(RH_RF22_RF23BP_TXPOW_30DBM);
	return true;
}

void RFM23::sendPaket(uint8_t addr, uint8_t *data, uint8_t len)
{
	HAL_GPIO_WritePin(LED_LINK_TX_GPIO_Port, LED_LINK_TX_Pin, GPIO_PIN_SET);
	this->write_register(RH_RF22_REG_3A_TRANSMIT_HEADER3, 0xff);
	this->write_register(RH_RF22_REG_3B_TRANSMIT_HEADER2, 0xff);
	this->write_register(RH_RF22_REG_3C_TRANSMIT_HEADER1, 0);
	this->write_register(RH_RF22_REG_3D_TRANSMIT_HEADER0, 0);

	this->write_burst(RH_RF22_REG_7F_FIFO_ACCESS, data, len);
	this->write_register(RH_RF22_REG_3E_PACKET_LENGTH, len);

	this->mode_tx();
	HAL_GPIO_WritePin(LED_LINK_TX_GPIO_Port, LED_LINK_TX_Pin, GPIO_PIN_RESET);

}

void RFM23::swReset()
{
	this->write_register(RH_RF22_REG_07_OPERATING_MODE1, RH_RF22_SWRES);
}

bool RFM23::test()
{
	uint8_t reg = 0x05;
	uint8_t value = 0xEE;

	uint8_t val_orig = this->read_register(reg);
	this->write_register(reg, value);

	uint8_t val_new = this->read_register(reg);
	this->write_register(reg, val_orig);

	if(val_new == value)
	 	 return true;
	else
		return false;
}

void RFM23::setPromiscuous(bool prom)
{
	this->write_register(RH_RF22_REG_43_HEADER_ENABLE3, prom ? 0x00 : 0xff);
}

void RFM23::setTxPower(uint8_t power)
{
	this->write_register(RH_RF22_REG_6D_TX_POWER,  power | RH_RF22_LNA_SW);
}

void RFM23::setSyncWords(uint8_t* syncWords, uint8_t len)
{
	this->write_burst(RH_RF22_REG_36_SYNC_WORD3, syncWords, len);
}

void RFM23::receive(uint8_t data[], uint8_t len)
{
	this->read_burst(0x7f, data, len);
}

void RFM23::setAddress(uint8_t address)
{
	// set sender address?
	this->write_register(0x3A, address);

	// check header2 on receive
	this->write_register(0x40, address);

	// only receive when header2 match
	this->write_register(0x32, 0x04);
}

void RFM23::clear_rxfifo()
{
	this->write_register(0x08, 0x02);
	this->write_register(0x08, 0x00);
}

void RFM23::clear_txfifo()
{
	this->write_register(0x08, 0x01);
	this->write_register(0x08, 0x00);
}

bool RFM23::setFrequency(float centre, float afcPullInRange)
{
	uint8_t fbsel = RH_RF22_SBSEL;
	uint8_t afclimiter;

	if (centre < 240.0 || centre > 960.0) // 930.0 for early silicon
		return false;
	if (centre >= 480.0)
	{
		if (afcPullInRange < 0.0 || afcPullInRange > 0.318750)
		    return false;
		centre /= 2;
		fbsel |= RH_RF22_HBSEL;
		afclimiter = afcPullInRange * 1000000.0 / 1250.0;
	 }
	 else
	 {
		if (afcPullInRange < 0.0 || afcPullInRange > 0.159375)
		    return false;
			afclimiter = afcPullInRange * 1000000.0 / 625.0;
	 }

	centre /=10.0;
	float integerPart = floor(centre);
	float fractionalPart = centre - integerPart;

	uint8_t fb = (uint8_t) integerPart - 24;
	fbsel |= fb;
	uint16_t fc = fractionalPart * 64000;

	this->write_register(RH_RF22_REG_73_FREQUENCY_OFFSET1,0);
	this->write_register(RH_RF22_REG_74_FREQUENCY_OFFSET2,0);
	this->write_register(RH_RF22_REG_75_FREQUENCY_BAND_SELECT, fbsel);
	this->write_register(RH_RF22_REG_76_NOMINAL_CARRIER_FREQUENCY1, fc >> 8);
	this->write_register(RH_RF22_REG_77_NOMINAL_CARRIER_FREQUENCY0, fc & 0xff);
	this->write_register(RH_RF22_REG_2A_AFC_LIMITER, afclimiter);

	this->read_register(RH_RF22_REG_02_DEVICE_STATUS);

}
uint8_t RFM23::rssiRead()
{
	return this->read_register(RH_RF22_REG_26_RSSI);
}

void RFM23::setPreambleLength(uint8_t nibbles)
{
	this->write_register(RH_RF22_REG_34_PREAMBLE_LENGTH, nibbles);
}

void RFM23::mode_ready()
{
	this->write_register(RH_RF22_REG_07_OPERATING_MODE1, RH_RF22_XTON);
}

void RFM23::mode_rx()
{
	this->write_register(RH_RF22_REG_07_OPERATING_MODE1, 0x01 | RH_RF22_RXON);
}

void RFM23::mode_tx()
{
	this->write_register(RH_RF22_REG_07_OPERATING_MODE1, 0x01 | RH_RF22_TXON);
}

void RFM23::enable_interrupt_1(uint8_t ir)
{
	this->write_register(RH_RF22_REG_05_INTERRUPT_ENABLE1,ir);
}

void RFM23::enable_interrupt_2(uint8_t ir)
{
	this->write_register(RH_RF22_REG_06_INTERRUPT_ENABLE2, ir);
}

void RFM23::handle_interrupt()
{
	RFM23_ISR1 = this->read_register(RH_RF22_REG_03_INTERRUPT_STATUS1);
	RFM23_ISR2 = this->read_register(RH_RF22_REG_04_INTERRUPT_STATUS2);

	RFM23_STATUS |= (1 << RFM23_STATUS_INTERRUPT);
}

uint8_t RFM23::get_isr_1()
{
	return RFM23_ISR1;
}

uint8_t RFM23::get_isr_2()
{
	return RFM23_ISR2;
}

uint8_t RFM23::get_packetLength()
{
	this->read_register(0x4b);
}

uint8_t RFM23::read_register(uint8_t addr)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	uint8_t val = 0x00;

	if(HAL_SPI_Transmit(&hspi, (uint8_t *)&addr, 1, 100) != HAL_OK)
		return false;

	if(HAL_SPI_Receive(&hspi, (uint8_t *)&val,1,100) != HAL_OK)
		return false;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	delay_us(10);
	return val;
}

bool RFM23::write_register(uint8_t addr, uint8_t val)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	uint8_t ret;

	addr |= (1<<7);


	if(HAL_SPI_Transmit(&hspi, (uint8_t*)&addr, 1,100) != HAL_OK)
		return false;

	if(HAL_SPI_Transmit(&hspi, (uint8_t *)&val, 1, 100) != HAL_OK)
		return false;


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	delay_us(10);
	return true;
}



uint8_t RFM23::spiWrite(uint8_t addr)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	uint8_t ret;
	addr |= (1<<7);

	if(HAL_SPI_Transmit(&hspi, (uint8_t *)&addr, 1,100) != HAL_OK)
		return false;

	if(HAL_SPI_Receive(&hspi, (uint8_t *)&ret, 1, 100) != HAL_OK)
		return false;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return ret;
}

void RFM23::read_burst(uint8_t addr, uint8_t *reg, uint8_t len)
{
	uint8_t new_addr = addr;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);


	if(HAL_SPI_Transmit(&hspi, (uint8_t *)&new_addr,1,100) != HAL_OK)
		return;

	for(uint8_t i=0; i < len; i++)
	{
		reg[i] = this->spiWrite(0x00);
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	delay_us(10);
}

void RFM23::write_burst(uint8_t addr, uint8_t* val, uint8_t len)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	addr |= (1<<7);

	if(HAL_SPI_Transmit(&hspi, (uint8_t *)&addr, 1,100) == HAL_OK)
	{
		while(len--)
		{
			if(HAL_SPI_Transmit(&hspi,(uint8_t*)val++,1,100) != HAL_OK)
				printf("ERROR");
		}
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	delay_us(10);
}


uint8_t RFM23::getPacketLength()
{
	this->read_register(0x4b);
}

void RFM23::waitPacketSent()
{
	int error;

	this->handle_interrupt();
	uint8_t isr1 = this->read_register(RH_RF22_REG_03_INTERRUPT_STATUS1);
	uint8_t isr2 = this->read_register(RH_RF22_REG_04_INTERRUPT_STATUS2);

	RFM23_ISR1 = isr1;
	RFM23_ISR2 = isr2;

	if(isr1 == RH_RF22_IFFERROR)
	{
		error = 0;
	}
	else if(isr1 == RH_RF22_ITXFFAEM)
	{
		error = 1;
	}
	else if(isr1 == RH_RF22_ITXFFAFULL)
	{
		error = 2;
	}
	else if(isr1 == RH_RF22_IEXT)
	{
		error = 3;
	}
	else if(isr1 == RH_RF22_IWUT)
	{
		error = 4;
	}
	else if(isr1 == RH_RF22_IPKSENT)
	{
		error = 5;
	}
	else if(isr1 == RH_RF22_IPKVALID)
	{
		error = 6;
	}
	else if(isr1 == RH_RF22_ICRCERROR)
	{
		error = 7;
	}
	else if(isr1 == RH_RF22_IPREAVAL)
	{
		error = 8;
	}
}

uint8_t RFM23::getTemperature()
{
	this->write_register(0x0f, 0x00 | (1 << 6) | (1 << 5) | (1 << 4) );
	this->write_register(0x12, 0x00 | ( 1 << 5) );
	this->write_register(0x0f, 0x00 | ( 1 << 7));

	while(!this->read_register(0x0f) & ( 1 << 7));

	return this->read_register(0x11);
}
