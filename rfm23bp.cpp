	/*
 * rfm23.c
 *
 *  Created on: 16.03.2022
 *      Author: Daniel Steiner
 */


#include "main.h"
#include "rfm23bp.h"
#include <cassert>
#include <math.h>

XTEA *xtea;

// interrupt status register
volatile static uint8_t RFM23_ISR1 = 0x00;
volatile static uint8_t RFM23_ISR2 = 0x00;

#define RF22_SPI_WRITE_MASK 0x80

// status variable (only internal usage)
volatile static uint8_t RFM23_STATUS = 0x00;
#define RFM23_STATUS_INTERRUPT		0x00 | (1 << 0)

SPI_HandleTypeDef *hspi;

RFM23::RFM23()
{}



bool RFM23::init(SPI_HandleTypeDef *spi, uint32_t khz)
{
	uint8_t fb;
	uint16_t fc;
	uint32_t freq = 434000;

	hspi = spi;
	uint8_t sync_nibble[2] = {0x2D, 0xD4};

	if(this->read_register(RH_RF22_REG_00_DEVICE_TYPE) != 0x8)
		return false;

	uint8_t chipVersion = this->read_register(0x01);
	uint8_t chipState = this->read_register(RH_RF22_REG_02_DEVICE_STATUS);

	if(!this->test())
		return false;

	this->swReset();

	this->write_register(RH_RF22_REG_05_INTERRUPT_ENABLE1, RH_RF22_ENTXFFAEM | RH_RF22_ENRXFFAFULL | RH_RF22_ENPKSENT | RH_RF22_ENPKVALID | RH_RF22_ENCRCERROR | RH_RF22_ENFFERR);
	this->write_register(RH_RF22_REG_06_INTERRUPT_ENABLE2, RH_RF22_ENPREAVAL);

	this->mode_ready();

	chipState = this->read_register(RH_RF22_REG_02_DEVICE_STATUS);

	this->write_register(RH_RF22_REG_7D_TX_FIFO_CONTROL2, RH_RF22_TXFFAEM_THRESHOLD);
	this->write_register(RH_RF22_REG_7E_RX_FIFO_CONTROL, RH_RF22_RXFFAFULL_THRESHOLD);

	this->write_register(RH_RF22_REG_30_DATA_ACCESS_CONTROL, RH_RF22_ENPACRX | RH_RF22_ENPACTX | RH_RF22_ENCRC | (RH_RF22_CRC_CRC_16_IBM & RH_RF22_CRC));
	this->write_register(RH_RF22_REG_32_HEADER_CONTROL1, RH_RF22_BCEN_HEADER3 | RH_RF22_HDCH_HEADER3);
	this->write_register(RH_RF22_REG_33_HEADER_CONTROL2, RH_RF22_HDLEN_4 | RH_RF22_SYNCLEN_2);

	this->setPreambleLength(8);
	uint8_t syncWords[] = {0x2d,0x4d};

	this->setTxPower(RH_RF22_RF23BP_TXPOW_28DBM);

	this->setSyncWords(syncWords, sizeof(syncWords));
	this->setPromiscuous(false);
	this->setFrequency(434.00, 0.05);

	// 57.6kbps

	this->write_register(0x1C,0x06);	// IF Bandwidth
	this->write_register(0x1F,0x03);	// AFC Timing
	this->write_register(0x20,0x45);	// clock recovery oversampling rate
	this->write_register(0x21,0x01);	// clock recovery offset 2
	this->write_register(0x22, 0xD7); 	// clock recovery offset 1
	this->write_register(0x23, 0xDC);	// clock recovery offset 0
	this->write_register(0x24, 0x07); 	// clock recovery timing loop gain 1
	this->write_register(0x25, 0x6E); 	// clock recovery timing loop gain 0
	this->write_register(0x2C, 0x40); 	// (txdr[15:8] = 0x0E)
	this->write_register(0x2D, 0x0A); 	// (txdr[7:0] = 0xBF)
	this->write_register(0x2E, 0x2D); 	// (txdtrtscale = 0, enphpwdn = 0, manppol = 1, enmaninv = 1, enmanch = 0, enwhite = 1)
	this->write_register(0x58, 0x80); 	// (trclk = 0, dtmod = 2, eninv = 0, fd[8] = 0, modtyp = 3)
	this->write_register(0x69, 0x60); 	// AGC on
	this->write_register(0x6E, 0x0E);	// set baud high
	this->write_register(0x6F, 0xBF);	// set baud low
	this->write_register(0x70, 0x0C);	// modulation control
	this->write_register(0x71, 0x23);	// modulation control 2
	this->write_register(0x72, 0x2E);	// frequency deviation


	this->write_register(RH_RF22_REG_0B_GPIO_CONFIGURATION0, 0x12);
	this->write_register(RH_RF22_REG_0C_GPIO_CONFIGURATION1, 0x15);

	return true;
}

void RFM23::sendPaket(uint8_t addr, uint8_t data[], uint8_t len)
{
	this->mode_rx();
	this->write_register(RH_RF22_REG_3A_TRANSMIT_HEADER3, 0xff);
	this->write_register(RH_RF22_REG_3B_TRANSMIT_HEADER2, 0xff);
	this->write_register(RH_RF22_REG_3C_TRANSMIT_HEADER1, 0x00);
	this->write_register(RH_RF22_REG_3D_TRANSMIT_HEADER0, 0x00);

	this->clear_rxfifo();
	this->clear_txfifo();


	this->write_burst(RH_RF22_REG_7F_FIFO_ACCESS, data, len);
	this->write_register(RH_RF22_REG_3E_PACKET_LENGTH, len);
	this->mode_tx();

}

void RFM23::swReset()
{
	this->write_register(RH_RF22_REG_07_OPERATING_MODE1, RH_RF22_SWRES);

	HAL_Delay(5);
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
	HAL_Delay(2);
}

void RFM23::mode_rx()
{
	this->write_register(RH_RF22_REG_07_OPERATING_MODE1, RH_RF22_RXON);
	HAL_Delay(2);
}

void RFM23::mode_tx()
{
	this->write_register(RH_RF22_REG_07_OPERATING_MODE1, RH_RF22_TXON);
	HAL_Delay(2);
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

	HAL_Delay(16);
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

	addr &= ~(1<<7);

	if(HAL_SPI_Transmit(hspi, &addr, 1, 100) != HAL_OK)
		return false;

	if(HAL_SPI_Receive(hspi, &val,1,100) != HAL_OK)
		return false;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return val;

}

bool RFM23::write_register(uint8_t addr, uint8_t val)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	uint8_t ret;

	addr |= (1<<7);

	if(HAL_SPI_Transmit(hspi, &addr, 1,100) != HAL_OK)
		return false;

	if(HAL_SPI_Transmit(hspi, &val, 1, 100) != HAL_OK)
		return false;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return true;
}

uint8_t RFM23::spiWrite(uint8_t addr)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	uint8_t ret;

	addr |= (1<<7);

	if(HAL_SPI_Transmit(hspi, &addr, 1,100) != HAL_OK)
		return false;

	if(HAL_SPI_Receive(hspi, &ret, 1, 100) != HAL_OK)
		return false;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return ret;
}

void RFM23::read_burst(uint8_t addr, uint8_t *reg, uint8_t len)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	addr &= ~(1<<7);

	if(HAL_SPI_Transmit(hspi, &addr,1,100) != HAL_OK)
		return;

	for(uint8_t i=0; i < len; i++)
	{
		reg[i] = this->spiWrite(0x00);
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void RFM23::write_burst(uint8_t addr, uint8_t val[], uint8_t len)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	addr |= (1<<7);

	if(HAL_SPI_Transmit(hspi, &addr, 1,100) != HAL_OK)
	{
		for(uint8_t i=0; i < len; i++)
		{
			if(HAL_SPI_Transmit(hspi, &val[i], 1,100) != HAL_OK)
				return;
		}

	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}


uint8_t RFM23::getPacketLength()
{
	this->read_register(0x4b);
}

void RFM23::waitPacketSent()
{
	this->handle_interrupt();

	while(!(this->get_isr_1() & (1<< RH_RF22_IPKSENT)))
		this->handle_interrupt();
}

uint8_t RFM23::getTemperature()
{
	this->write_register(0x0f, 0x00 | (1 << 6) | (1 << 5) | (1 << 4) );
	this->write_register(0x12, 0x00 | ( 1 << 5) );
	this->write_register(0x0f, 0x00 | ( 1 << 7));

	while(!this->read_register(0x0f) & ( 1 << 7));

	return this->read_register(0x11);
}
