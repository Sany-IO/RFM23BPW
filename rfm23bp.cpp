	/*
 * rfm23.c
 *
 *  Created on: 16.03.2022
 *      Author: Daniel Steiner
 */


#include "main.h"
#include "rfm23bp.h"
#include <cassert>

// interrupt status register
volatile static uint8_t RFM23_ISR1 = 0x00;
volatile static uint8_t RFM23_ISR2 = 0x00;

#define RF22_SPI_WRITE_MASK 0x80

// status variable (only internal usage)
volatile static uint8_t RFM23_STATUS = 0x00;
#define RFM23_STATUS_INTERRUPT		0x00 | (1 << 0)


RFM23::RFM23(SPI_HandleTypeDef *spi_handle)
{
	this->rfm_handle = spi_handle;
}

RFM23::RFM23()
{}

void RFM23::init()
{
	assert(this->rfm_handle->Init.Mode == SPI_MODE_MASTER);
	assert(this->rfm_handle->Init.Direction == SPI_DIRECTION_2LINES);
	assert(this->rfm_handle->Init.DataSize == SPI_DATASIZE_8BIT);
	assert(this->rfm_handle->Init.CLKPolarity == SPI_POLARITY_LOW);
	assert(this->rfm_handle->Init.CLKPhase == SPI_PHASE_1EDGE);

	uint8_t deviceType = this->read_register(0x00);

	this->read_register(RFM23_03h_ISR1);
	this->read_register(RFM23_04h_ISR2);
}

void RFM23::sendPaket(uint8_t addr, uint8_t data[], uint8_t len)
{
	this->write_register(0x3b, addr);
	this->clear_txfifo();

	this->write_register(0x3e, len);
	this->write_burst(addr, data, len);
	this->write_register(0x07, 0x09);

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

void RFM23::mode_ready()
{
	this->write_register(RFM23_07h_OPMODE, RFM23_07h_OPMODE_XTON);
	HAL_Delay(2);
}

void RFM23::mode_rx()
{
	this->write_register(RFM23_07h_OPMODE, RFM23_07h_OPMODE_RXON);
	HAL_Delay(2);
}

void RFM23::mode_tx()
{
	this->write_register(RFM23_07h_OPMODE, RFM23_07h_OPMODE_TXON);
	HAL_Delay(2);
}

void RFM23::enable_interrupt_1(uint8_t ir)
{
	this->write_register(RFM23_05h_ENIR1,ir);
}

void RFM23::enable_interrupt_2(uint8_t ir)
{
	this->write_register(RFM23_06h_ENIR2, ir);
}

void RFM23::handle_interrupt()
{
	RFM23_ISR1 = this->read_register(RFM23_03h_ISR1);
	RFM23_ISR2 = this->read_register(RFM23_04h_ISR2);

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

	if(HAL_SPI_Transmit_DMA(this->rfm_handle, &addr, 1) != HAL_OK)
	{
		return false;
	}

	if(HAL_SPI_Receive_DMA(this->rfm_handle, (uint8_t *) val, 1) != HAL_OK)
	{
		return false;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void RFM23::write_register(uint8_t addr, uint8_t val)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	addr |= (1<<7);

	if(HAL_SPI_Transmit_DMA(this->rfm_handle, &addr, 1) != HAL_OK)
	{
		return;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void RFM23::read_burst(uint8_t addr, uint8_t *reg, uint8_t len)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_StatusTypeDef hal_stat;

	addr &= ~(1<<7);

	hal_stat = HAL_SPI_Transmit_DMA(this->rfm_handle, &addr,1);

	if(hal_stat != HAL_OK)
		return;

	hal_stat = HAL_SPI_Receive_DMA(this->rfm_handle, reg, len);
	if(hal_stat != HAL_OK)
		return;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void RFM23::write_burst(uint8_t addr, uint8_t val[], uint8_t len)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_StatusTypeDef hal_stat;

	addr |= (1<<7);

	hal_stat = HAL_SPI_Transmit_DMA(this->rfm_handle, &addr, 1);

	hal_stat = HAL_SPI_Transmit_DMA(this->rfm_handle, val, 1);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void RFM23::setTxPower(uint8_t pwr)
{

}

uint8_t RFM23::ezmacStatusRead()
{

}

uint8_t RFM23::rssiRead()
{

}

void RFM23::setFHChannel(uint8_t fhch)
{

}

void RFM23::setFHStepSize(uint8_t fhs)
{

}

void RFM23::setFrequency(float freq)
{

}
