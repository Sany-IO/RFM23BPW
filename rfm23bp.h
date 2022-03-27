/*
 * rfm23.h
 *
 *  Created on: Mar 16, 2022
 *      Author: daniel
 */

#pragma once

#include <stdbool.h>

#if (defined STM32L011xx) || (defined STM32L021xx) || \
	(defined STM32L031xx) || (defined STM32L041xx) || \
	(defined STM32L051xx) || (defined STM32L052xx) || (defined STM32L053xx) || \
	(defined STM32L061xx) || (defined STM32L062xx) || (defined STM32L063xx) || \
	(defined STM32L071xx) || (defined STM32L072xx) || (defined STM32L073xx) || \
	(defined STM32L081xx) || (defined STM32L082xx) || (defined STM32L083xx)
#include "stm32l0xx_hal.h"
#elif defined (STM32L412xx) || defined (STM32L422xx) || \
	defined (STM32L431xx) || (defined STM32L432xx) || defined (STM32L433xx) || defined (STM32L442xx) || defined (STM32L443xx) || \
	defined (STM32L451xx) || defined (STM32L452xx) || defined (STM32L462xx) || \
	defined (STM32L471xx) || defined (STM32L475xx) || defined (STM32L476xx) || defined (STM32L485xx) || defined (STM32L486xx) || \
    defined (STM32L496xx) || defined (STM32L4A6xx) || \
    defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
#include "stm32l4xx_hal.h"
#elif defined (STM32F405xx) || defined (STM32F415xx) || defined (STM32F407xx) || defined (STM32F417xx) || \
    defined (STM32F427xx) || defined (STM32F437xx) || defined (STM32F429xx) || defined (STM32F439xx) || \
    defined (STM32F401xC) || defined (STM32F401xE) || defined (STM32F410Tx) || defined (STM32F410Cx) || \
    defined (STM32F410Rx) || defined (STM32F411xE) || defined (STM32F446xx) || defined (STM32F469xx) || \
    defined (STM32F479xx) || defined (STM32F412Cx) || defined (STM32F412Rx) || defined (STM32F412Vx) || \
    defined (STM32F412Zx) || defined (STM32F413xx) || defined (STM32F423xx)
#include "stm32f4xx_hal.h"
#elif defined (TESTING)
#include "testing/mock_hal.h"
#else
#error Platform not implemented
#endif

// Each of 16 RFM23 interrups is enabled by a bit in regs 5 & 6 (Int enable
// regs 1 & 2). POR setting: only enpor enabled.
// RFM23 sets nIRQ pin =0 until Arduino reads interrupt regs 3 and(or?) 4.

// RFM23 register 3(R): interrupt status register 1
#define RFM23_03h_ISR1				0x03
#define RFM23_03h_ISR1_IFFERR		0x00 | (1 << 7)
#define RFM23_03h_ISR1_ITXFFAFULL	0x00 | (1 << 6)
#define RFM23_03h_ISR1_ITXFFAEM		0x00 | (1 << 5)
#define RFM23_03h_ISR1_IRXFFAFULL	0x00 | (1 << 4)
#define RFM23_03h_ISR1_IEXT			0x00 | (1 << 3)
#define RFM23_03h_ISR1_IPKSENT		0x00 | (1 << 2)
#define RFM23_03h_ISR1_IPKVALID		0x00 | (1 << 1)
#define RFM23_03h_ISR1_ICRCERROR	0x00 | (1 << 0)

// RFM23 register 4(R): interrupt status register 2
#define RFM23_04h_ISR2				0x04
#define RFM23_04h_ISR2_ISWDET		0x00 | (1 << 7)
#define RFM23_04h_ISR2_IPREAVAL		0x00 | (1 << 6)
#define RFM23_04h_ISR2_IPREAINVAL	0x00 | (1 << 5)
#define RFM23_04h_ISR2_IRSSI		0x00 | (1 << 4)
#define RFM23_04h_ISR2_IWUT			0x00 | (1 << 3)
#define RFM23_04h_ISR2_ILBD			0x00 | (1 << 2)
#define RFM23_04h_ISR2_ICHIPRDY		0x00 | (1 << 1)
#define RFM23_04h_ISR2_IPOR			0x00 | (1 << 0)


#define RF22_REG_26_RSSI                                0x26
#define RF22_REG_27_RSSI_THRESHOLD                      0x27
#define RF22_REG_28_ANTENNA_DIVERSITY1                  0x28
#define RF22_REG_29_ANTENNA_DIVERSITY2                  0x29

// RFM23 register 4(RW): Interrupt Enable 1
// single bit are equal as in regs 3 and 4
#define RFM23_05h_ENIR1				0x05
// RFM23 register 5(RW): Interrupt Enable 2
#define RFM23_06h_ENIR2				0x06

// RFM23 operating modes are set & read by register 7 (RW):
// IDLE (5 submodes: standby, sleeep, sensor, ready, tune), TX, RX.
// Mode SHUTDOWN controlled by SDN pin, connected to gnd so never shutdown.
// From standby, sleep, or sensor submode, 0.8ms delay to TX or RX.
// From ready or tune submode, 0.2ms delay to TX or RX.
#define RFM23_07h_OPMODE			0x07
// Reg 7 bits:
#define RFM23_07h_OPMODE_SWRES		0x00 | (1 << 7)
#define RFM23_07h_OPMODE_ENLBD		0x00 | (1 << 6)
#define RFM23_07h_OPMODE_ENWT		0x00 | (1 << 5)
#define RFM23_07h_OPMODE_X32KSEL	0x00 | (1 << 4)
#define RFM23_07h_OPMODE_TXON		0x00 | (1 << 3)    // 1->TX state
#define RFM23_07h_OPMODE_RXON		0x00 | (1 << 2)    // 1->RX state
#define RFM23_07h_OPMODE_PLLON		0x00 | (1 << 1)
#define RFM23_07h_OPMODE_XTON		0x00 | (1 << 0)

#define RFM23_STATUS_INTERRUPT		0x00 | (1 << 0)

class RFM23
{
public:
	RFM23(SPI_HandleTypeDef *spi_handle);
	RFM23();
	void init();
	bool test();
	void setFrequency(float freq);
	void setFHStepSize(uint8_t fhs);
	void setFHChannel(uint8_t fhch);
	uint8_t rssiRead();
	uint8_t ezmacStatusRead();
	void setPromiscuous();
	bool available();
	void waitAvailable();
	bool waitAvailableTimout(uint8_t timeout);

	uint8_t headerTo();
	uint8_t headerFrom();
	uint8_t headerId();
	uint8_t headerFlags();
	uint8_t lastRssi();

	void setHeaderTo(uint8_t to);
	void setHeaderFrom(uint8_t from);
	void setHeaderId(uint8_t id);
	void setHeaderFlags(uint8_t flags);
	void setTxPower(uint8_t pwr);

	void handle_interrupt();
	void mode_ready();
	void clear_rxfifo();
	void clear_txfifo();
	void sendPaket(uint8_t addr, uint8_t data[], uint8_t len);
	void setAddress(uint8_t address);
	void receive(uint8_t data[], uint8_t len);
	uint8_t get_packetLength();
	void enable_interrupt_1(uint8_t ir);
	void enable_interrupt_2(uint8_t ir);
	uint8_t get_isr_1();
	uint8_t get_isr_2();
	void mode_rx();
	void mode_tx();
	uint8_t read_register(uint8_t addr);
	void write_register(uint8_t addr, uint8_t val);
	void read_burst(uint8_t addr, uint8_t *reg, uint8_t len);
	void write_burst(uint8_t addr, uint8_t val[], uint8_t len);
private:

	uint8_t _mode;
	uint8_t _idleMode;
	uint8_t _buf[64];

	volatile uint8_t __txPacketSent;
	volatile uint8_t _lastRssi;
	SPI_HandleTypeDef *rfm_handle;
};
