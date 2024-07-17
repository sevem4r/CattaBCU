/*
 * spi.c
 *
 *  Created on: 19.11.2020
 *      Author: mariusz
 */
#include "stm32f0xx.h"
#include "SPI/spi1.h"

#if SPI1_SCK_PIN < 8
#define SPI1_SCK_AFR		SPI1_SCK_PORT->AFR[0]
#define SPI1_SCK_AFR_PIN	SPI1_SCK_PIN * 0x04
#else
#define SPI1_SCK_AFR		SPI1_SCK_PORT->AFR[1]
#define SPI1_SCK_AFR_PIN	(SPI1_SCK_PIN - 8) * 0x04
#endif

#if SPI1_MISO_PIN < 8
#define SPI1_MISO_AFR		SPI1_MISO_PORT->AFR[0]
#define SPI1_MISO_AFR_PIN	SPI1_MISO_PIN * 0x04
#else
#define SPI1_MISO_AFR		SPI1_MISO_PORT->AFR[1]
#define SPI1_MISO_AFR_PIN	(SPI1_MISO_PIN - 8) * 0x04
#endif

#if SPI1_MOSI_PIN < 8
#define SPI1_MOSI_AFR		SPI1_MOSI_PORT->AFR[0]
#define SPI1_MOSI_AFR_PIN	SPI1_MOSI_PIN * 0x04
#else
#define SPI1_MOSI_AFR		SPI1_MOSI_PORT->AFR[1]
#define SPI1_MOSI_AFR_PIN	(SPI1_MOSI_PIN - 8) * 0x04
#endif

void spi1_set(void){

	// SPI1 SCK Gpio set
	SPI1_SCK_PORT_EN;
	SPI1_SCK_PORT->MODER   |= (GPIO_AF 	  << (SPI1_SCK_PIN * 2));
	SPI1_SCK_PORT->OSPEEDR |= (GPIO_SHIGH << (SPI1_SCK_PIN * 2));

    SPI1_SCK_AFR &= ~(GPIO_AF_CLR	<< SPI1_SCK_AFR_PIN);
    SPI1_SCK_AFR |=  (SPI1_SCK_AF   << SPI1_SCK_AFR_PIN);

#if SPI1_MODE == SPI1_MODE_FULL_DUPLEX_MASTER
    // SPI1 MISO Gpio set
	SPI1_MISO_PORT_EN;
    SPI1_MISO_PORT->MODER   |= (GPIO_AF    << (SPI1_MISO_PIN * 2));
	SPI1_MISO_PORT->OSPEEDR |= (GPIO_SHIGH << (SPI1_MISO_PIN * 2));
	SPI1_MISO_PORT->PUPDR   |= (GPIO_PULL_DOWN << (SPI1_MISO_PIN * 2));

    SPI1_MISO_AFR &= ~(GPIO_AF_CLR	<< SPI1_MISO_AFR_PIN);
    SPI1_MISO_AFR |=  (SPI1_MISO_AF << SPI1_MISO_AFR_PIN);
#endif

	// SPI1 MOSI Gpio set
	SPI1_MOSI_PORT_EN;
    SPI1_MOSI_PORT->MODER   |= (GPIO_AF    << (SPI1_MOSI_PIN * 2));
	SPI1_MOSI_PORT->OSPEEDR |= (GPIO_SHIGH << (SPI1_MOSI_PIN * 2));
	SPI1_MOSI_PORT->PUPDR   |= (GPIO_PULL_DOWN << (SPI1_MOSI_PIN * 2));

    SPI1_MOSI_AFR &= ~(GPIO_AF_CLR	<< SPI1_MOSI_AFR_PIN);
    SPI1_MOSI_AFR |=  (SPI1_MOSI_AF << SPI1_MOSI_AFR_PIN);

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI1->CR1 |= (SPI1_CPOL << 1);
    SPI1->CR1 |= SPI1_CPHA;
    SPI1->CR1 |= SPI_CR1_MSTR; // Master
    SPI1->CR1 |= (SPI1_FPCLK << 3);
    SPI1->CR1 |= (SPI1_FORMAT << 7);
    SPI1->CR1 |= SPI_CR1_SSI;  // Internal Slave Select
    SPI1->CR1 |= SPI_CR1_SSM;  // Soft Slave Managment

    SPI1->CR2 |= SPI_CR2_FRXTH;
    SPI1->CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; // 8bit

    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}

uint8_t spi1_transfer(uint8_t data){
	uint32_t attempts;
	uint8_t res;

	attempts = 10000;
	while(!(SPI1->SR & SPI_SR_TXE)){
		__asm("nop");

		attempts--;

		if(!attempts){
			return 0;
		}
	}

	*(uint8_t *)&SPI1->DR = data;

	attempts = 10000;
	while(SPI1->SR & SPI_SR_BSY){
		__asm("nop");

		attempts--;

		if(!attempts){
			return 0;
		}
	}

	res = *(uint8_t *)&SPI1->DR;

	return res;
}

