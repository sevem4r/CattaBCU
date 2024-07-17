/*
 * common.h
 *
 *  Created on: 12.11.2020
 *      Author: mariusz
 */

#ifndef COMMON_H_
#define COMMON_H_

#define F_CPU	16000000

#define GPIO_INPUT		0x00
#define GPIO_OUTPUT		0x01
#define GPIO_AF			0x02
#define GPIO_ANALOG		0x03

#define GPIO_CLR1B		0x01
#define GPIO_CLR2B		0x03

#define GPIO_NO_PULL	0x00
#define GPIO_PULL_UP	0x01
#define GPIO_PULL_DOWN	0x02

#define GPIO_PP			0x00
#define	GPIO_OC			0x01

#define GPIO_SLOW		0x00
#define GPIO_SMEDIUM	0x01
#define GPIO_SHIGH		0x03

#define GPIO_AF_CLR		0x0F
#define GPIO_AF0		0x00
#define GPIO_AF1		0x01
#define GPIO_AF2		0x02
#define GPIO_AF3		0x03
#define GPIO_AF4		0x04
#define GPIO_AF5		0x05
#define GPIO_AF6		0x06
#define GPIO_AF7		0x07

#define GPIO_SET(port, pin) port->ODR |= (1<<pin)
#define GPIO_CLR(port, pin) port->ODR &= ~(1<<pin)
#define GPIO_TOG(port, pin) port->ODR ^= (1<<pin)

#define ERROR_SPI1		0
#define ERROR_ADC1		1
#define ERROR_CAN1		2
#define ERROR_USART1	3
#define ERROR_USART2	4
#define ERROR_CLOCK		5

#define ERROR_CODE_SPI1_TX_NOT_EMPTY	0
#define ERROR_CODE_SPI1_BUSY			1

#define ERROR_CODE_ADC1_HSI_INIT		0
#define ERROR_CODE_ADC1_ENABLE			1
#define ERROR_CODE_ADC1_CALIBRATE		2
#define ERROR_CODE_ADC1_CONVERSION		3

#define ERROR_CODE_CAN1_INIT				0
#define ERROR_CODE_CAN1_MAILBOX0_NOT_EMPTY	1

#define ERROR_CODE_USART1_SEND			0
#define ERROR_CODE_USART2_SEND			0

#define ERROR_CLOCK_HSE					0

void gpio_set_in(GPIO_TypeDef *port, uint8_t pin, uint8_t pull);
void gpio_set_out(GPIO_TypeDef *port, uint8_t pin);

void timeout_register_tim(TIM_TypeDef *tim);
void timeout_set(uint32_t time);
uint8_t timeout_get();



void set_error_event(void (*callback)(uint8_t src, uint8_t code));
void set_error_show_event(void (*callback)(uint8_t src, uint8_t code));
void error_call(uint8_t src, uint8_t code);
void error_show();
uint8_t errors_number();

// GPIOA
#define PA0		0
#define PA1		1
#define PA2		2
#define PA3		3
#define PA4		4
#define PA5		5
#define PA6		6
#define PA7		7

#define PA8		8
#define PA9		9
#define PA10	10
#define PA11	11
#define PA12	12
#define PA13	13
#define PA14	14
#define PA15	15

// GPIOB
#define PB0		0
#define PB1		1
#define PB2		2
#define PB3		3
#define PB4		4
#define PB5		5
#define PB6		6
#define PB7		7

#define PB8		8
#define PB9		9
#define PB10	10
#define PB11	11
#define PB12	12
#define PB13	13
#define PB14	14
#define PB15	15


#endif /* COMMON_H_ */
