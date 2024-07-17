/*
 * main.c
 *
 *  Created on: 14.11.2022
 *      Author: mariusz
 */

#include "stm32f0xx.h"
#include "main.h"
#include "app.h"
#include "common.h"
#include "SPI/spi1.h"
#include "ADC/adc1.h"

#define PERIPH_INIT_TIMOUT	1000

static volatile uint32_t sys_tick_counter;
static volatile RTC_time_t sys_tick_rtc;

void sys_tick_init(void);
void HSE_init(void);
void gpio_init(void);
//void rtc_init(void);
void tim1_init(void);
void tim14_init(void);
void tim17_init(void);
void tim16_init(void);
void tim3_init(void);
void exti_init(void);

int main(void){

	sys_tick_init();

	HSE_init();

	spi1_set();
	adc1_set();

	gpio_init();
//	rtc_init();
	tim1_init();
	tim14_init();
	tim17_init();
	tim16_init();
	tim3_init();

	exti_init();

	gpio_set_out(GPIOA, GPIO_OLED_RES);
	gpio_set_out(GPIOA, GPIO_OLED_DC);
	gpio_set_out(GPIOA, GPIO_OLED_CS);

	app_init();
	app_loop();
}

//************************* CLOCK ***************************
void HSE_init(void){
	uint32_t tickstart;

	// Set memory access latency
	if(F_CPU >= 24000000){
		FLASH->ACR &= ~(0x00000017);
		FLASH->ACR |=  (FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE);
	}

	// HSE Clock enable
	RCC->CR |= RCC_CR_HSEON;

	// Wait for HSE ready
	tickstart = sys_tick_counter;
	while(!(RCC->CR & RCC_CR_HSERDY)){
		if((sys_tick_counter - tickstart) > PERIPH_INIT_TIMOUT){
			return;
		}
	}

	// Switch system clock to HSE
	RCC->CFGR = ((RCC->CFGR & (~(RCC_CFGR_SW | RCC_CFGR_HPRE))) | RCC_CFGR_SW_0 | RCC_CFGR_HPRE_3); /* (3) */
}

//************************ Memory ***************************
uint32_t flash_unlock(void){
	// (1) Wait till no operation is on going
	// (2) Check that the Flash is unlocked
	// (3) Perform unlock sequence
	while((FLASH->SR & FLASH_SR_BSY) != 0){ // (1)
		// For robust implementation, add here time-out management
	}

	if((FLASH->CR & FLASH_CR_LOCK) != 0){ // (2)
		FLASH->KEYR = FLASH_KEY1; // (3)
		FLASH->KEYR = FLASH_KEY2;
	}

	return 1;
}

uint32_t flash_lock(void){
	// Set the LOCK Bit to lock the FLASH Registers access
	FLASH->CR |= FLASH_CR_LOCK;

	return 1;
}

uint32_t flash_page_erase(uint32_t address){
	// (1) Set the PER bit in the FLASH_CR register to enable page erasing
	// (2) Program the FLASH_AR register to select a page to erase
	// (3) Set the STRT bit in the FLASH_CR register to start the erasing
	// (4) Wait until the BSY bit is reset in the FLASH_SR register
	// (5) Check the EOP flag in the FLASH_SR register
	// (6) Clear EOP flag by software by writing EOP at 1
	// (7) Reset the PER Bit to disable the page erase

	FLASH->CR |= FLASH_CR_PER; 	// (1)
	FLASH->AR = address; 		// (2)
	FLASH->CR |= FLASH_CR_STRT; // (3)
	while ((FLASH->SR & FLASH_SR_BSY) != 0){ // (4)
		// For robust implementation, add here time-out management
	}

	if((FLASH->SR & FLASH_SR_EOP) != 0){ // (5)
		FLASH->SR = FLASH_SR_EOP; // (6)
	}
	else{
		// Manage the error cases
	}

	FLASH->CR &= ~FLASH_CR_PER; // (7)

	return 1;
}

uint32_t flash_write(uint32_t address, uint16_t* data, uint32_t size){
	// (1) Set the PG bit in the FLASH_CR register to enable programming
	// (2) Perform the data write (half-word) at the desired address
	// (3) Wait until the BSY bit is reset in the FLASH_SR register
	// (4) Check the EOP flag in the FLASH_SR register
	// (5) clear it by software by writing it at 1
	// (6) Reset the PG Bit to disable programming

	for(uint32_t i = 0; i < size; i++){

		FLASH->CR |= FLASH_CR_PG; 					// (1)

		*(__IO uint16_t*)(address + 2U * i) = data[i]; // (2)

		while((FLASH->SR & FLASH_SR_BSY) != 0){ 	// (3)
			// For robust implementation, add here time-out management
		}

		if((FLASH->SR & FLASH_SR_EOP) != 0){ 	// (4)
			FLASH->SR = FLASH_SR_EOP; 			// (5)
		}
		else{
			// Manage the error cases
		}

		FLASH->CR &= ~FLASH_CR_PG; // (6)

	}

	return 1;
}

uint32_t flash_read(uint32_t address, uint16_t* data, uint32_t size){

	for(uint32_t i = 0; i < size; i++){
		data[i] = *(__IO uint16_t*)(address + 2U * i);
	}

	return 1;
}

//************************* Delay ***************************
void sys_tick_init(void){
	SysTick_Config(8000000U / 1000U);
	NVIC_SetPriority(SysTick_IRQn, 0U);
}

void sys_tick_delay_ms(uint32_t ms){
	uint32_t tickstart = sys_tick_counter;
	uint32_t wait = ms;

	// Add a freq to guarantee minimum wait
	if(wait < 0xFFFFFFFFU){
		wait += (uint32_t)(1);
	}

	while((sys_tick_counter - tickstart) < ms);
}

uint32_t sys_tick_get(void){
	return sys_tick_counter;
}

void SysTick_Handler(){
	static uint32_t ms;

	sys_tick_counter++;

	ms++;

	if(ms >= 1000){
		ms = 0;

		sys_tick_rtc.seconds++;

		if(sys_tick_rtc.seconds >= 60){
			sys_tick_rtc.seconds = 0;
			sys_tick_rtc.minutes++;

			if(sys_tick_rtc.minutes >= 60){
				sys_tick_rtc.minutes = 0;

				sys_tick_rtc.hours++;
			}
		}
	}
}

//*********************** GPIO ***************************
void gpio_init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// LIGHTS(in)
	GPIOA->MODER &= ~(0x03 << (GPIO_LIGHTS * 2));
	GPIOA->PUPDR &= ~(0x03 << (GPIO_LIGHTS * 2));
	GPIOA->MODER |= (0x00 << (GPIO_LIGHTS * 2));
	GPIOA->PUPDR |= (0x00 << (GPIO_LIGHTS * 2));

	// BRAKES(in)
	GPIOA->MODER &= ~(0x03 << (GPIO_BRAKES * 2));
	GPIOA->PUPDR &= ~(0x03 << (GPIO_BRAKES * 2));
	GPIOA->MODER |= (0x00 << (GPIO_BRAKES * 2));
	GPIOA->PUPDR |= (0x00 << (GPIO_BRAKES * 2));

	// GPIO_KEY_LEFT(in)
	GPIOB->MODER &= ~(0x03 << (GPIO_KEY_LEFT * 2));
	GPIOB->PUPDR &= ~(0x03 << (GPIO_KEY_LEFT * 2));
	GPIOB->MODER |= (0x00 << (GPIO_KEY_LEFT * 2));
	GPIOB->PUPDR |= (0x00 << (GPIO_KEY_LEFT * 2));

	// GPIO_KEY_CENTER(in)
	GPIOB->MODER &= ~(0x03 << (GPIO_KEY_CENTER * 2));
	GPIOB->PUPDR &= ~(0x03 << (GPIO_KEY_CENTER * 2));
	GPIOB->MODER |= (0x00 << (GPIO_KEY_CENTER * 2));
	GPIOB->PUPDR |= (0x00 << (GPIO_KEY_CENTER * 2));

	// GPIO_KEY_RIGHT(in)
	GPIOA->MODER &= ~(0x03 << (GPIO_KEY_RIGHT * 2));
	GPIOA->PUPDR &= ~(0x03 << (GPIO_KEY_RIGHT * 2));
	GPIOA->MODER |= (0x00 << (GPIO_KEY_RIGHT * 2));
	GPIOA->PUPDR |= (0x00 << (GPIO_KEY_RIGHT * 2));

	// AUTO_OFF(out)
	GPIOA->MODER &= ~(0x03 << (GPIO_AUTO_OFF * 2));
	GPIOA->OTYPER &= ~(0x03 << (GPIO_AUTO_OFF * 2));
	GPIOA->PUPDR &= ~(0x03 << (GPIO_AUTO_OFF * 2));
	GPIOA->MODER |= (0x01 << (GPIO_AUTO_OFF * 2));
}

////*************************** RTC *****************************
//void rtc_init(void){
//	uint32_t tickstart;
//
//	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
//
//	PWR->CR |= PWR_CR_DBP;
//
//	tickstart = sys_tick_counter;
//	while(!(PWR->CR & PWR_CR_DBP)){
//		if((sys_tick_counter - tickstart) > PERIPH_INIT_TIMOUT){
//			return;
//		}
//	}
//
//	// Write access for RTC registers
//	do{
//		RTC->WPR = 0xCAU;
//		RTC->WPR = 0x53U;
//	}while(0);
//
//	// Enter Initialization mode
//	if(!(RTC->ISR & RTC_ISR_INITF)){
//	    // Set the Initialization mode
//	    RTC->ISR = (uint32_t)0xFFFFFFFFU;
//
//	    tickstart = sys_tick_counter;
//	    while(!(RTC->ISR & (0x01<<6U))){
//			if((sys_tick_counter - tickstart) > PERIPH_INIT_TIMOUT){
//				return;
//			}
//	    }
//	}
//
//	// Select clock source
//	RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_HSE;
//
//	// Clear RTC_CR FMT, OSEL and POL Bits
//	RTC->CR &= ((uint32_t)~(RTC_CR_FMT | RTC_CR_OSEL | RTC_CR_POL));
//
//	// Asynch predivider
//	RTC->PRER = (uint32_t)(127 << 16U);
//	// Symch predivider
//	RTC->PRER |= (uint32_t)(3905);
//
//	// Exit Initialization mode
//	RTC->ISR &= (uint32_t)~RTC_ISR_INIT;
//
//	// Disable write access
//	RTC->WPR = 0xFE;
//	RTC->WPR = 0x64;
//}

//void rtc_set_time(RTC_time_t* time){
//	uint32_t tickstart;
//
//	// Write access for RTC registers
//	do{
//		RTC->WPR = 0xCAU;
//		RTC->WPR = 0x53U;
//	}while(0);
//
//	// Enter Initialization mode
//	if(!(RTC->ISR & RTC_ISR_INITF)){
//	    // Set the Initialization mode
//	    RTC->ISR = (uint32_t)0xFFFFFFFFU;
//
//	    tickstart = sys_tick_counter;
//	    while(!(RTC->ISR & (0x01<<6U))){
//			if((sys_tick_counter - tickstart) > PERIPH_INIT_TIMOUT){
//				return;
//			}
//	    }
//	}
//
//	// Set time
//	uint32_t ht = (time->hours / 10) << RTC_TR_HT_Pos;
//	uint32_t hu = (time->hours % 10) << RTC_TR_HU_Pos;
//
//	uint32_t mnt = (time->minutes / 10) << RTC_TR_MNT_Pos;
//	uint32_t mnu = (time->minutes % 10) << RTC_TR_MNU_Pos;
//
//	uint32_t st = (time->seconds / 10) << RTC_TR_ST_Pos;
//	uint32_t su = (time->seconds % 10) << RTC_TR_SU_Pos;
//
//	RTC->TR = ht | hu | mnt | mnu | st | su;
//
//	// Exit Initialization mode
//	RTC->ISR &= (uint32_t)~RTC_ISR_INIT;
//
//	// Disable write access
//	RTC->WPR = 0xFE;
//	RTC->WPR = 0x64;
//}
//
//void rtc_get_time(RTC_time_t* time){
//	time->hours = ((RTC->TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos) * 10;
//	time->hours += (RTC->TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos;
//
//	time->minutes = ((RTC->TR & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos) * 10;
//	time->minutes += (RTC->TR & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos;
//
//	time->seconds = ((RTC->TR & RTC_TR_ST_Msk) >> RTC_TR_ST_Pos) * 10;
//	time->seconds += (RTC->TR & RTC_TR_SU_Msk) >> RTC_TR_SU_Pos;
//}

void rtc_set_time(RTC_time_t* time){
	sys_tick_rtc.seconds = time->seconds;
	sys_tick_rtc.minutes = time->minutes;
	sys_tick_rtc.hours = time->hours;
}

void rtc_get_time(RTC_time_t* time){
	time->seconds = sys_tick_rtc.seconds;
	time->minutes = sys_tick_rtc.minutes;
	time->hours = sys_tick_rtc.hours;
}

//********************* TIM1 10ms interrupt **********************
void tim1_init(void){
	// Enable the TIM14 clock.
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1->CR1 &= ~(TIM_CR1_CEN);
	// Peripherial reset
	RCC->APB2RSTR |=  (RCC_APB2RSTR_TIM1RST);
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM1RST);
	// Prescaler - 16bit
	TIM1->PSC   = 7999;
	// Autoreload - 16bit
	TIM1->ARR   = 10;
	// Re-initialize the counter and generates update of the registers
	TIM1->EGR  |= TIM_EGR_UG;
	// Enable the timer.
	TIM1->CR1  |= TIM_CR1_CEN;

	// Enable the hardware interrupt.
	TIM1->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0x03);
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}

__attribute__((weak)) void tim1_ovf_callback(void){
	// NOTE:
	// This function should not be modified, when the callback is needed,
	// the tim1_ovf_callback could be implemented in the user file
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void){
	if(TIM1->SR & TIM_SR_UIF){
		TIM1->SR &= ~TIM_SR_UIF;

		tim1_ovf_callback();
	}
}

//********************* TIM14 10 sec trip timeout interrupt **********************
void tim14_init(void){
	// Enable the TIM14 clock.
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

	TIM14->CR1 &= ~(TIM_CR1_CEN);
	// Peripherial reset
	RCC->APB1RSTR |=  (RCC_APB1RSTR_TIM14RST);
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM14RST);
	// Prescaler - 16bit
	TIM14->PSC   = 7999;
	// Autoreload - 16bit
	TIM14->ARR   = 10000;
	// Re-initialize the counter and generates update of the registers
	TIM14->EGR  |= TIM_EGR_UG;
	// Enable the timer.
	TIM14->CR1  |= TIM_CR1_CEN;

	// Enable the hardware interrupt.
	TIM14->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM14_IRQn, 0x03);
    NVIC_EnableIRQ(TIM14_IRQn);
}

__attribute__((weak)) void tim14_ovf_callback(void){
	// NOTE:
	// This function should not be modified, when the callback is needed,
	// the tim14_ovf_callback could be implemented in the user file
}

void TIM14_IRQHandler(void){
	if(TIM14->SR & TIM_SR_UIF){
		TIM14->SR &= ~TIM_SR_UIF;

		tim14_ovf_callback();
	}
}

//********************* TIM17 1 sec interrupt **********************
void tim17_init(void){
	// Enable the TIM17 clock.
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;

	TIM17->CR1 &= ~(TIM_CR1_CEN);
	// Peripherial reset
	RCC->APB2RSTR |=  (RCC_APB2RSTR_TIM17RST);
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM17RST);
	// Prescaler - 16bit
	TIM17->PSC   = 7999;
	// Autoreload - 16bit
	TIM17->ARR   = 10000;
	// Re-initialize the counter and generates update of the registers
	TIM17->EGR  |= TIM_EGR_UG;
	// Enable the timer.
	TIM17->CR1  |= TIM_CR1_CEN;

	// Enable the hardware interrupt.
	TIM17->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM17_IRQn, 0x03);
    NVIC_EnableIRQ(TIM17_IRQn);
}

__attribute__((weak)) void tim17_ovf_callback(void){
	// NOTE:
	// This function should not be modified, when the callback is needed,
	// the tim17_ovf_callback could be implemented in the user file
}

void TIM17_IRQHandler(void){
	if(TIM17->SR & TIM_SR_UIF){
		TIM17->SR &= ~TIM_SR_UIF;

		tim17_ovf_callback();
	}
}

//************************** TIM16 PWM_FRONT **************************
void tim16_init(void){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Alternate function
	GPIOA->MODER |= (0x02 << (GPIO_PWM_FRONT * 2));
	// Alterante function as tim output
	GPIOA->AFR[0] |= (0x05 << GPIO_AFRL_AFSEL6_Pos);

	// Enable the TIM16 clock.
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	TIM16->CR1 &= ~(TIM_CR1_CEN);
	// Peripherial reset
	RCC->APB2RSTR |=  (RCC_APB2RSTR_TIM16RST);
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM16RST);

	// Prescaler - 16bit
	TIM16->PSC = 312;
	// Autoreload - 16bit
	TIM16->ARR = 255;
	// Compare - 16bit
	TIM16->CCR1 = 0;
	// Select PWM mode 1 on OC1 (OC1M = 110),
	TIM16->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	// Select active high polarity on OC1 (CC1P = 0, reset value),
	// enable the output on OC1 (CC1E = 1)
	TIM16->CCER |= TIM_CCER_CC1E;
	// Enable output (MOE = 1)
	TIM16->BDTR |= TIM_BDTR_MOE;
	// Enable counter (CEN = 1)
	// select edge aligned mode (CMS = 00, reset value)
	// select direction as upcounter (DIR = 0, reset value)
	TIM16->CR1 |= TIM_CR1_CEN;
	// Force update generation (UG = 1)
	TIM16->EGR |= TIM_EGR_UG;
}

void pwm_front_set(uint8_t pwm){
	TIM16->CCR1 = pwm;
}

//************************** TIM3 PWM_REAR **************************
void tim3_init(void){
	// Enable the TIM3 clock.
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Alternate function
	GPIOB->MODER |= (0x02 << (GPIO_PWM_REAR * 2));
	// Alterante function as tim output
	GPIOB->AFR[0] |= (0x01 << GPIO_AFRL_AFSEL5_Pos);

	TIM3->CR1 &= ~(TIM_CR1_CEN);
	// Peripherial reset
	RCC->APB1RSTR |=  (RCC_APB1RSTR_TIM3RST);
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM3RST);

	// Prescaler - 16bit
	TIM3->PSC = 312;
	// Autoreload - 16bit
	TIM3->ARR = 255;
	// Compare - 16bit
	TIM3->CCR2 = 0;
	// Select PWM mode 1 on OC2 (OC2M = 110),
	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	// Select active high polarity on OC1 (CC1P = 0, reset value),
	// enable the output on OC2 (CC2E = 1)
	TIM3->CCER |= TIM_CCER_CC2E;
	// Enable output (MOE = 1)
	TIM3->BDTR |= TIM_BDTR_MOE;
	// Enable counter (CEN = 1)
	// select edge aligned mode (CMS = 00, reset value)
	// select direction as upcounter (DIR = 0, reset value)
	TIM3->CR1 |= TIM_CR1_CEN;
	// Force update generation (UG = 1)
	TIM3->EGR |= TIM_EGR_UG;
}

void pwm_rear_set(uint8_t pwm){
	TIM3->CCR2 = pwm;
}

//********************* PA9 falling edge interrupt ********************
void exti_init(void){
	// (1) Enable the peripheral clock of GPIOA
	// (2) Select Port A for pin 0 external interrupt by writing 0000 in
	// EXTI0 (reset value)
	// (3) Configure the corresponding mask bit in the EXTI_IMR register
	// (4) Configure the Trigger Selection bits of the Interrupt line on
	// rising edge
	// (5) Configure the Trigger Selection bits of the Interrupt line on
	// falling edge
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // (1)

	GPIOA->MODER |= (0x00 << (PA9 * 2));
	GPIOA->PUPDR |= (0x00 << (PA9 * 2));

	SYSCFG->EXTICR[1] &= (uint16_t)~SYSCFG_EXTICR1_EXTI1_PA; /* (2) */
	EXTI->IMR |= (1<<PA9); // (3)
	EXTI->RTSR |= (0<<PA9); // (4)
	EXTI->FTSR |= (1<<PA9); // (5)
	// Configure NVIC for External Interrupt
	// (1) Enable Interrupt on EXTI0_1
	// (2) Set priority for EXTI0_1
	NVIC_EnableIRQ(EXTI4_15_IRQn); // (1)
	NVIC_SetPriority(EXTI4_15_IRQn,0); // (2)
}

__attribute__((weak)) void exti_PA9_callback(void){
	// NOTE:
	// This function should not be modified, when the callback is needed,
	// the exti_PA9_callback could be implemented in the user file
}

void EXTI4_15_IRQHandler(void){
	if(EXTI->PR & (1<<PA9)){
		EXTI->PR |= (1<<PA9);

		exti_PA9_callback();
	}
}
