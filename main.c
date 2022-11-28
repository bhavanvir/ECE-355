/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "diag/trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_hal_spi.h"


// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace-impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

SPI_HandleTypeDef SPI_Handle;

int edge_flag = 0;
int resistance = 0;
int frequency = 0;

static void ADC_Config(void);
static void DAC_Config(void);
static void myTIM2_Init();
static void myEXTI_Init();
static void display_text(char s[]);
static void display_number(uint16_t num);
static void position_lcd(unsigned short x, unsigned short y);

void refresh_LCD();
void SystemClock48MHz(void);
void HC595_Config();
void H595_Write(uint8_t data);
void write_cmd(char cmd);
void write_data(char data);
void LCD_init();

int
main(int argc, char* argv[])
{
	SystemClock48MHz();
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	ADC_Config();
	DAC_Config();
	myTIM2_Init();
	myEXTI_Init();

	// Configure LCD
	HC595_Config();
	LCD_init();

	while (1)
	{
	  ADC1->CR |= ((uint32_t)0x00000004);

	  while(!(ADC1->ISR & ((uint32_t)0x00000004)));

	  int ADC1ConvertedVal = (uint16_t) ADC1->DR;

	  resistance = (ADC1ConvertedVal * 5000)/0xFFF;

	  DAC-> DHR12R1 = ADC1ConvertedVal;

	  refresh_LCD();
	}
}

void HC595_Config() {
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable clock for GPIOB and SPI1
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure PB3 and PB5 alternate function for SPI
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure PB4 in output mode to be used as storage clock input in 74HC595
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Point to SPI1 of the stm32f0
    SPI_Handle.Instance = SPI1;

    // Initialize the SPI_Direction member
	SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;
	SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_Handle.Init.NSS = SPI_NSS_SOFT;
	SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	SPI_Handle.Init.FirstBit=SPI_FIRSTBIT_MSB;
	SPI_Handle.Init.CRCPolynomial = 7;

    // Initialize the SPI interface
    HAL_SPI_Init( &SPI_Handle );

    // Enable the SPI
    __HAL_SPI_ENABLE( &SPI_Handle );
}

void refresh_LCD() {
	//trace_printf("Resistance: %d\n", r);

	position_lcd(1, 1);
	display_text("R:");
	display_number(resistance);
	position_lcd(7, 1);
	display_text("Oh");

	position_lcd(1, 2);
	display_text("F:");
	display_number(frequency);
	position_lcd(7, 2);
	display_text("Hz");
}

static void display_text(char s[]) {
	for (unsigned short i = 0; i < strlen(s); i++)
		write_data(s[i]);
}

static void display_number(uint16_t num) {
	char n[4] = {'0', '0', '0', '0'};
	itoa(num, n, 10);
	display_text(n);
}

static void position_lcd(unsigned short x, unsigned short y) {
	unsigned short val = 127 + x;

	// If we want to write to the second row
	if (y == 2) {
		// Shift the current value by 64 bits or 4 bytes
		val += 64;
		// Output to the LCD
		write_cmd(val);
	}
}

void LCD_init(void) {
	H595_Write(0x02);
	H595_Write(0x82);
	H595_Write(0x02);

	write_cmd(0x28);
	write_cmd(0x0C);
	write_cmd(0x06);
	write_cmd(0x01);
}

void write_cmd(char cmd) {
	char low = cmd & 0x0F;
	char high = (cmd >> 4) & 0x0F;

	H595_Write(high);
	H595_Write(high | 0x80);
	H595_Write(high);
	H595_Write(low);
	H595_Write(low | 0x80);
	H595_Write(low);
}

void write_data(char data) {
	char low = data & 0x0F;
	char high = (data >> 4) & 0x0F;

	H595_Write(high | 0x40);
	H595_Write(high | 0xC0);
	H595_Write(high | 0x40);
	H595_Write(low | 0x40);
	H595_Write(low | 0xC0);
	H595_Write(low | 0x40);
}

void H595_Write(uint8_t data) {
	GPIOB->BRR = GPIO_PIN_4;

	while(! __HAL_SPI_GET_FLAG(&SPI_Handle, SPI_FLAG_TXE));

	HAL_SPI_Transmit(&SPI_Handle, &data, 1, HAL_MAX_DELAY);

	while(! __HAL_SPI_GET_FLAG(&SPI_Handle, SPI_FLAG_TXE));

	GPIOB->BSRR = GPIO_PIN_4;
}

static void ADC_Config() {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
`
	// Set MODER0 as analog and MODER1 as input for the ADC
	GPIOC->MODER &= 0xFFFFFFF3;
	GPIOC->MODER |= 0x0000000C;

	GPIOC->PUPDR &= 0xFFFFFFF3;
	GPIOC->PUPDR |= 0x00000000;

	ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD;

	ADC1->CHSELR |= ADC_CHSELR_CHSEL11;

	ADC1->SMPR &= ~((uint32_t)0x00000007);
	ADC1->SMPR |= (uint32_t)0x00000007;

	ADC1->CR |= (uint32_t)ADC_CR_ADEN;

	while(!(ADC1->ISR & ADC_CR_ADEN));
}

static void DAC_Config() {
	RCC->AHBENR |= ((uint32_t)0x00020000);

	RCC->APB1ENR |= ((uint32_t)0x20000000);

	GPIOA->MODER &= 0xFFFFFCFF;
	GPIOA->MODER |= 0x00000300;

	GPIOA->PUPDR &= 0xFFFFFCFF;
	GPIOA->PUPDR |= 0x00000000;

	DAC->CR &= 0xFFFFFFFF9;
	DAC->CR |= 0x00000000;

	DAC->CR |= 0x00000001;
}

void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	* enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = ((uint16_t)0x0000);
	/* Set auto-reloaded delay */
	TIM2->ARR = ((uint32_t)12000000);

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;

	/* Start counting timer pulses*/
	TIM2->CR1 |= TIM_CR1_CEN;
}

void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= 0x0000FF0F; //0x00001000

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0){
		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

void SystemClock48MHz( void )
{
//
// Disable the PLL
//
    RCC->CR &= ~(RCC_CR_PLLON);
//
// Wait for the PLL to unlock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != 0 );
//
// Configure the PLL for a 48MHz system clock
//
    RCC->CFGR = 0x00280000;

//
// Enable the PLL
//
    RCC->CR |= RCC_CR_PLLON;

//
// Wait for the PLL to lock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );

//
// Switch the processor to the PLL clock source
//
    RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

//
// Update the system with the new clock frequency
//
    SystemCoreClockUpdate();

}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
