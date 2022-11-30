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

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

/* Clock prescaler for TIM3 timer: no prescaling */
#define myTIM3_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM3_PERIOD ((uint32_t)0xFFFFFFFF)

SPI_HandleTypeDef SPI_Handle;

int rising_edge = 0;
int r = 0;
int f = 0;

static void ADC_Config(void);
static void DAC_Config(void);
static void myTIM2_Init(void);
static void myEXTI_Init(void);
static void LCD_Display_Text(char display_string[]);
static void LCD_Display_Number(uint16_t number);
static void LCD_Set_Position(unsigned short row, unsigned short column);

void LCD_Reset_Display(void);
void SystemClock48MHz(void);
void HC595_Config(void);
void HC595_Transfer(uint8_t data);
void LCD_Write_Command(char command);
void LCD_Write_Data(char data);
void myLCD_Init(void);

int main(int argc, char *argv[])
{
    // Configure the system clock to 48 MHz
    SystemClock48MHz();
    trace_printf("System clock: %u Hz\n", SystemCoreClock);

    ADC_Config();
    DAC_Config();
    myTIM2_Init();
    myEXTI_Init();

    HC595_Config();
    myLCD_Init();

    while (1)
    {
        // Start the ADC
        ADC1->CR |= ((uint32_t)0x00000004);

        // Wait for the ADC to finish initializing
        while (!(ADC1->ISR & ((uint32_t)0x00000004))){
            // Do nothing until the ADC is ready
        }

        // Get our ADC value
        int ADC1ConvertedVal = (uint16_t)ADC1->DR;
        // Convert our ADC value to achieve a resistance value
        r = (ADC1ConvertedVal * 5000) / 0xFFF;

        // Normalize our value through our DAC
        DAC->DHR12R1 = ADC1ConvertedVal;

        // Display our value on the LCD
        LCD_Reset_Display();
    }
}

void HC595_Config()
{
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
    SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_Handle.Init.CRCPolynomial = 7;

    // Initialize the SPI interface
    HAL_SPI_Init(&SPI_Handle);

    // Enable the SPI
    __HAL_SPI_ENABLE(&SPI_Handle);
}

void LCD_Reset_Display()
{
    // Set the position of the LCD to the 2nd line and 1st column to display the resistance
    LCD_Set_Position(1, 1);
    LCD_Display_Text("R:");
    LCD_Display_Number(r);
    // Set the position of the LCD to the 7th column to display the units
    LCD_Set_Position(7, 1);
    LCD_Display_Text("Oh");

    // Set the position of the LCD to the 2nd line and 1st column to display the frequency
    LCD_Set_Position(1, 2);
    LCD_Display_Text("F:");
    LCD_Display_Number(f);
    // Set the position of the LCD to the 7th column to display the units
    LCD_Set_Position(7, 2);
    LCD_Display_Text("Hz");
}

static void LCD_Display_Text(char display_string[])
{
    // Loop through the string and display each character
    for (unsigned short i = 0; i < strlen(display_string); i++)
    {
        LCD_Write_Data(display_string[i]);
    }
}

static void LCD_Display_Number(uint16_t number)
{
    // Allocate 4 digits to be used to represent the frequency and resistance
    char allocated_digits[4] = {'0', '0', '0', '0'};
    // Convert the integer to a null terminiated string, with a radix of 10 (decimal number ranging from 0-9)
    itoa(number, allocated_digits, 10);
    LCD_Display_Text(allocated_digits);
}

static void LCD_Set_Position(unsigned short row, unsigned short column)
{
    // 8 possible bits to set, with a maximum value of 2^8 - 1 = 255, with 127 as the base value
    unsigned short shift = 127 + row;

    // If column corresponds to the second row, add 64 to the value to shift it to the next line
    if (column == 2)
    {
        // 64 = 0x40 in hexadecimal, which corresponds to the 1st column of the 2nd line, adding 64 each time will shift the spot down
        shift += 64;
        LCD_Write_Command(shift);
    }
}

// Source: Slide 22 of L17 - SPI and LCD interface 
void myLCD_Init()
{
    // Change the LCD to use a 4-bit interface
    // Enable: write only the H portion of the instruction with RS = 0
    HC595_Transfer(0x02);
    // Send the instruction to change the LCD to use a 4-bit interface 
    HC595_Transfer(0x82);
    // Enable: write only the H portion of the instruction with RS = 0
    HC595_Transfer(0x02);

    // Set DL = 0, N = 1, F = 0 for the LCD
    LCD_Write_Command(0x28);
    // Set D = 1, C= 0, B = 0 for the LCD
    LCD_Write_Command(0x0C);
    // Set I/D = 1, S = 0 for the LCD
    LCD_Write_Command(0x06);
    // Clear the LCD display
    LCD_Write_Command(0x01);
}

// Source: Slide 20 of L17 - SPI and LCD interface
void LCD_Write_Command(char command)
{
    // We need to break the command instruction into two 4-bit halves, L for the lower 4 bits and H for the higher 4 bits
    // Mask the lower 4 bits
    char L = command & 0x0F;
    // Bit shift the input 4 bits to the right, then mask the lower 4 bits
    char H = (command >> 4) & 0x0F;

    // Since we first send data in the form of 00xxH, we can just send the H bits themselves
    HC595_Transfer(H);
    // Since we send data in the form of 10xxH, we can disregard the xxH portion and set them to all 0's, leading to 100000000 = 0x80
    // We then bitwise OR the H bits with 0x80 to get the final instruction which includes H
    HC595_Transfer(H | 0x80);
    // We finally send the same instruction, 00xxH again to complete transmission
    HC595_Transfer(H);
    // L is sent in the same way as H, but with the series of instructions being 00xxL, 10xxL, 00xxL
    HC595_Transfer(L);
    HC595_Transfer(L | 0x80);
    HC595_Transfer(L);
}

// Source: Slide 21 of L17 - SPI and LCD interface
void LCD_Write_Data(char data)
{
    // We need to break the command instruction into two 4-bit halves, L for the lower 4 bits and H for the higher 4 bits
    // Mask the lower 4 bits
    char L = data & 0x0F;
    // Bit shift the input 4 bits to the right, then mask the lower 4 bits
    char H = (data >> 4) & 0x0F;

    // Since we first send data in the form of 01xxH, we can disregard the xxH portion and set them to all 0's, leading to 010000000 = 0x40
    // We then bitwise OR the H bits with 0x40 to get the final instruction which includes H
    HC595_Transfer(H | 0x40);
    // Since we send data in the form of 11xxH, we can disregard the xxH portion and set them to all 0's, leading to 110000000 = 0xC0
    // We then bitwise OR the H bits with 0xC0 to get the final instruction which includes H
    HC595_Transfer(H | 0xC0);
    // We finally send the same instruction, 01xxH again to complete transmission
    HC595_Transfer(H | 0x40);
    // L is sent in the same way as H, but with the series of instructions being 01xxL, 11xxL, 01xxL
    HC595_Transfer(L | 0x40);
    HC595_Transfer(L | 0xC0);
    HC595_Transfer(L | 0x40);
}

void HC595_Transfer(uint8_t data)
{
    // Reset bits in Port B Pin_4 to low
    GPIOB->BRR = GPIO_PIN_4;

    // Wait until the Tx buffer empty flag is set to false, meaning that there is data in the buffer waiting to be transmitted via SPI
    while (!__HAL_SPI_GET_FLAG(&SPI_Handle, SPI_FLAG_TXE))
    {
        // Do nothing as long as the Tx buffer is empty
    }

    // Transmit the data, in this case, the data is 8 bits of the LCD corresponding to a character
    HAL_SPI_Transmit(&SPI_Handle, &data, 1, HAL_MAX_DELAY);

    // Wait until the Tx buffer empty flag is set to false, meaning that there is data in the buffer waiting to be transmitted via SPI
    while (!__HAL_SPI_GET_FLAG(&SPI_Handle, SPI_FLAG_TXE))
    {
        // Do nothing as long as the Tx buffer is empty
    }

    // Set bits in Port B Pin_4 to high
    GPIOB->BSRR = GPIO_PIN_4;
}

static void ADC_Config()
{
    // Enabled GPIO Clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    // Enable clock for ADC
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    // Configure GPIO as output
    GPIOC->MODER &= 0xFFFFFFF3;
    GPIOC->MODER |= 0x0000000C;

    // Configure GPIO for input
    GPIOC->PUPDR &= 0xFFFFFFF3;
    GPIOC->PUPDR |= 0x00000000;

    // Configure continuous read
    ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD;

    // Set ADC to select chanel 11
    ADC1->CHSELR |= ADC_CHSELR_CHSEL11;
    
    // Set sampling input to 239.5 ADC clock cycles
    ADC1->SMPR &= ~((uint32_t)0x00000007);
    ADC1->SMPR |= (uint32_t)0x00000007;

    // Start the ADC
    ADC1->CR |= (uint32_t)ADC_CR_ADEN;

    // Wait for initialization of the ADC to complete
    while (!(ADC1->ISR & ADC_CR_ADEN))
    {
        // Do nothing as long as the initialization is not complete
    }
}

static void DAC_Config()
{
    // Enable TSCEN clock
    RCC->AHBENR |= ((uint32_t)0x00020000);

    // Enable Timer 14 for the DAC
    RCC->APB1ENR |= ((uint32_t)0x20000000);

    // Configure all GPIO pins as Input (Analog)
    GPIOA->MODER &= 0xFFFFFCFF;
    // Configure GPIO pin 5 as input
    GPIOA->MODER |= 0x00000300;

    // Configure all GPIO pins as reserved
    GPIOA->PUPDR &= 0xFFFFFCFF;
    // Set all GPIO pins to pull/push
    GPIOA->PUPDR |= 0x00000000;

    // Set DAC channel 1 to enabled and use all 12 channel 1 bits
    DAC->CR &= 0xFFFFFFFF9;
    //Restart channel 1
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
    TIM2->PSC = myTIM2_PRESCALER;
    /* Set auto-reloaded delay */
    TIM2->ARR = myTIM2_PERIOD;

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

    /* start counting timer pulses TIM2->CR1 |= TIM_CR1_CEN; */
    // TIM2->CR1 |= TIM_CR1_CEN;
}

void myEXTI_Init()
{
    /* Map EXTI1 line to PA1 */
    // Relevant register: SYSCFG->EXTICR[0]
    SYSCFG->EXTICR[0] &= 0x0000FF0F; // 0x00001000

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
    if ((TIM2->SR & TIM_SR_UIF) != 0)
    {
        /* Clear update interrupt flag */
        // Relevant register: TIM2->SR
        TIM2->SR &= ~(TIM_SR_UIF);

        /* Restart stopped timer */
        // Relevant register: TIM2->CR1
        TIM2->CR1 |= TIM_CR1_CEN;
    }
}

void EXTI0_1_IRQHandler()
{
    // Declare/initialize your local variables here...
    double period = 0;
    double frequency = 0;

    /* Check if EXTI1 interrupt pending flag is indeed set */
    if ((EXTI->PR & EXTI_PR_PR1) != 0)
    {
        // First edge
        if (rising_edge == 0)
        {
            rising_edge = 1;

            // Clear count register
            TIM2->CNT = 0x00000000;
            // Start timer
            TIM2->CR1 |= TIM_CR1_CEN;
        }
        // Second edge
        else
        {
            rising_edge = 0;

            EXTI->IMR &= ~(EXTI_IMR_MR1);

            // Stop timer
            TIM2->CR1 &= ~(TIM_CR1_CEN);
            // Save the current clock cycle
            uint32_t count = TIM2->CNT;

            // Calculate period and frequency
            period = (double)count / (double)SystemCoreClock;
            frequency = 1.0 / period;

            // Set the global variable f to the calculated frequency
            f = frequency;

            EXTI->IMR |= EXTI_IMR_MR1;
        }
        // Clear EXTI1 interrupt pending flag
        EXTI->PR |= EXTI_PR_PR1;
    }
}

void SystemClock48MHz(void)
{
    // Disable the PLL
    RCC->CR &= ~(RCC_CR_PLLON);
    // Wait for the PLL to unlock
    while ((RCC->CR & RCC_CR_PLLRDY) != 0);
    // Configure the PLL for a 48MHz system clock
    RCC->CFGR = 0x00280000;

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;

    // Wait for the PLL to lock
    while ((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

    // Switch the processor to the PLL clock source
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

    // Update the system with the new clock frequency
    SystemCoreClockUpdate();
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------