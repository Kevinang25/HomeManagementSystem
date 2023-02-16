
/********************************************
*			STM32F439 Main (C Startup File)  			*
*			Developed for the STM32								*
*			Author: Kevin Angryanto								*
*			Template by Dr. Glenn Matthews				*
*			Source File														*
********************************************/

#include <stdint.h>
#include <stdio.h>
#include "boardSupport.h"
#include "main.h"
#include "config.h"
#include "stm32f439xx.h"
#include "ioMapping.h"
#include "system_stm32f4xx.h"
#include "gpioControl.h"

void RCC_config(void);
void GPIO_config(void);
void UART_config(void);
void set_TIM6(void);
void sec_timeout(uint8_t timeoutSec);
void ADC_config(void);
void transmit_Char(uint8_t charValue);
void receive_Char(float humidity);
void payload_transmit(float humidity, char humidarray[], char payloadarray[]);
uint16_t potsample(void);
float ADC_humidity(uint16_t rawHumidity);

const uint16_t time_10ms = 0x79E0;
const uint8_t payload_size = 9;
const uint8_t humidarray_size = 3;
const uint16_t time_1s = 20;			

//******************************************************************************//
// Function: main()
// Input : None
// Return : None
// Description : Entry point into the application.
// *****************************************************************************//
int main(void)
{
	uint16_t i = 0;
	uint16_t j = 0;
	uint8_t fanLED_flag = 0;
	uint8_t lightLED_flag = 0;
	uint16_t humidity_val = 0;
	float final_humidity = 0.0;
	char payload_array[payload_size];
	char humid_array[humidarray_size];
	
	// Bring up the GPIO for the power regulators.
	boardSupport_init();
	
	// Configure the required subsystems.
	RCC_config();
	GPIO_config();
	UART_config();
	ADC_config();
	
	// Initialise the humidity array.
	for (i = 0; i < payload_size; i++)
	{
		payload_array[i] = 0x00;
		
		if (i < humidarray_size)
		{
			humid_array[i] = 0x00;
		}
	}

	while(1)
	{
		//******************************* Grab value from the potentiometer *******************************
		
		humidity_val = potsample();
		final_humidity = ADC_humidity(humidity_val);
		
		//****************************** Check if the humidity is too high ******************************
		
		if (final_humidity > 75)
		{
			GPIOA->ODR &= ~(GPIO_ODR_OD10);	//Turn on the fan LED.
		}
		
		//************************************* Check for UART input *************************************
		
		receive_Char(final_humidity);
		
		//****************************** Check if the fan switch is pressed ******************************
		
		if ((GPIOA->IDR & GPIO_IDR_ID8) == 0)	//If fan switch is pressed.
		{
			fanLED_flag = 1;	//Indicates that fan switch has been pressed.
			
			//Call a one sec timeout.
			for (i = 0; i < time_1s; i++)
			{
				set_TIM6();
				while((TIM6->SR & TIM_SR_UIF) == 0)
				{
					if ((GPIOA->IDR & GPIO_IDR_ID8) == 0)	//Fan switch is still pressed.
					{	
						fanLED_flag = 1;
					}
					else //Fan switch is no longer pressed.
					{
						fanLED_flag = 0;
						break;
					}
				}
				TIM6->SR &= ~(TIM_SR_UIF);	// Clear the UIF flag.	
			}
			
			if ((fanLED_flag == 1) && (final_humidity > 75)) //Switch is pressed for more than a second and humidity > 75%.
			{
				//This timer will run for 30 seconds and continuosly check for final humidity value.
				j = 0;
				while ((j < 30) && (final_humidity > 75))
				{
					while ((GPIOA->IDR & GPIO_IDR_ID8) == 0)
					{
						//LED is only turned on when switch is released.
					}
					GPIOA->ODR |= GPIO_ODR_OD10;	//Turn the fan off.
					humidity_val = potsample();
					final_humidity = ADC_humidity(humidity_val);
					
					// **************************************** RECEIVE PROCESS ****************************************
					
					receive_Char(final_humidity); 
					
					// **************************************** LIGHT CONFIGURATION ****************************************

					if ((GPIOA->IDR & GPIO_IDR_ID9) == 0)	//If light switch is pressed.
					{
						lightLED_flag = 1;	//Indicates that fan switch has been pressed
						
						//Call a one sec timeout.
						for (i = 0; i < time_1s; i++)
						{
							set_TIM6();
							while((TIM6->SR & TIM_SR_UIF) == 0)
							{
								if ((GPIOA->IDR & GPIO_IDR_ID9) == 0)	//Light switch is still pressed.
								{	
									lightLED_flag = 1;
								}
								else //Light switch is no longer pressed.
								{
									lightLED_flag = 0;
									break;
								}
							}
							TIM6->SR &= ~(TIM_SR_UIF);	// Clear the UIF flag.	
						}
						
						if ((lightLED_flag == 1) && (GPIOA->IDR & GPIO_IDR_ID3)) // Switch is pressed & light sensor is not on.
						{
							while ((GPIOA->IDR & GPIO_IDR_ID9) == 0)
							{
								//LED is only turned on when switch is released.
							}
							GPIOB->ODR ^= GPIO_ODR_OD0;	// Toggle the light LED.
						}
						else if ((lightLED_flag == 1) && ((GPIOA->IDR & GPIO_IDR_ID3) == 0))//Switch is pressed & light sensor is on.
						{
							while ((GPIOA->IDR & GPIO_IDR_ID9) == 0)
							{
								//LED is only turned on when switch is released.
							}
							GPIOB->ODR |= GPIO_ODR_OD0;	// Can only turn off light LED.
						}
						else
						{
							// do nothing, which means switch not pressed for more than a second.
						}
					}
					
					// **************************************** TRANSMIT PROCESS ****************************************
					
					payload_transmit(final_humidity, humid_array, payload_array);
					j++;
				}
				if (j == 30)	//30 seconds have elapsed from the timer.
				{
					GPIOA->ODR &= ~(GPIO_ODR_OD10);	//Turn the fan on again.
					fanLED_flag = 1;
				}
				else	//It has not reached 30 seconds and humidity_val is now less than 75%.
				{
					GPIOA->ODR |= GPIO_ODR_OD10;	//Turn the fan off.
					fanLED_flag = 0;
				}
			}
			else if ((fanLED_flag == 1) && (final_humidity < 75)) //Switch is pressed for more than a second and humidity < 75%.
			{
				while ((GPIOA->IDR & GPIO_IDR_ID8) == 0)
				{
					//LED is only turned on when switch is released.
				}
				GPIOA->ODR ^= GPIO_ODR_OD10;	//Toggle the fan.
			}
		}
		
		//****************************** Check if the light switch is pressed ******************************

		if ((GPIOA->IDR & GPIO_IDR_ID9) == 0)	//If light switch is pressed.
		{
			lightLED_flag = 1;	//Indicates that fan switch has been pressed
			
			//Call a one sec timeout.
			for (i = 0; i < time_1s; i++)
			{
				set_TIM6();
				while((TIM6->SR & TIM_SR_UIF) == 0)
				{
					if ((GPIOA->IDR & GPIO_IDR_ID9) == 0)	//Light switch is still pressed.
					{	
						lightLED_flag = 1;
					}
					else //Light switch is no longer pressed.
					{
						lightLED_flag = 0;
						break;
					}
				}
				TIM6->SR &= ~(TIM_SR_UIF);	// Clear the UIF flag.	
			}
			
			if ((lightLED_flag == 1) && (GPIOA->IDR & GPIO_IDR_ID3)) // Switch is pressed & light sensor is not on.
			{
				while ((GPIOA->IDR & GPIO_IDR_ID9) == 0)
				{
					//LED is only turned on when switch is released.
				}
				GPIOB->ODR ^= GPIO_ODR_OD0;	// Toggle the light LED.
			}
			else if ((lightLED_flag == 1) && ((GPIOA->IDR & GPIO_IDR_ID3) == 0))//Switch is pressed & light sensor is on.
			{
				while ((GPIOA->IDR & GPIO_IDR_ID9) == 0)
				{
					//LED is only turned on when switch is released.
				}
				GPIOB->ODR |= GPIO_ODR_OD0;	// Can only turn off light LED.
			}
			else
			{
				// do nothing, which means switch not pressed for more than a second.
			}
		}
	
//****************************** Transmit the payload data ******************************

		payload_transmit(final_humidity, humid_array, payload_array);
	}
}
 

//******************************************************************************//
// Function: RCC_config()
// Input : None
// Return : None
// Description : Configure the RCC module for GPIO, TIM6, UART, and ADC.
// *****************************************************************************//
void RCC_config()
{
	//Enable RCC Module.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;	
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
	
	//Reset the peripheral interface.
	RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOARST;
	RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOBRST;
	RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOFRST;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST;
	RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;	
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	
	//Wait for two clock cycles.
	__ASM("NOP");
	__ASM("NOP");
	
	//Clear the reset bit.
	RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOARST);
	RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOBRST);
	RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOFRST);
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST);	
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART3RST);
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST);
	
	//Wait for two clock cycles.
	__ASM("NOP");
	__ASM("NOP");
}

//******************************************************************************//
// Function: GPIO_config()
// Input : None
// Return : None
// Description : Configure the GPIO for the LEDs.
// *****************************************************************************//
void GPIO_config()
{
	// Bits of interest: PB0, PA10, PA9, PA8, PA3, PF10
	
	//*****Configuring I/O for PORT A*****
	//Input: PA3, PA8, PA9
	//Output: PA10
	
	//MODER
	//Clear bits of interest.
	GPIOA->MODER &= ~(0x00 << GPIO_MODER_MODER3_Pos);
	GPIOA->MODER &= ~(0x00 << GPIO_MODER_MODER8_Pos);
	GPIOA->MODER &= ~(0x00 << GPIO_MODER_MODER9_Pos);
	GPIOA->MODER &= ~(0x03 << GPIO_MODER_MODER10_Pos);
	
	//Set the bit that should be an output.
	GPIOA->MODER |= (0x01 << GPIO_MODER_MODER10_Pos);
	
	//OTYPER
	//Clear output bit to enable push-pull.
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT10);
	
	//OSPEEDR
	//Clear output bit to set to low speed (0b00).
	GPIOA->OSPEEDR &= ~(0x03 << GPIO_OSPEEDR_OSPEED10_Pos);
	
	//PUPDR
	//Clear bits of interest to disable pull-up/pull-down.
	GPIOA->PUPDR &= ~(0x03 << GPIO_PUPDR_PUPD3_Pos);
	GPIOA->PUPDR &= ~(0x03 << GPIO_PUPDR_PUPD8_Pos);
	GPIOA->PUPDR &= ~(0x03 << GPIO_PUPDR_PUPD9_Pos);
	GPIOA->PUPDR &= ~(0x03 << GPIO_PUPDR_PUPD10_Pos);
	
	//ODR
	//Set the bits to turn the LEDs of for initial condition.
	GPIOA->ODR |= GPIO_ODR_OD10;

	//*****Configuring I/O for PORT B*****
	//Output: PB0
	
	//MODER
	//Clear bit of interest.
	GPIOB->MODER &= ~(0x03 << GPIO_MODER_MODER0_Pos);
	
	//Set the bit of interest to set as output.
	GPIOB->MODER |= (0x01 << GPIO_MODER_MODER0_Pos);

	//OTYPER
	//Clear bit of interest to enable push-pull.
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT0);
	
	//OSPEEDR
	//Clear bit of interest to set to low speed (0b00).
	GPIOB->OSPEEDR &= ~(0x03 << GPIO_OSPEEDR_OSPEED0_Pos);
	
	//PUPDR
	//Clear bit of interest to disable pull-up/pull-down.
	GPIOB->PUPDR &= ~(0x03 << GPIO_PUPDR_PUPD0_Pos);
	
	//ODR
	//Set the bit of interest to turn off the LEDs for initial condition.
	GPIOB->ODR |= GPIO_ODR_OD0;
	
	//*****Configuring I/O for PORT F*****
	//Analog: PF10 (Potentiometer)
	
	//MODER
	//Clear bit of interest first.
	GPIOF->MODER &= ~(0x03 << GPIO_MODER_MODER10_Pos);
	
	//Set bit of interest to set it into analog mode.
	GPIOF->MODER |= (0x03 << GPIO_MODER_MODER10_Pos);
}

//******************************************************************************//
// Function: UART_config()
// Input : None
// Return : None
// Description : Configure the UART setup.
// *****************************************************************************//
void UART_config()
{
	// 115200, 8, N, 1
	
	//Configure GPIOB MODER register.
	GPIOB->MODER &= ~(GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE10_Msk);
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODE11_Pos) | (0x02 << GPIO_MODER_MODE10_Pos);
	
	//Setup the Alternate Function - AF7.
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL10_Msk);
	GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL11_Pos) | (0x07 << GPIO_AFRH_AFSEL10_Pos);
	
	//Turn on 16 time over sampling.
	USART3->CR1 &= ~(USART_CR1_OVER8);
	
	//Set the USART Baud Rate - Clear out the register first.
	USART3->BRR &= 0xFFFF0000;
	
	//Now set the baud rate (9600).
	USART3->BRR |= (0x16 << USART_BRR_DIV_Mantissa_Pos) | (0x0D << USART_BRR_DIV_Fraction_Pos);	
	
	//Set the number of bits per transfer (8-bit).
	USART3->CR1 &= ~(USART_CR1_M);
	
	//Set the number of stop bits.
	USART3->CR2 &= ~(USART_CR2_STOP_Msk);
	USART3->CR2 |= (0x00 << USART_CR2_STOP_Pos);
	
	//Disable the system parity.
	USART3->CR1 &= ~(USART_CR1_PCE);
	
	//Select mode of operation (async - no clock).
	USART3 ->CR2 &= ~(USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA);
	
	//Disable hardware flow control.
	USART3->CR2 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	
	//Enable the USART, transmitter, and receive sections.
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_UE | USART_CR1_RE);
}

//******************************************************************************//
// Function: set_TIM6(void)
// Input : None
// Return : None
// Description : Set the timer to timeout for 10 ms.
// *****************************************************************************//
void set_TIM6()
{
	TIM6->CR1 &= ~(TIM_CR1_CEN);					//Ensure that timer is off.
	TIM6->PSC &= ~(TIM_PSC_PSC_Msk);			//Clear the prescaler register.
	TIM6->PSC |= 26;							
	
	TIM6->ARR &= ~(TIM_ARR_ARR_Msk);			//Clear the auto-reload register.
	TIM6->ARR |= time_10ms;
	TIM6->CR1 |= TIM_CR1_OPM;							//Set the timer in 'single-shot' mode.
	TIM6->CR1 |= TIM_CR1_CEN;							//Start the timer.
}

//******************************************************************************//
// Function: sec_timeout(uint8_t timeoutSec)
// Input : timeoutSec - How many second to call a timeout.
// Return : None
// Description : Create a timeout for a defined second.
// *****************************************************************************//
void sec_timeout(uint8_t timeoutSec)
{
	uint16_t i = 0;
	for (i = 0; i < (timeoutSec * 100); i++)
	{
		set_TIM6();
		while((TIM6->SR & TIM_SR_UIF) == 0)
		{
	
		}
		TIM6->SR &= ~(TIM_SR_UIF);	// Clear the UIF flag.	
	
	}
}
//******************************************************************************//
// Function: ADC_config(void)
// Input : None
// Return : None
// Description : Configure the ADC configuration.
// *****************************************************************************//
void ADC_config()
{
	// Enable the battery sensing channel, disable the temperature sensor channel,
	// and adjust the prescaler.
	ADC123_COMMON->CCR |= ((ADC_CCR_VBATE) | (0x03 << ADC_CCR_ADCPRE_Pos));
	ADC123_COMMON->CCR &= ~(ADC_CCR_TSVREFE);
	
	// Disable scan mode and set resolution to 12 bits.
	ADC3->CR1 &= ~((ADC_CR1_SCAN) | (0x03 << ADC_CR1_RES_Pos));
	
	// Alignment set to right and set single mode conversion
	ADC3->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_ALIGN | ADC_CR2_SWSTART);
	
	// Single channel - Channel 8 (Potentiometer)
	ADC3->SQR3 &= ~(ADC_SQR3_SQ1_Msk);
	ADC3->SQR3 |= 0x08; 
	ADC3->SQR1 &= ~(ADC_SQR1_L_Msk);
	
	// Set the sample time register - 56 cycles
	ADC3->SMPR2 &= ~(ADC_SMPR2_SMP0_Msk);
	ADC3->SMPR2 |= 0X03 << (ADC_SMPR2_SMP0_Pos);
	
	// Enable the ADC
	ADC3->CR2 |= ADC_CR2_ADON;
}

//******************************************************************************//
// Function: transmit_Char(void)
// Input : None
// Return : None
// Description : Transmit character to PC by UART.
// *****************************************************************************//
void transmit_Char(uint8_t charValue)
{
	while((USART3->SR & USART_SR_TXE) == 0x00);
	
	USART3->DR = (uint32_t)charValue;
	
	while((USART3->SR & USART_SR_TC) == 0x00);
}

//******************************************************************************//
// Function: receive_Char(void)
// Input : None
// Return : None
// Description : Transmit character to PC by UART.
// *****************************************************************************//
void receive_Char(float humidity)
{
		uint16_t receivedChar = 0;
	
		if (USART3->SR & USART_SR_RXNE)	// If no input, this code will just be skipped.
		{
			receivedChar = USART3->DR;
			
			if (receivedChar == 0x21)	// If input is '!', this will run. Otherwise, this code is skipped.
			{
				transmit_Char(0x0A);
				transmit_Char(receivedChar);
				transmit_Char(0x20);
				
				while((USART3->SR & USART_SR_RXNE) == 0)
				{
					// This loop will keep on running until an input is received.
				}
				
				receivedChar = USART3->DR;

				if (receivedChar == 0x30)	// light OFF, fan OFF
				{
					transmit_Char(receivedChar);
					transmit_Char(0x0D);
					transmit_Char(0x0A);
					GPIOB->ODR |= GPIO_ODR_OD0;				//Turn off the light LED.
					if (humidity < 75)
					{
						GPIOA->ODR |= GPIO_ODR_OD10;	//Turn off the fan LED.
					}
				}
				else if (receivedChar == 0x34)	// light OFF, fan ON
				{
					transmit_Char(receivedChar);
					transmit_Char(0x0D);
					transmit_Char(0x0A);
					GPIOB->ODR |= GPIO_ODR_OD0;				//Turn off the light LED.
					if (humidity < 75)
					{
						GPIOA->ODR &= ~(GPIO_ODR_OD10);	//Turn on the fan LED.
					}
				}
				else if (receivedChar == 0x38)	// light ON, fan OFF
				{
					transmit_Char(receivedChar);
					transmit_Char(0x0D);
					transmit_Char(0x0A);
					GPIOB->ODR &= ~(GPIO_ODR_OD0);		//Turn on the light LED.
					if (humidity < 75)
					{
						GPIOA->ODR |= GPIO_ODR_OD10;	//Turn off the fan LED.
					}
				}
				else if (receivedChar == 0x43)	// light ON, fan ON.
				{
					transmit_Char(receivedChar);
					transmit_Char(0x0D);
					transmit_Char(0x0A);
					GPIOB->ODR &= ~(GPIO_ODR_OD0);		//Turn on the light LED.
					if (humidity < 75)
					{
						GPIOA->ODR &= ~(GPIO_ODR_OD10);	//Turn on the fan LED.
					}
				}
				else	// Invalid input.
				{
					transmit_Char(receivedChar);
					transmit_Char(0x20);
					transmit_Char(0x58);	// display 'X'
					transmit_Char(0x0D);
					transmit_Char(0x0A);
				}
			}
		}
}

//******************************************************************************//
// Function: payload_transmit(float humidity, char humidarray[], char payloadarray[])
// Input : float humidity, char humidarray[], and char payloadarray[]
// Return : None
// Description : Transmit the payload by UART.
// *****************************************************************************//
void payload_transmit(float humidity, char humidarray[], char payloadarray[])
{
	//*********************************** Initialise local variables ***********************************
	uint8_t i = 0;
	uint8_t LEDbit = 0;

	//****************************** Configure the data payload (LED FLAG) ******************************
			
	if ((GPIOB->ODR & (1<<0)) && (GPIOA->ODR & (1<<10)))	// light and fan OFF
	{
		LEDbit = 0x30;
	}
	else if ((GPIOB->ODR & (1<<0)) && ((GPIOA->ODR & (1<<10)) == 0))	// light OFF and fan ON
	{
		LEDbit = 0x31;
	}
	else if (((GPIOB->ODR & (1<<0)) == 0) && (GPIOA->ODR & (1<<10))) // light ON and fan OFF
	{
		LEDbit = 0x32;
	}
	else	// light and fan ON
	{
		LEDbit = 0x33;
	}

	//****************************** Convert the final humidity to ASCII ******************************

	sprintf(humidarray, "%d", (uint8_t)humidity);

	//****************************** Configure the data payload (ARRAY) ******************************

	if (humidity > 99)
	{
		payloadarray[0] = 0x21;
		payloadarray[1] = 0x20;
		payloadarray[2] = humidarray[0];
		payloadarray[3] = humidarray[1];
		payloadarray[4] = humidarray[2];
		payloadarray[5] = 0x20;
		payloadarray[6] = LEDbit;
		payloadarray[7] = 0x0D;
		payloadarray[8] = 0x0A;
	}
	else
	{
		payloadarray[0] = 0x21;
		payloadarray[1] = 0x20;
		payloadarray[2] = 0x00;
		payloadarray[3] = humidarray[0];
		payloadarray[4] = humidarray[1];
		payloadarray[5] = 0x20;
		payloadarray[6] = LEDbit;
		payloadarray[7] = 0x0D;
		payloadarray[8] = 0x0A;
	}

	//****************************** Transmit data payload to PC by UART ******************************
	sec_timeout(1);
	for (i = 0; i < payload_size; i++)
	{
		transmit_Char(payloadarray[i]);
	}
}

//******************************************************************************//
// Function: potsample(void)
// Input : None
// Return : currentpotval
// Description : Read the value (analog) from the potentiometer.
// *****************************************************************************//
uint16_t potsample(void)
{
	uint16_t currentpotval = 0;
	
	// Trigger ADC conversion.
	ADC3->CR2 |= ADC_CR2_SWSTART;
	
	// Wait until conversion is complete.
	while((ADC3->SR & ADC_SR_EOC) == 0X00); 
	
	// Get value from the ADC
	currentpotval = (ADC3->DR & 0x0000FFFF); 
	
	// Return the raw value.
	return currentpotval;
}

//******************************************************************************//
// Function: ADC_humidity(uint16_t rawHumidity)
// Input : uint16_t rawHumidity
// Return : calculated_humidity
// Description : Convert the potentiometer reading from analog to digital.
// *****************************************************************************//
float ADC_humidity(uint16_t rawHumidity)
{
	float calculated_humidity = 0.0;
	
	// The calculated humidity value is in percentage.
	calculated_humidity = ((float)rawHumidity / 4095) * 100;
	
	return calculated_humidity;
}



