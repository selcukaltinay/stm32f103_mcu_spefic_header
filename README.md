# stm32f103_mcu_spefic_header
stm32f103_mcu_spefic_header_file

The MCU Specific Header File has been posted to github as a general template.

MCU Specific Header File restricted with GPIO is below.

``` c
/*
 * stm32f103xx.h
 *
 *  Created on: Jan 17, 2022
 *      Author: Satech
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_
#include <stdint.h>
/*
 * Memory Base Addresses
 *
 */

#define FLASH_BASE_ADDR 			(0x08000000UL)						 /* Flash Base Address (length 64kb)			 */
#define SRAM_BASE_ADDR				(0x20000000UL) 						 /* SRAM Base Address (length 20kb)  			 */


/*
 * Peripheral Bus Base Addresses
 *
 */

#define PERIPH_BASE_ADDR			(0x40000000UL)						 /* Base Address for All Peripheral			     */
#define APB1_BASE_ADDR				(0x40000000UL)						 /* APB1 Bus Domain Base Address				 */
#define APB2_BASE_ADDR				(PERIPH_BASE_ADDR + 0x00010000UL) 	 /*APB2 Bus Domain Base Address                  */
#define AHB_BASE_ADDR				(PERIPH_BASE_ADDR + 0x00018000UL) 		 /*AHB Bus Domain Base Address                   */

/*
 * Peripheral Addresses
 *
 */

/*
 * APB1 Peripherals
 *
 */

#define TIM2_BASE_ADDR 				(PERIPH_BASE_ADDR + 0x0000UL)
#define TIM3_BASE_ADDR				(TIM2_BASE_ADDR   + 0x0400UL)
#define TIM4_BASE_ADDR				(TIM3_BASE_ADDR   + 0x0400UL)
#define TIM5_BASE_ADDR				(TIM4_BASE_ADDR   + 0x0400UL)
#define TIM6_BASE_ADDR				(TIM5_BASE_ADDR   + 0x0400UL)
#define TIM7_BASE_ADDR				(TIM6_BASE_ADDR   + 0x0400UL)

#define TIM12_BASE_ADDR				(TIM7_BASE_ADDR   + 0x0400UL)
#define TIM13_BASE_ADDR				(TIM12_BASE_ADDR  + 0x0400UL)
#define TIM14_BASE_ADDR				(TIM13_BASE_ADDR  + 0x0400UL)


#define SPI2_BASE_ADDR				(APB1_BASE_ADDR   + 0x3800UL)
#define SPI3_BASE_ADDR				(APB1_BASE_ADDR   + 0x3C00UL)

#define USART2_BASE_ADDR			(APB1_BASE_ADDR   + 0x4400UL)
#define USART3_BASE_ADDR			(APB1_BASE_ADDR   + 0x4800UL)
#define UART4_BASE_ADDR				(APB1_BASE_ADDR   +	0x4C00UL)
#define UART5_BASE_ADDR				(APB1_BASE_ADDR   + 0x5000UL)

#define I2C1_BASE_ADDR				(APB1_BASE_ADDR   + 0x5400UL)
#define I2C2_BASE_ADDR				(APB1_BASE_ADDR   + 0x5800UL)

/*
 * APB2 Peripherals
 *
 */

#define EXTI_BASE_ADDR 				(APB2_BASE_ADDR   + 0x0400UL)

#define GPIO_BASE_ADDR				(APB2_BASE_ADDR   + 0x0800UL)
#define GPIOA_BASE_ADDR				(GPIO_BASE_ADDR   + 0x0000UL)
#define GPIOB_BASE_ADDR				(GPIOA_BASE_ADDR  + 0x0400UL)
#define GPIOC_BASE_ADDR				(GPIOB_BASE_ADDR  + 0x0400UL)
#define GPIOD_BASE_ADDR				(GPIOC_BASE_ADDR  + 0x0400UL)
#define GPIOE_BASE_ADDR				(GPIOD_BASE_ADDR  + 0x0400UL)
#define GPIOF_BASE_ADDR				(GPIOE_BASE_ADDR  + 0x0400UL)
#define GPIOG_BASE_ADDR				(GPIOF_BASE_ADDR  + 0x0400UL)

#define ADC1_BASE_ADDR				(APB2_BASE_ADDR   + 0x2400UL)
#define ADC2_BASE_ADDR				(APB2_BASE_ADDR   + 0x2800UL)

#define TIM1_BASE_ADDR  			(APB2_BASE_ADDR   + 0x2C00UL)

#define SPI1_BASE_ADDR				(APB2_BASE_ADDR   + 0x3000UL)

#define TIM8_BASE_ADDR				(APB2_BASE_ADDR   + 0x3400UL)

#define USART1_BASE_ADDR			(APB2_BASE_ADDR   + 0x3800UL)

#define ADC3_BASE_ADDR				(APB2_BASE_ADDR   + 0x4001UL)

#define TIM9_BASE_ADDR  			(APB2_BASE_ADDR   + 0x4C00UL)
#define TIM10_BASE_ADDR  			(APB2_BASE_ADDR   + 0x5000UL)
#define TIM11_BASE_ADDR  			(APB2_BASE_ADDR   + 0x5400UL)

/*
 * AHB Peripherals
 *
 */

#define SDIO_BASE_ADDR				(AHB_BASE_ADDR   + 0x0000UL)
#define DMA1_BASE_ADDR				(AHB_BASE_ADDR   + 0x8000UL)
#define DMA2_BASE_ADDR				(AHB_BASE_ADDR   + 0x0400UL)
#define CRC_BASE_ADDR				(AHB_BASE_ADDR   + 0xB000UL)


/*
 * Peripheral Structure Definitions
 */

typedef struct{
	volatile uint32_t CRL;
	volatile uint32_t CRH;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
}GPIO_TypeDef_t;

typedef struct{
	volatile uint32_t SR; /*Status register, */
	volatile uint32_t CR[2];
	volatile uint32_t SMPR[2];
	volatile uint32_t JOFR[4];
	volatile uint32_t HTR;
	volatile uint32_t LTR;
	volatile uint32_t SQR[3];
	volatile uint32_t JSQR;
	volatile uint32_t JDR[4];
	volatile uint32_t DR;
}ADC_TypeDef_t;

typedef struct{
	volatile uint32_t ;

}DAC_TypeDef_t;
#define GPIOA						((GPIO_TypeDef_t*)(GPIOA_BASE_ADDR))
#define GPIOB						((GPIO_TypeDef_t*)(GPIOB_BASE_ADDR))
#define GPIOC						((GPIO_TypeDef_t*)(GPIOC_BASE_ADDR))
#define GPIOD						((GPIO_TypeDef_t*)(GPIOD_BASE_ADDR))
#define GPIOE						((GPIO_TypeDef_t*)(GPIOE_BASE_ADDR))
#define GPIOF						((GPIO_TypeDef_t*)(GPIOF_BASE_ADDR))
#define GPIOG						((GPIO_TypeDef_t*)(GPIOG_BASE_ADDR))




#endif /* INC_STM32F103XX_H_ */
```
