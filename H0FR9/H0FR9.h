/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H0FR9.h
 Description: Header for H0FR9 module, defining hardware and software interfaces.
 Module: RGB LED control with UART communication.
 Ports: 6 UART ports (USART1-6) mapped to P1-P6.
 Timers: TIM2-4 for RGB PWM (red, green, blue).
 LED: Indicator LED on GPIOB14.
 Enums: Basic colors, RGB LED modes (pulse, sweep, dim).
 Status: Module-specific error codes.
 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H0FR9_H
#define H0FR9_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H0FR9_MemoryMap.h"
#include "H0FR9_uart.h"
#include "H0FR9_gpio.h"
#include "H0FR9_dma.h"
#include "H0FR9_inputs.h"
#include "H0FR9_eeprom.h"

/* Exported Macros *********************************************************/
#define	MODULE_PN		_H0FR9

/* Port-related Definitions */
#define	NUM_OF_PORTS	5
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define Available ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define Available USARTs */
#define _USART1
#define _USART2
#define _USART3
#define _USART4
#define _USART5
#define _USART6

/* Port-UART mapping */
#define UART_P1 &huart4
#define UART_P2 &huart2
#define UART_P3 &huart3
#define UART_P4 &huart1
#define UART_P5 &huart5
#define UART_P6 &huart6

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6

/* Timer Pin Definition */
#define RGB_RED_PORT		GPIOA
#define RGB_RED_PIN		    GPIO_PIN_15
#define RGB_RED_TIM_CH		TIM_CHANNEL_1

#define RGB_BLUE_PORT		GPIOA
#define RGB_BLUE_PIN		GPIO_PIN_6
#define RGB_BLUE_TIM_CH 	TIM_CHANNEL_1

#define RGB_GREEN_PORT		GPIOB
#define RGB_GREEN_PIN		GPIO_PIN_7
#define RGB_GREEN_TIM_CH	TIM_CHANNEL_2

/* Indicator LED */
#define _IND_LED_PORT		GPIOB
#define _IND_LED_PIN		GPIO_PIN_14

/* Module-specific Macro Definitions ***************************************/
#define PWM_TIMER_CLOCK			122880	    /* freq 120 HZ at ARR 1023 */
#define RGB_PWM_FREQ			120
#define RGB_PWM_PERIOD			((float) (1/RGB_PWM_FREQ) )
#define NUM_MODULE_PARAMS		1

/* Module-specific Enumeration Definitions *********************************/

enum RGBLedMode {
	RGB_PULSE_RGB = 1,    /* Pulsing RGB colors */
	RGB_PULSE_COLOR,      /* Pulsing a single color */
	RGB_SWEEP_BASIC,      /* Sweeping through basic colors */
	RGB_SWEEP_FINE,       /* Smooth color sweeping */
	RGB_DIM_UP,           /* Gradually increasing brightness */
	RGB_DIM_UP_WAIT,      /* Gradually increasing brightness with wait time */
	RGB_DIM_DOWN,         /* Gradually decreasing brightness */
	RGB_DIM_DOWN_WAIT,    /* Gradually decreasing brightness with wait time */
	RGB_DIM_UP_DOWN,      /* Brightness up then down */
	RGB_DIM_DOWN_UP,      /* Brightness down then up */
	RGB_DIM_UP_DOWN_WAIT, /* Brightness up-down with wait */
	RGB_DIM_DOWN_UP_WAIT  /* Brightness down-up with wait */
};

/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H0FR9_OK = 0,
	H0FR9_ERR_UNKNOWNMESSAGE,
	H0FR9_ERR_WRONGCOLOR,
	H0FR9_ERR_WRONGINTENSITY,
	H0FR9_ERR_WRONGMODE,
	H0FR9_ERROR = 255
} Module_Status;

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/

#endif /* H0FR9_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
