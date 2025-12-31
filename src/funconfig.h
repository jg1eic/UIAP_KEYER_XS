//
//	CH32V003 Fun Config Header
//
#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

//#define FUNCONF_USE_HSE 1  			// external crystal on PA1 PA2
#define FUNCONF_USE_HSI 1    		// internal 24MHz clock oscillator
#define FUNCONF_USE_PLL 1			// use PLL x2
#define FUNCONF_SYSTEM_CORE_CLOCK 48000000  // Computed Clock in Hz (Default only for 003, other chips have other defaults)


#define FUNCONF_USE_DEBUGPRINTF 0
#define FUNCONF_USE_UARTPRINTF  1
#define FUNCONF_UART_PRINTF_BAUD 115200

#define CH32V003           1

#endif

