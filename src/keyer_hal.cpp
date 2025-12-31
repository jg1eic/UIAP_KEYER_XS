#include "ch32fun.h"
#include "ch32v003_GPIO_branchless.h"
#include "keyer_hal.h"
#include <stdio.h>
#include <stdint.h>

uint32_t count;

void tim1_int_init()
{
	// Enable TIM1 Clock
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
	RCC->APB2PCENR |= RCC_APB2Periph_TIM1;

	// Reset TIM1 to init all regs
  	//RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
  	//RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;

	// 高機能タイマー TIM1
	TIM1->CTLR1 = 0;
	TIM1->ATRLR = 256 - 1;				// 256 -1  256us
	TIM1->PSC = 48 - 1;  // 1000kHz		// 48 -1   1us
	TIM1->RPTCR = 0;					// 繰り返しなし

	TIM1->SWEVGR = TIM_PSCReloadMode_Immediate;

	// 割り込みフラグクリア
	TIM1->INTFR = (uint16_t)~TIM_IT_Update;

	// 割り込み設定
	// PreemptionPriority = 0, SubPriority = 1
	NVIC->IPRIOR[TIM1_UP_IRQn] = 0 << 7 | 1 << 6;
	// 有効化
	NVIC->IENR[((uint32_t)(TIM1_UP_IRQn) >> 5)] |= (1 << ((uint32_t)(TIM1_UP_IRQn) & 0x1F));

	// 有効化
	TIM1->DMAINTENR |= TIM_IT_Update;

	// Enable
	TIM1->CTLR1 |= TIM_CEN;
}

/*
 * initialize TIM2 for PWM
 */
 #define TIM2_DEFAULT 0xff
void tim2_pwm_init( void )
{
	// Enable GPIOC and TIM2
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
    RCC->APB2PCENR |= RCC_APB2Periph_AFIO;

    AFIO->PCFR1 &= ~AFIO_PCFR1_TIM2_REMAP_FULLREMAP;
	AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_FULLREMAP;
	// PC7 is T2CH2, 10MHz Output alt func, push-pull
	GPIOC->CFGLR &= ~(0xf<<(4*7));
	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*7);

	// Reset TIM2 to init all regs
	RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;
	
	// SMCFGR: default clk input is CK_INT
	// set TIM2 clock prescaler divider 
	TIM2->PSC = 312;
	// set PWM total cycle width
	TIM2->ATRLR = 255;
	
	// for channel 1 and 2, let CCxS stay 00 (output), set OCxM to 110 (PWM I)
	// enabling preload causes the new pulse width in compare capture register only to come into effect when UG bit in SWEVGR is set (= initiate update) (auto-clears)
	TIM2->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE | TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2PE;

	// CTLR1: default is up, events generated, edge align
	// enable auto-reload of preload
	TIM2->CTLR1 |= TIM_ARPE;

	// Enable Channel outputs, set default state (based on TIM2_DEFAULT)
	TIM2->CCER |= TIM_CC1E | (TIM_CC1P & TIM2_DEFAULT);
	TIM2->CCER |= TIM_CC2E | (TIM_CC2P & TIM2_DEFAULT);

	// initialize counter
	TIM2->SWEVGR |= TIM_UG;
    // set default duty cycle 50% for channel 1 (600Hz tone)
    TIM2->CH2CVR = 128;
	// Enable TIM2
	//TIM2->CTLR1 |= TIM_CEN;
}

// ===================== トーン制御 =====================
void start_pwm() 
{
	AFIO->PCFR1 |= AFIO_PCFR1_TIM2_REMAP_FULLREMAP;
	TIM2->CTLR1 |= TIM_CEN;
}

void stop_pwm() 
{
    AFIO->PCFR1 &= ~AFIO_PCFR1_TIM2_REMAP_FULLREMAP;
	TIM2->CTLR1 &= ~TIM_CEN;
}

//==================================================================
//	setup
//==================================================================
int GPIO_setup()
{
    // Enable GPIO Ports A, C, D
    GPIO_port_enable(GPIO_port_A);
    GPIO_port_enable(GPIO_port_C);
    GPIO_port_enable(GPIO_port_D);
    // Set Pin Modes
    GPIO_pinMode(PIN_DOT, GPIO_pinMode_I_pullUp, GPIO_Speed_10MHz);
    GPIO_pinMode(PIN_DASH, GPIO_pinMode_I_pullUp, GPIO_Speed_10MHz);
    GPIO_pinMode(PIN_SW1, GPIO_pinMode_I_pullUp, GPIO_Speed_10MHz);
    GPIO_pinMode(PIN_SW2, GPIO_pinMode_I_pullUp, GPIO_Speed_10MHz);
    GPIO_pinMode(PIN_SPEED, GPIO_pinMode_I_analog, GPIO_Speed_10MHz);
	GPIO_pinMode(PIN_KEYOUT, GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);
	GPIO_pinMode(PIN_TONE, GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);
    return 0;
}

