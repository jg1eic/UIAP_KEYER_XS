/*
  ==== ピン割り当て（要: 自分の配線に合わせて変更）====
  - PD1 は SWIO(書き込み/デバッグ) なので基本触らない方が安全
*/
#define micros() (SysTick->CNT / DELAY_US_TIME)
#define millis() (SysTick->CNT / DELAY_MS_TIME)

#define PIN_DOT    PC5     // パドル左
#define PIN_DASH   PC6     // パドル右
#define PIN_TONE   PC7     // トーン出力 (GPIOでトグル)
#define PIN_KEYOUT PC0     // 無線機用KEYOUT
#define PIN_SPEED  PA2     // 速度調整ADC（A0相当。CH32V003F4P6でPA2がAIN0）
#define PIN_SW1    PD0     // SW 1
#define PIN_SW2    PC3     // SW 2
// ---- PWM出力ピン (TIM2 output set 0: CH1=D4, CH2=D3, CH3=C0, CH4=D7) ----
//#define PWM_TONE  GPIOv_from_PORT_PIN(GPIO_port_C, 7)  // PD4 = TIM2_CH1

extern int GPIO_setup(void);
extern void start_pwm(void);
extern void stop_pwm(void);
extern void tim2_pwm_init(void);
extern void tim1_int_init(void);

extern "C" void TIM1_UP_IRQHandler(void) __attribute__((interrupt));