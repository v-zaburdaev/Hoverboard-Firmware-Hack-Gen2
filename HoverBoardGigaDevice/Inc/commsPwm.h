#ifndef COMMSPWM
#define COMMSPWM_H

#ifdef ENABLE_PWM
#define DELAY_TIM_FREQUENCY_US 1000000

#define PWM_PORT_CH1 GPIOB 
#define PWM_PIN_CH1 GPIO_PIN_6
#define PWM_PORT_CH2 GPIOB
#define PWM_PIN_CH2 GPIO_PIN_7

// extern volatile uint16_t pwm_captured_ch1_value;
// extern volatile uint16_t pwm_captured_ch2_value;

void COMM_PWM_Init(void);
void PWM_SysTick_Callback(void);
void PWM_ISR_CH1_Callback(void);
void PWM_ISR_CH2_Callback(void);
uint16_t getPwmCh1();
uint16_t getPwmCh2();


#endif
#endif