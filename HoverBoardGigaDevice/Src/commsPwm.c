#include "../Inc/defines.h"
#include "../Inc/it.h"
#include "../Inc/comms.h"
#include "../Inc/bldc.h"
#include "stdio.h"
#include "string.h"
#include "gd32f1x0.h"
#include "gd32f1x0_exti.h"


#ifdef ENABLE_PWM
// Only master communicates with steerin device
#ifdef MASTER

/*
  * Illustration of the PWM functionality
  * CH1 ________|‾‾‾‾‾‾‾‾‾‾|________
  * CH2 ______________|‾‾‾‾‾‾‾‾‾‾‾|________
  *             ↑     ↑    ↑      ↑
  * TIM2       RST  SAVE RC_CH1 RC_CH1
 */
#define PWM_TIMER TIMER2
extern int32_t steer;
extern int32_t speed;
timer_parameter_struct timer2_struct;

uint16_t pwm_captured_ch1_value = 0;
uint16_t pwm_captured_ch2_value = 0;
uint16_t pwm_CNT_prev_ch1 = 0;
uint16_t pwm_CNT_prev_ch2 = 0;
uint32_t pwm_timeout_ch1 = 0;
uint32_t pwm_timeout_ch2 = 0;
uint8_t  timeoutFlgGen = 0;

void PWM_ISR_CH1_Callback(void) {
  // Dummy loop with 16 bit count wrap around
  if(gpio_input_bit_get(PWM_PORT_CH1, PWM_PIN_CH1)) {   // Rising  Edge interrupt -> save timer value OR reset timer
    //if (gpio_input_bit_get(PWM_PORT_CH2, PWM_PIN_CH2)) {
    // pwm_CNT_prev_ch1 = timer_counter_read(PWM_TIMER);
    // } else {
    //   // timer_counter_value_config(PWM_TIMER, 0); // TIM2->CNT = 0; TIM2->CNT = 0;
    //   pwm_CNT_prev_ch1 = 0;
    // }
    pwm_timeout_ch1=0;
  } else {                                    // Falling Edge interrupt -> measure pulse duration
    // uint16_t rc_signal = timer_counter_read(PWM_TIMER) - pwm_CNT_prev_ch1;
    // if (IN_RANGE(pwm_timeout_ch1, 900, 2100)){
      // timeoutCntGen = 0;
      // timeoutFlgGen = 0;
      // pwm_captured_ch1_value e= CLAMP(pwm_timeout_ch1, 1000, 2000) - 1000;
      // pwm_timeout_ch1 = 0;
      // speed=200;
      ResetTimeout();
    // }
  }
}

void PWM_ISR_CH2_Callback(void) {
  // Dummy loop with 16 bit count wrap around
  // if(gpio_input_bit_get(PWM_PORT_CH2, PWM_PIN_CH2)) {   // Rising  Edge interrupt -> save timer value OR reset timer
  //   if (gpio_input_bit_get(PWM_PORT_CH1, PWM_PIN_CH1)) {
  //     pwm_CNT_prev_ch2 = timer_counter_read(PWM_TIMER);
  //   } else {
  //     // timer_counter_value_config(PWM_TIMER, 0); // TIM2->CNT = 0;
  //     pwm_CNT_prev_ch2 = 0;
  //   }
  // } else {                                    // Falling Edge interrupt -> measure pulse duration
  //   uint16_t rc_signal = timer_counter_read(PWM_TIMER) - pwm_CNT_prev_ch2;
  //   if (IN_RANGE(rc_signal, 900, 2100)){
  //     pwm_timeout_ch2 = 0;
  //     pwm_captured_ch2_value = CLAMP(pwm_timeout_ch2, 1000, 2000) - 1000;
  //     speed = pwm_captured_ch2_value;
  //     ResetTimeout();
  //   }
  // }

  if(gpio_input_bit_get(PWM_PORT_CH2, PWM_PIN_CH2)) {   // Rising  Edge interrupt -> save timer value OR reset timer
    // if (gpio_input_bit_get(PWM_PORT_CH2, PWM_PIN_CH2)) {
    // pwm_CNT_prev_ch1 = timer_counter_read(PWM_TIMER);
    // } else {
    //   // timer_counter_value_config(PWM_TIMER, 0); // TIM2->CNT = 0; TIM2->CNT = 0;
    //   pwm_CNT_prev_ch1 = 0;
    // }
    pwm_timeout_ch2=0;
  } else {                                    // Falling Edge interrupt -> measure pulse duration
    // uint16_t rc_signal = timer_counter_read(PWM_TIMER) - pwm_CNT_prev_ch1;
    if (IN_RANGE(pwm_timeout_ch2, 900, 2100)){
      // timeoutCntGen = 0;
      // timeoutFlgGen = 0;
      pwm_captured_ch2_value = CLAMP(pwm_timeout_ch2, 1000, 2000) - 1000;
      // pwm_timeout_ch1 = 0;
      ResetTimeout();
    }
  }
}

// SysTick executes once each ms
void PWM_SysTick_Callback(void) {
  pwm_timeout_ch1++;
  pwm_timeout_ch2++;
  // Stop after 500 ms without PWM signal
  if(pwm_timeout_ch1 > 1000) {
    // pwm_captured_ch1_value = 1000;
    pwm_timeout_ch1 = 1000;
  }
  if(pwm_timeout_ch2 > 1000) {
    // pwm_captured_ch2_value = 1000;
    pwm_timeout_ch2 = 1000;
  }
}


void COMM_PWM_Init(void) {
  // PWM Timer (TIM2)
  rcu_periph_clock_enable(RCU_TIMER2);

  timer_deinit(TIMER2);
  timer_struct_para_init(&timer2_struct);
  timer2_struct.counterdirection = TIMER_COUNTER_UP;
  // timer2_struct.alignedmode 			= TIMER_COUNTER_CENTER_DOWN;
	timer2_struct.prescaler						= (uint16_t) (SystemCoreClock / 1000000 )-1;
  timer2_struct.period = 1000;
  timer2_struct.repetitioncounter = TIMER_SP_MODE_REPETITIVE;
	timer2_struct.clockdivision 		= TIMER_CKDIV_DIV1;

  // timer_auto_reload_shadow_disable(TIMER2);
  timer_update_source_config(TIMER2, TIMER_UPDATE_SRC_REGULAR);
  timer_init(TIMER2, &timer2_struct);
  nvic_irq_enable(TIMER2_IRQn, 0, 0);
	timer_interrupt_enable(TIMER2, TIMER_INT_UP);
	
	// Enable the timer and start PWM
	timer_enable(TIMER2);

    
    // gpio_mode_set(PWM_PORT_CH1, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, PWM_PIN_CH1);
    // gpio_mode_set(PWM_PORT_CH2, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, PWM_PIN_CH2);
    // // gpio_af_set(PWM_PORT_CH1, GPIO_AF_0, PWM_PIN_CH1);
    // gpio_af_set(PWM_PORT_CH2, GPIO_AF_0, PWM_PIN_CH2);

    
    // exti_deinit();
    // exti_init(PWM_PIN_CH1, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    // exti_interrupt_enable(PWM_PIN_CH1);
    // exti_init(PWM_PIN_CH2, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    // exti_interrupt_enable(PWM_PIN_CH2);
    // nvic_irq_enable(EXTI4_15_IRQn, 4,0);
    // nvic_irq_enable(EXTI2_3_IRQn, 4,0);
  

  // // Channel 1 (steering)
  // GPIO_InitTypeDef GPIO_InitStruct1 = {0};
  // // Configure GPIO pin : PA2 (Left) or PB10 (Right)
  // GPIO_InitStruct1.Pin          = PWM_PIN_CH1;
  // GPIO_InitStruct1.Mode         = GPIO_MODE_IT_RISING_FALLING;
  // GPIO_InitStruct1.Speed        = GPIO_SPEED_FREQ_HIGH;
  // GPIO_InitStruct1.Pull         = GPIO_PULLDOWN;
  // HAL_GPIO_Init(PWM_PORT_CH1, &GPIO_InitStruct1);

  // // Channel 2 (speed)
  // GPIO_InitTypeDef GPIO_InitStruct2 = {0};
  // /*Configure GPIO pin : PA3 (Left) or PB11 (Right) */
  // GPIO_InitStruct2.Pin          = PWM_PIN_CH2;
  // GPIO_InitStruct2.Mode         = GPIO_MODE_IT_RISING_FALLING;
  // GPIO_InitStruct2.Speed        = GPIO_SPEED_FREQ_HIGH;
  // GPIO_InitStruct2.Pull         = GPIO_PULLDOWN;
  // HAL_GPIO_Init(PWM_PORT_CH2, &GPIO_InitStruct2);

  // #ifdef CONTROL_PWM_LEFT
  // /* EXTI interrupt init*/
  // HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  // HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  // #endif

  // #ifdef CONTROL_PWM_RIGHT
  // /* EXTI interrupt init*/
  // HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  // #endif

  // Start timer
  // HAL_TIM_Base_Start(&TimHandle);
}

#endif	// MASTER_OR_SINGLE
#endif // REMOTE_CRSF


