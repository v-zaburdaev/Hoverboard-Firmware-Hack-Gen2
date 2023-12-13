#ifndef COMMSPPM
#define COMMSPPM_H

#ifdef ENABLE_PPM

#define PPM_PIN GPIO_PIN_6
#define PPM_PORT GPIOB


void init_PPM(void);
void PPM_SysTick_Callback(void);

#endif
#endif