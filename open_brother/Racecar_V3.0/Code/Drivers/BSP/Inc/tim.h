
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "bsp.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM6_Init(void);

void PWM1_Setus(uint32_t us);
void PWM2_Setus(uint32_t us);
void PWM3_Setus(uint32_t us);
void PWM4_Setus(uint32_t us);

void MY_SYSCLK_Callback(void);


#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

