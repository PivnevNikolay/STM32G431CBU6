/**-------------------------------------------------------------------
 \date  07.01.2026
 *
 *   STM32G431CBU6
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |        PC.6| ---->  LED
 *  |            |
 *  |        PB.1| <----  Button
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 * В качестве кнопки использую  touch button ttp223
 *
 *
 *\ authors        ScuratovaAnna + PivnevNikolay 
 *\ сode debugging ScuratovaAnna + PivnevNikolay 
 */
#include "stm32g4xx.h"

#define GPIO_Pin_0 ((uint16_t)0x0001)
#define GPIO_Pin_1 ((uint16_t)0x0002)
#define GPIO_Pin_2 ((uint16_t)0x0004)
#define GPIO_Pin_3 ((uint16_t)0x0008)
#define GPIO_Pin_4 ((uint16_t)0x0010)
#define GPIO_Pin_5 ((uint16_t)0x0020)
#define GPIO_Pin_6 ((uint16_t)0x0040)
#define GPIO_Pin_7 ((uint16_t)0x0080)
#define GPIO_Pin_8 ((uint16_t)0x0100)
#define GPIO_Pin_9 ((uint16_t)0x0200)
#define GPIO_Pin_10 ((uint16_t)0x0400)
#define GPIO_Pin_11 ((uint16_t)0x0800)
#define GPIO_Pin_12 ((uint16_t)0x1000)
#define GPIO_Pin_13 ((uint16_t)0x2000)
#define GPIO_Pin_14 ((uint16_t)0x4000)
#define GPIO_Pin_15 ((uint16_t)0x8000)

#define Input_mode (0x0UL)
#define General_purpose_output_mode (0x1UL)
#define Alternate_function_mode (0x2UL)
#define Analog_mode (0x3UL)

#define Output_push_pull (0x0UL)
#define Output_open_drain (0x1UL)

#define Low_speed (0x0UL)
#define Medium_speed (0x1UL)
#define High_speed (0x2UL)
#define Very_high_speed (0x3UL)

#define No_pull_up_No_pull_down (0x0UL)
#define Pull_up (0x1UL)
#define Pull_down (0x2UL)

#define PA (0x0UL)
#define PB (0x1UL)
#define PC (0x2UL)
#define PD (0x3UL)
#define PE (0x4UL)
#define PF (0x5UL)
#define PG (0x6UL)

void PORTC_6_INIT_Led(void) {
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOCEN);                                                      // Запуск тактирования порта C
  MODIFY_REG(GPIOC->MODER, GPIO_MODER_MODE6, General_purpose_output_mode << GPIO_MODER_MODE6_Pos); // Настройка GPIOC пин 6 на выход (output mode)
  MODIFY_REG(GPIOC->OTYPER, GPIO_OTYPER_OT6, Output_push_pull << GPIO_OTYPER_OT6_Pos);             // Настройка GPIOC пин 6 в режим Push-Pull
  MODIFY_REG(GPIOC->OSPEEDR, GPIO_OSPEEDR_OSPEED6, High_speed << GPIO_OSPEEDR_OSPEED6_Pos);        // Настройка GPIOC пин 6 в режим High_speed
  MODIFY_REG(GPIOC->PUPDR, GPIO_PUPDR_PUPD6, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD6_Pos);     // Настройка GPIOC пин 6 в режим High_speed
}

void PORTB_1_INIT_Button(void) {
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN);
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE1, Input_mode << GPIO_MODER_MODE1_Pos);              // Настройка GPIOB пин 1 на выход (Input_mode)
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD1, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD1_Pos); // Настройка GPIOB пин 1 в режим No_pull_up_No_pull_down
}

void EXTI_Init(void) {
  MODIFY_REG(SYSCFG->EXTICR[0], SYSCFG_EXTICR1_EXTI1, PB << SYSCFG_EXTICR1_EXTI1_Pos);

  SET_BIT(EXTI->FTSR1, EXTI_FTSR1_FT1);   // Настройка детектора спадающего фронта (Falling trigger)
  CLEAR_BIT( EXTI->RTSR1, EXTI_RTSR1_RT1);// Отключаем нарастающий фронт

  SET_BIT(EXTI->IMR1,EXTI_IMR1_IM1);

  NVIC_SetPriority(EXTI1_IRQn, 2);
  NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI1_IRQHandler(void) {
  if (EXTI->PR1 & EXTI_PR1_PIF1) {

    WRITE_REG(GPIOC->ODR, READ_REG(GPIOC->ODR) ^ GPIO_ODR_OD6_Msk);

    EXTI->PR1 = EXTI_PR1_PIF1;
  }
}

int main(void) {
  // Включение тактирования SYSCFG (нужно для выбора источника EXTI)
  // LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
  // LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN);

  PORTC_6_INIT_Led();
  PORTB_1_INIT_Button();
  EXTI_Init();

  while (1) {
  }
}