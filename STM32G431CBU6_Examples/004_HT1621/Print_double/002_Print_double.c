 /**-------------------------------------------------------------------
 *\date  03.01.2026
 *\brief
 *
 ***********************************************************************************************
 * В данном примере выведем вещественное число с плавающей запятой на экран LCD.
 ***********************************************************************************************
 *
 *   STM32G431CBU6          HT1624
 *   ------------         ------------
 *  |            |       |
 *  |            |       |
 *  |            |       |
 *  |        PA.0| ----> | CS
 *  |        PA.1| ----> | WR
 *  |        PA.2| ----> | Data
 *  |        PA.3| ----> | LED+ для работы данного вывода необходимо доработать плату экрана
 *  |            |       |
 *  |        +5V | <---> | Vcc
 *  |        GND | <---> | GND
 *
 *
 *
 *\ authors        ScuratovaAnna
 *\ сode debugging ScuratovaAnna
 */
//***************************** Пример первый ****************************

#include "main.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"

#define CS 0
#define WR 1
#define DATA 2

#define CS_clear 16
#define WR_clear 17
#define DATA_clear 18

#define BIAS 0x52
#define SYSDIS 0X00
#define SYSEN 0X02
#define LCDOFF 0X04
#define LCDON 0X06
#define XTAL 0x28
#define RC256 0X30
#define TONEON 0X12
#define TONEOFF 0X10
#define WDTDIS1 0X0A

int _buffer[7] = {
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
};
/************************************************************************/
void wrDATA(uint8_t data, uint8_t len);
void wrCMD(uint8_t CMD);
void config(void);
void clear(void);
void wrCLR(uint8_t len);
void wrclrdata(uint8_t addr, uint8_t sdata);
void update(void);
void wrone(uint8_t addr, uint8_t sdata);
void setdecimalseparator(int decimaldigits);
char charToSegBits(char character);
void print_long(long num, const char* flags, int precision);
void print_double(double num, int precision);
/************************************************************************/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/************************************************************************/
double num = 0.711;
/************************************************************************/
int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  config();
  clear();
  while (1) {
    print_double(num, 3);
    HAL_Delay(10);
    num  += 0.001;
    if(num >= 3)num = -2.711;
  }
}
/************************************************************************/
void wrDATA(uint8_t data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    GPIOA->BSRR |= (1 << WR_clear);
    GPIOA->BSRR = (data & 0x80) ? GPIOA->BSRR | (1 << DATA) : GPIOA->BSRR | (1 << DATA_clear);
    GPIOA->BSRR |= (1 << WR);
    data = (data << 1);
  }
}
/************************************************************************/
void wrCMD(uint8_t CMD) {
  GPIOA->BSRR |= (1 << CS_clear);
  wrDATA(0x80, 4);
  wrDATA(CMD, 8);
  GPIOA->BSRR |= (1 << CS);
}
/************************************************************************/
void config(void) {
  wrCMD(BIAS);
  wrCMD(RC256);
  wrCMD(SYSDIS);
  wrCMD(WDTDIS1);
  wrCMD(SYSEN);
  wrCMD(LCDON);
}
/************************************************************************/
void clear(void) {
  wrCLR(16);
}
/************************************************************************/
void wrCLR(uint8_t len) {
  uint8_t addr = 0;
  uint8_t i;
  for (i = 0; i < len; i++) {
    wrclrdata(addr, 0x00);
    addr = addr + 2;
  }
}
/************************************************************************/
void wrclrdata(uint8_t addr, uint8_t sdata) {
  addr <<= 2;
  GPIOA->BSRR |= (1 << CS_clear);
  wrDATA(0xa0, 3);
  wrDATA(addr, 6);
  wrDATA(sdata, 8);
  GPIOA->BSRR |= (1 << CS);
}
/************************************************************************/
void update(void) {
  wrone(0, _buffer[5]);
  wrone(2, _buffer[4]);
  wrone(4, _buffer[3]);
  wrone(6, _buffer[2]);
  wrone(8, _buffer[1]);
  wrone(10, _buffer[0]);
}
/************************************************************************/
void wrone(uint8_t addr, uint8_t sdata) {
  addr <<= 2;
  GPIOA->BSRR |= (1 << CS_clear);
  wrDATA(0xa0, 3);
  wrDATA(addr, 6);
  wrDATA(sdata, 8);
  GPIOA->BSRR |= (1 << CS);
}
/************************************************************************/
void setdecimalseparator(int decimaldigits) {
  _buffer[3] &= 0x7F;
  _buffer[4] &= 0x7F;
  _buffer[5] &= 0x7F;

  if (decimaldigits <= 0 || decimaldigits > 3) {
    return;
  }
  _buffer[6 - decimaldigits] |= 0x80;
}
/************************************************************************/
char charToSegBits(char character) {
  switch (character) {
    case '0':
      return 0x7D;
    case '1':
      return 0x60;
    case '2':
      return 0x3E;
    case '3':
      return 0x7A;
    case '4':
      return 0x63;
    case '5':
      return 0x5B;
    case '6':
      return 0x5F;
    case '7':
      return 0x70;
    case '8':
      return 0x7F;
    case '9':
      return 0x7B;
    case ' ':
      return 0x00;
    case '*':
      return 0x33;
    case '|':
      return 0x5;
    case '-':
      return 0x2;
    case '_':
      return 0x8;
  }
}
/************************************************************************/
void print_long(long num, const char* flags, int precision) {
  if (num > 999999)
    num = 999999;
  if (num < -99999)
    num = -99999;
  char localbuffer[7];
  snprintf(localbuffer, 7, flags, num);
  if (precision > 0 && (num) < pow(10, precision)) {
    for (int i = 0; i < (5 - precision); i++) {
      if (localbuffer[i + 1] == '0' && localbuffer[i] != '-') {
        localbuffer[i] = ' ';
      } else {
        break;
      }
    }
  }
  for (int i = 0; i < 6; i++) {
    _buffer[i] &= 0x80;
    _buffer[i] |= charToSegBits(localbuffer[i]);
  }
  update();
}
/************************************************************************/
void print_double(double num, int precision) {
  if (num > 999999)
    num = 999999;
  if (num < -99999)
    num = -99999;
  if (precision > 3 && num > 0)
    precision = 3;
  else if (precision > 2 && num < 0)
    precision = 3;//!?
  if (precision < 0)
    precision = 0;
  const char* flags = (precision > 0 && abs(num) < 1) ? "%06li" : "%6li";
  long integerpart;
  integerpart = ((long)(num * pow(10, precision)));
  print_long(integerpart, flags, precision);
  setdecimalseparator(precision);
  update();
}
/************************************************************************/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}
/************************************************************************/
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
}
#endif
/****************************** End of file *****************************/
