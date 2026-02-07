#include "main.h"
#include "gpio.h"
#include "can.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* Required prototype */
void SystemClock_Config(void);

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void CAN_Filter_Config(void);
void GSM_SendSMS(char *msg);
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();   // GSM
  MX_USART2_UART_Init();   // GPS (not parsed yet)

  /* Configure CAN Filter */
  CAN_Filter_Config();

  /* Start CAN */
  HAL_CAN_Start(&hcan2);

  /* Enable CAN RX interrupt */
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  while (1)
  {
    /* Alert node works using CAN interrupt */
  }
}

/* CAN RX Interrupt Callback */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

  uint8_t seatbelt = RxData[0];
  uint8_t accident = RxData[1];
  uint8_t alcohol  = RxData[2];

  if (accident)
  {
    /* Alert Outputs */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   // Red LED
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   // Buzzer

    /* Vehicle Control */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // Relay OFF
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // Motor OFF

    GSM_SendSMS("Accident Detected! Emergency Alert Sent.");
  }
}

/* CAN Filter Configuration */
void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef filter;

  filter.FilterActivation = ENABLE;
  filter.FilterBank = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterIdHigh = 0x123 << 5;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x7FF << 5;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan2, &filter);
}

/* GSM SMS Function */
void GSM_SendSMS(char *msg)
{
  char cmd[200];

  HAL_UART_Transmit(&huart1,
      (uint8_t *)"AT+CMGF=1\r\n", 11, 1000);
  HAL_Delay(500);

  HAL_UART_Transmit(&huart1,
      (uint8_t *)"AT+CMGS=\"+91 9699260580\"\r\n", 26, 1000);
  HAL_Delay(500);

  sprintf(cmd, "%s%c", msg, 26);   // CTRL+Z
  HAL_UART_Transmit(&huart1,
      (uint8_t *)cmd, strlen(cmd), 1000);
}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    /* Stay here */
  }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}


