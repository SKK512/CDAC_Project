

#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

char gsm_rx;
char gsm_buffer[300];
uint16_t gsm_index = 0;

char gps_rx;
char gps_buffer[200];
uint16_t gps_index = 0;

char latitude[20]  = "NA";
char longitude[20] = "NA";

char sms[200];

void SystemClock_Config(void);

void GSM_Init(void);
void GSM_CheckSMS(void);
void GSM_SendSMS(char *number, char *message);
void GPS_ReadAndParse(void);


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  GSM_Init();

  HAL_UART_Transmit(&huart3,
      (uint8_t *)"System Ready\r\nSend SMS: LOCATION\r\n",
      34, HAL_MAX_DELAY);

  while (1)
  {
    GSM_CheckSMS();
    HAL_Delay(3000);
  }
}
void GSM_Init(void)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)"AT\r\n", 4, HAL_MAX_DELAY);
  HAL_Delay(1000);

  HAL_UART_Transmit(&huart2, (uint8_t *)"AT+CMGF=1\r\n", 11, HAL_MAX_DELAY);
  HAL_Delay(1000);

  HAL_UART_Transmit(&huart2,
      (uint8_t *)"AT+CNMI=1,2,0,0,0\r\n",
      21, HAL_MAX_DELAY);
  HAL_Delay(1000);
}

void GSM_CheckSMS(void)
{
  memset(gsm_buffer, 0, sizeof(gsm_buffer));
  gsm_index = 0;

  if (HAL_UART_Receive(&huart2,
      (uint8_t *)&gsm_rx, 1, 3000) != HAL_OK)
    return;

  while (gsm_rx != '\n' && gsm_index < sizeof(gsm_buffer) - 1)
  {
    gsm_buffer[gsm_index++] = gsm_rx;
    HAL_UART_Receive(&huart2, (uint8_t *)&gsm_rx, 1, HAL_MAX_DELAY);
  }

  gsm_buffer[gsm_index] = '\0';

  /* Check for LOCATION keyword */
  if (strstr(gsm_buffer, "LOCATION") != NULL)
  {
    GPS_ReadAndParse();

    sprintf(sms,
        "Latitude: %s\r\nLongitude: %s",
        latitude, longitude);
    GSM_SendSMS("+919699260580", sms);
  }
}
void GPS_ReadAndParse(void)
{
  memset(gps_buffer, 0, sizeof(gps_buffer));
  gps_index = 0;

  while (1)
  {
    HAL_UART_Receive(&huart1,
        (uint8_t *)&gps_rx, 1, HAL_MAX_DELAY);

    if (gps_rx == '\n')
      break;

    if (gps_index < sizeof(gps_buffer) - 1)
      gps_buffer[gps_index++] = gps_rx;
  }

  if (strstr(gps_buffer, "$GPGGA") != NULL)
  {
    char *token;
    int field = 0;

    token = strtok(gps_buffer, ",");

    while (token != NULL)
    {
      field++;

      if (field == 3)
        strcpy(latitude, token);

      if (field == 5)
        strcpy(longitude, token);

      token = strtok(NULL, ",");
    }
  }
}

void GSM_SendSMS(char *number, char *message)
{
  char cmd[50];

  sprintf(cmd, "AT+CMGS=\"%s\"\r\n", number);
  HAL_UART_Transmit(&huart2,
      (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
  HAL_Delay(1000);

  HAL_UART_Transmit(&huart2,
      (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2,
      (uint8_t *)"\x1A", 1, HAL_MAX_DELAY);

  HAL_Delay(3000);
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
