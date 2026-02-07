#include "main.h"
#include "gpio.h"
#include "can.h"
#include "adc.h"

/* REQUIRED prototype (CubeMX provides definition elsewhere) */
void SystemClock_Config(void);

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[3];
/* USER CODE END PV */

void CAN_Filter_Config(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();

  /* CAN Filter */
  CAN_Filter_Config();

  HAL_CAN_Start(&hcan2);

  /* CAN Header */
  TxHeader.StdId = 0x123;
  TxHeader.ExtId = 0x00;
  TxHeader.IDE   = CAN_ID_STD;
  TxHeader.RTR   = CAN_RTR_DATA;
  TxHeader.DLC   = 3;
  TxHeader.TransmitGlobalTime = DISABLE;

  while (1)
  {
    uint16_t adc_value;
    uint8_t alcohol, vibration, seatbelt;

    seatbelt  = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    vibration = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    alcohol = (adc_value > 2000) ? 1 : 0;

    TxData[0] = seatbelt;
    TxData[1] = vibration;
    TxData[2] = alcohol;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
    {
      HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
    }

    HAL_Delay(500);
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
