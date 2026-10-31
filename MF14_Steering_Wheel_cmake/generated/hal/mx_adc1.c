/**
  ******************************************************************************
  * @file           : mx_adc1.c
  * @brief          : ADC1 Peripheral initialization
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the mx_stm32c5xx_hal_drivers_license.md file
  * in the same directory as the generated code.
  * If no mx_stm32c5xx_hal_drivers_license.md file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mx_adc1.h"

static hal_adc_handle_t hADC1;

hal_adc_handle_t *mx_adc1_init(void)
{
  hal_gpio_config_t gpio_config = {0};
  hal_adc_config_t adc_config = {0};
  hal_adc_reg_config_t reg_config = {0};
  hal_adc_channel_config_t channel_config = {0};

  HAL_RCC_GPIOA_EnableClock();

  /* Rotary inputs: PA1, PA2, PA3, PA4. */
  gpio_config.mode = HAL_GPIO_MODE_ANALOG;
  gpio_config.pull = HAL_GPIO_PULL_NO;

  if (HAL_GPIO_Init(HAL_GPIOA,
                    HAL_GPIO_PIN_1 |
                    HAL_GPIO_PIN_2 |
                    HAL_GPIO_PIN_3 |
                    HAL_GPIO_PIN_4,
                    &gpio_config) != HAL_OK)
  {
    return NULL;
  }

  if (HAL_ADC_Init(&hADC1, HAL_ADC1) != HAL_OK)
  {
    return NULL;
  }

  /* Retain the clock source from your generated initialization. */
  HAL_RCC_ADC12_EnableClock();

  if (HAL_RCC_ADCDAC_SetKernelClkSource(
        HAL_RCC_ADCDAC_CLK_SRC_PSIS) != HAL_OK)
  {
    return NULL;
  }

  adc_config.resolution = HAL_ADC_RESOLUTION_12_BIT;
  adc_config.sampling_mode = HAL_ADC_SAMPLING_MODE_NORMAL;

  if (HAL_ADC_SetConfig(&hADC1, &adc_config) != HAL_OK)
  {
    return NULL;
  }

  reg_config.trigger_src = HAL_ADC_REG_TRIG_SOFTWARE;
  reg_config.trigger_edge = HAL_ADC_REG_TRIG_EDGE_NONE;
  reg_config.sequencer_length = 1U;
  reg_config.sequencer_discont = HAL_ADC_REG_SEQ_DISCONT_DISABLE;
  reg_config.continuous = HAL_ADC_REG_CONV_SINGLE;
  reg_config.overrun = HAL_ADC_REG_OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_REG_SetConfig(&hADC1, &reg_config) != HAL_OK)
  {
    return NULL;
  }

  /*
   * Initialize sampling settings for all four channels.
   * Rank 1 is reassigned before each reading in main.c.
   */
  static const hal_adc_channel_t channels[4] =
  {
    HAL_ADC_CHANNEL_1,
    HAL_ADC_CHANNEL_2,
    HAL_ADC_CHANNEL_3,
    HAL_ADC_CHANNEL_4
  };

  channel_config.group = HAL_ADC_GROUP_REGULAR;
  channel_config.sequencer_rank = 1U;
  channel_config.sampling_time = HAL_ADC_SAMPLING_TIME_289CYCLES;
  channel_config.input_mode = HAL_ADC_IN_SINGLE_ENDED;

  for (uint32_t i = 0U; i < 4U; i++)
  {
    if (HAL_ADC_SetConfigChannel(&hADC1,
                                channels[i],
                                &channel_config) != HAL_OK)
    {
      return NULL;
    }
  }

  return &hADC1;
}

void mx_adc1_deinit(void)
{
  (void)HAL_ADC_DeInit(&hADC1);

  HAL_GPIO_DeInit(HAL_GPIOA,
                 HAL_GPIO_PIN_1 |
                 HAL_GPIO_PIN_2 |
                 HAL_GPIO_PIN_3 |
                 HAL_GPIO_PIN_4);
}

hal_adc_handle_t *mx_adc1_gethandle(void)
{
  return &hADC1;
}