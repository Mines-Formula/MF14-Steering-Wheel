/**
  ******************************************************************************
  * @file           : mx_fdcan1.c
  * @brief          : FDCAN1 initialization for the custom steering-wheel board.
  ******************************************************************************
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the
  * mx_stm32c5xx_hal_drivers_license.md file in the same directory as the
  * generated code.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "mx_fdcan1.h"

/*
 ******************************************************************************
 * FDCAN1 BOARD CONFIGURATION
 ******************************************************************************
 *
 * MCU-to-transceiver connections:
 *
 *   PA12, package pin 22:
 *     FDCAN1_TX -> SN65HVD230 D input, pin 1
 *
 *   PB8 / BOOT0, package pin 31:
 *     FDCAN1_RX <- SN65HVD230 R output, pin 4
 *
 * GPIO configuration retained from the generated project:
 *   Alternate function 9
 *   Push-pull
 *   No internal pull-up/down
 *   Low GPIO speed
 *
 * CAN configuration:
 *   Classic CAN
 *   Normal operating mode
 *   Automatic retransmission enabled
 *   Transmit FIFO mode
 *
 * Clock and nominal timing:
 *   Kernel clock source: PCLK1
 *   FDCAN clock divider: 1
 *   Nominal prescaler:   1
 *   Nominal segment 1:   251 time quanta
 *   Nominal segment 2:   36 time quanta
 *   Synchronization:    1 time quantum
 *
 * If the kernel clock is 144 MHz:
 *
 *   Time quanta per bit = 1 + 251 + 36 = 288
 *
 *   Bitrate = 144,000,000 / (1 * 1 * 288)
 *           = 500,000 bits/second
 *
 *   Sample point = (1 + 251) / 288
 *                = 87.5 percent
 *
 * IMPORTANT:
 *   This file selects PCLK1 but does not set its frequency.
 *   The actual clock configuration must supply the expected 144 MHz.
 *
 * Receive filtering:
 *   No dedicated standard or extended filters allocated.
 *   All non-remote standard and extended data frames go to RX FIFO 0.
 *   Remote frames are rejected.
 *
 *   main.c further selects the standard Classic CAN frames it understands.
 *
 * Startup:
 *   mx_system_init() calls mx_fdcan1_init().
 *   main.c subsequently calls HAL_FDCAN_Start().
 *   Do not start FDCAN a second time in this file.
 *
 * Hardware note:
 *   PB8 also serves as BOOT0. Verify the boot option configuration separately.
 *   This file does not change option bytes.
 *
 ******************************************************************************
 */

static hal_fdcan_handle_t hFDCAN1;

/**
  * @brief Initialize FDCAN1 and its GPIO pins.
  * @retval Pointer to FDCAN1 handle, or NULL on failure.
  */
hal_fdcan_handle_t *mx_fdcan1_init(void)
{
  hal_fdcan_config_t fdcan_config = {0};
  hal_fdcan_nominal_bit_timing_t nominal_timing = {0};
  hal_fdcan_global_filter_config_t global_filter = {0};
  hal_gpio_config_t gpio_config = {0};

  /* Enable the peripheral clock. */
  HAL_RCC_FDCAN_EnableClock();

  if (HAL_FDCAN_Init(&hFDCAN1, HAL_FDCAN1) != HAL_OK)
  {
    return NULL;
  }

  /* Actual PCLK1 frequency is established by the system clock setup. */
  if (HAL_RCC_FDCAN_SetKernelClkSource(
        HAL_RCC_FDCAN_CLK_SRC_PCLK1) != HAL_OK)
  {
    return NULL;
  }

  /*
   * 500 kbit/s with a 144 MHz kernel clock and divider 1.
   * Values retained from your generated configuration.
   */
  nominal_timing.nominal_prescaler = 1U;
  nominal_timing.nominal_time_seg1 = 251U;
  nominal_timing.nominal_time_seg2 = 36U;
  nominal_timing.nominal_jump_width = 1U;

  fdcan_config.nominal_bit_timing = nominal_timing;
  fdcan_config.frame_format = HAL_FDCAN_FRAME_FORMAT_CLASSIC_CAN;
  fdcan_config.mode = HAL_FDCAN_MODE_NORMAL;
  fdcan_config.auto_retransmission =
      HAL_FDCAN_AUTO_RETRANSMISSION_ENABLE;
  fdcan_config.transmit_pause = HAL_FDCAN_TRANSMIT_PAUSE_DISABLE;
  fdcan_config.protocol_exception =
      HAL_FDCAN_PROTOCOL_EXCEPTION_ENABLE;

  fdcan_config.std_filters_nbr = 0U;
  fdcan_config.ext_filters_nbr = 0U;
  fdcan_config.tx_fifo_queue_mode = HAL_FDCAN_TX_MODE_FIFO;

  if (HAL_FDCAN_SetConfig(&hFDCAN1, &fdcan_config) != HAL_OK)
  {
    return NULL;
  }

  if (HAL_FDCAN_SetClockDivider(&hFDCAN1,
                                HAL_FDCAN_CLOCK_DIV_1) != HAL_OK)
  {
    return NULL;
  }

  /*
   * With no dedicated filters, accept non-remote data frames into FIFO 0.
   * Application-level filtering remains in receive_can() in main.c.
   */
  global_filter.acceptance_non_matching_std =
      HAL_FDCAN_NO_MATCH_TO_RX_FIFO_0;
  global_filter.acceptance_non_matching_ext =
      HAL_FDCAN_NO_MATCH_TO_RX_FIFO_0;
  global_filter.acceptance_remote_std =
      HAL_FDCAN_REMOTE_REJECT;
  global_filter.acceptance_remote_ext =
      HAL_FDCAN_REMOTE_REJECT;

  if (HAL_FDCAN_SetGlobalFilter(&hFDCAN1,
                                &global_filter) != HAL_OK)
  {
    return NULL;
  }

  /* Enable clocks for the TX and RX GPIO ports. */
  HAL_RCC_GPIOA_EnableClock();
  HAL_RCC_GPIOB_EnableClock();

  /*
   * PA12 -> FDCAN1_TX
   * PB8  -> FDCAN1_RX
   *
   * Both use AF9 in your generated configuration.
   */
  gpio_config.mode = HAL_GPIO_MODE_ALTERNATE;
  gpio_config.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
  gpio_config.pull = HAL_GPIO_PULL_NO;
  gpio_config.speed = HAL_GPIO_SPEED_FREQ_LOW;
  gpio_config.alternate = HAL_GPIO_AF_9;

  if (HAL_GPIO_Init(HAL_GPIOA,
                    HAL_GPIO_PIN_12,
                    &gpio_config) != HAL_OK)
  {
    return NULL;
  }

  if (HAL_GPIO_Init(HAL_GPIOB,
                    HAL_GPIO_PIN_8,
                    &gpio_config) != HAL_OK)
  {
    return NULL;
  }

  return &hFDCAN1;
}

/**
  * @brief Deinitialize FDCAN1 and release its GPIO pins.
  */
void mx_fdcan1_deinit(void)
{
  (void)HAL_FDCAN_DeInit(&hFDCAN1);

  HAL_RCC_FDCAN_Reset();
  HAL_RCC_FDCAN_DisableClock();

  HAL_GPIO_DeInit(HAL_GPIOA, HAL_GPIO_PIN_12);
  HAL_GPIO_DeInit(HAL_GPIOB, HAL_GPIO_PIN_8);
}

/**
  * @brief Get the FDCAN1 handle.
  */
hal_fdcan_handle_t *mx_fdcan1_gethandle(void)
{
  return &hFDCAN1;
}