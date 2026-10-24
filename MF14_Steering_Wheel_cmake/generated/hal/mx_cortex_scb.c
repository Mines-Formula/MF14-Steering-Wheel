/**
  ******************************************************************************
  * @file           : mx_cortex_scb.c
  * @brief          : CORTEX_SCB Peripheral initialization
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
#include "mx_cortex_scb.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions prototype------------------------------------------------*/
/* Exported variables by reference--------------------------------------------*/

/******************************************************************************/
/* Exported functions for CORTEX_SCB in HAL layer */
/******************************************************************************/
system_status_t mx_cortex_scb_init(void)
{
  /* MemManagement fault is enabled by MPU */

  /* Memory management */
  HAL_CORTEX_NVIC_SetPriority(MemoryManagement_IRQn, HAL_CORTEX_NVIC_PREEMP_PRIORITY_0, HAL_CORTEX_NVIC_SUB_PRIORITY_0);

  return SYSTEM_OK;
}

void mx_cortex_scb_deinit(void)
{
  HAL_CORTEX_SCB_EnableHardFaultEscalation(HAL_CORTEX_SCB_MEM_MANAGEMENT_FAULT);
}

/******************************************************************************/
/*            Memory management is managed directly in user code.             */
/******************************************************************************/
/* void MemManage_Handler(void)
{
}
  */
