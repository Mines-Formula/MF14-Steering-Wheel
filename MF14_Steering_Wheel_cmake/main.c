/**
  ******************************************************************************
  * file           : main.c
  * brief          : Main program body
  *                  Calls target system initialization then loop in main.
  ******************************************************************************
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mx_fdcan1.h"
#include <stddef.h>
#include <stdint.h>


/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint16_t engine_rpm;
  int16_t coolant_temp_c;
  int16_t oil_temp_c;
  float ecu_voltage_v;
  uint8_t gear;

  uint8_t engine_data_received;
  uint8_t temp_data_received;
  uint8_t gear_data_received;

  uint32_t rx_count;
  uint32_t rx_errors;
} wheel_vehicle_state_t;

/* Private define ------------------------------------------------------------*/
#define CAN_ID_ENGINE_DATA        0x640U
#define CAN_ID_TEMP_AND_VOLTAGE    0x649U
#define CAN_ID_STATES              0x64DU

#define CAN_ID_STEERING_BUTTONS    0x7F1U
#define CAN_ID_STEERING_DIAL       0x7F2U

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static hal_fdcan_handle_t *wheel_can;

/* Volatile so the latest values can be inspected in a debugger. */
static volatile wheel_vehicle_state_t vehicle = {0};

/* Private functions prototype -----------------------------------------------*/
static uint16_t read_u16_be(const uint8_t *data);
static void write_u16_be(uint8_t *data, uint16_t value);
static void receive_can(void);

hal_status_t steering_send_buttons(uint16_t button1, uint16_t button2,
                                   uint16_t button3, uint16_t button4);
hal_status_t steering_send_dial(uint16_t dial_raw);


/**
  * brief:  The application entry point.
  * retval: none but we specify int to comply with C99 standard
  */
int main(void)
{
  /** System Init: this code placed in targets folder initializes your system.
    * It calls the initialization (and sets the initial configuration) of the peripherals.
    * You can use STM32CubeMX to generate and call this code or not in this project.
    * It also contains the HAL initialization and the initial clock configuration.
    */
  if (mx_system_init() != SYSTEM_OK)
  {
    return (-1);
  }
  else
  {
    /*
      * You can start your application code here
      */
    wheel_can = mx_fdcan1_gethandle();

    if (wheel_can == NULL)
    {
      return (-2);
    }

    if (HAL_FDCAN_Start(wheel_can) != HAL_OK)
    {
      return (-3);
    }

    while (1)
    {
      receive_can();

      /*
       * Add button/dial reads here once pin assignments are known.
       *
       * Call steering_send_buttons() and steering_send_dial()
       * on an agreed schedule, not on every loop iteration.
       */

      /* Add display and indicator updates here. */
    }
  }
} /* end main */


/**
  * brief: Read a big-endian unsigned 16-bit value.
  */
static uint16_t read_u16_be(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}


/**
  * brief: Write a big-endian unsigned 16-bit value.
  */
static void write_u16_be(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)value;
}


/**
  * brief: Process up to eight queued frames per main-loop iteration.
  */
static void receive_can(void)
{
  hal_fdcan_rx_header_t header;
  uint8_t data[64];
  uint32_t fill_level;
  uint32_t identifier;

  for (uint32_t i = 0U; i < 8U; i++)
  {
    HAL_FDCAN_GetRxFifoFillLevel(wheel_can, HAL_FDCAN_RX_FIFO_0,
                               &fill_level);

    if (fill_level == 0U)
    {
      break;
    }

    if (HAL_FDCAN_GetReceivedMessage(wheel_can, HAL_FDCAN_RX_FIFO_0,
                                     &header, data) != HAL_OK)
    {
      vehicle.rx_errors++;
      break;
    }

    vehicle.rx_count++;

    /* Only decode standard, classic CAN data frames here. */
    if ((header.b.identifier_type != HAL_FDCAN_ID_STANDARD) ||
        (header.b.frame_type != HAL_FDCAN_FRAME_DATA) ||
        (header.b.frame_format != 0U) ||
        (header.b.data_length != 8U))
    {
      continue;
    }

    identifier = header.b.identifier >> 18;

    switch (identifier)
    {
      case CAN_ID_ENGINE_DATA:
        vehicle.engine_rpm = read_u16_be(&data[0]);
        vehicle.engine_data_received = 1U;
        break;

      case CAN_ID_TEMP_AND_VOLTAGE:
        vehicle.coolant_temp_c = (int16_t)data[0] - 40;
        vehicle.oil_temp_c = (int16_t)data[1] - 40;
        vehicle.ecu_voltage_v = (float)data[5] * 0.1f;
        vehicle.temp_data_received = 1U;
        break;

      case CAN_ID_STATES:
        vehicle.gear = data[6] & 0x0FU;
        vehicle.gear_data_received = 1U;
        break;

      default:
        break;
    }
  }
}


/**
  * brief: Queue four raw button values using the DBC byte layout.
  * note:  HAL_OK means queued, not confirmed delivered on the bus.
  */
hal_status_t steering_send_buttons(uint16_t button1, uint16_t button2,
                                   uint16_t button3, uint16_t button4)
{
  hal_fdcan_tx_header_t header = {0};
  uint8_t data[8];

  header.b.identifier = CAN_ID_STEERING_BUTTONS << 18;
  header.b.identifier_type = HAL_FDCAN_ID_STANDARD;
  header.b.frame_type = HAL_FDCAN_FRAME_DATA;

  /* Classic CAN format, no BRS, no Tx event; DLC 8 = eight bytes. */
  header.b.data_length = (hal_fdcan_data_length_code_t)8U;

  write_u16_be(&data[0], button1);
  write_u16_be(&data[2], button2);
  write_u16_be(&data[4], button3);
  write_u16_be(&data[6], button4);

  return HAL_FDCAN_ReqTransmitMsgFromFIFOQ(wheel_can, &header, data);
}


/**
  * brief: Queue the raw dial value using the DBC byte layout.
  * note:  Physical DBC value = dial_raw * 0.001.
  */
hal_status_t steering_send_dial(uint16_t dial_raw)
{
  hal_fdcan_tx_header_t header = {0};
  uint8_t data[2];

  header.b.identifier = CAN_ID_STEERING_DIAL << 18;
  header.b.identifier_type = HAL_FDCAN_ID_STANDARD;
  header.b.frame_type = HAL_FDCAN_FRAME_DATA;

  /* Classic CAN format, no BRS, no Tx event; DLC 2 = two bytes. */
  header.b.data_length = (hal_fdcan_data_length_code_t)2U;

  write_u16_be(data, dial_raw);

  return HAL_FDCAN_ReqTransmitMsgFromFIFOQ(wheel_can, &header, data);
}