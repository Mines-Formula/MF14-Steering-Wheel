/**
  ******************************************************************************
  * file           : main.c
  * brief          : CAN receive, six button reads, four rotary ADC reads,
  *                  and periodic control transmission.
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
#include "mx_gpio_default.h"
#include "mx_adc1.h"

#include <stddef.h>
#include <stdint.h>

/*
 ******************************************************************************
 *                       CAN MESSAGE LAYOUT
 ******************************************************************************
 *
 * IMPORTANT:
 *   The outgoing layout below is a PROPOSED application protocol.
 *   It has NOT been verified against the receiver's DBC.
 *
 *   - Confirm that CAN ID 0x7F3 is available before using it on the vehicle.
 *   - Update the receiver/DBC to accept all three outgoing messages.
 *   - 0x7F2 is now EIGHT bytes, replacing the previous TWO-byte dial message.
 *   - This file assumes the companion mx_adc1.c has been updated to configure
 *     PA1-PA4 as analog inputs and ADC1 for single-channel software conversion.
 *
 * FRAME FORMAT:
 *   Standard 11-bit identifiers.
 *   Classic CAN data frames.
 *   No CAN FD or bit-rate switching.
 *
 * MULTIBYTE VALUES:
 *   All outgoing 16-bit values are BIG-ENDIAN:
 *
 *     byte N     = high byte
 *     byte N + 1 = low byte
 *
 *     Example: value 0x1234 -> bytes 12 34
 *
 * ---------------------------------------------------------------------------
 * TRANSMIT: 0x7F1 -- BUTTONS 1 THROUGH 4
 * ---------------------------------------------------------------------------
 * Payload length: 8 bytes.
 * Proposed transmission period: 20 ms, approximately 50 Hz.
 *
 *   Byte(s)   Signal       Encoding
 *   -------   ----------   -----------------------------------------------
 *   0-1       Button 1     uint16_t, 0 = released, 1 = pressed
 *   2-3       Button 2     uint16_t, 0 = released, 1 = pressed
 *   4-5       Button 3     uint16_t, 0 = released, 1 = pressed
 *   6-7       Button 4     uint16_t, 0 = released, 1 = pressed
 *
 * Example: buttons 1 and 3 pressed:
 *   00 01 00 00 00 01 00 00
 *
 * ---------------------------------------------------------------------------
 * TRANSMIT: 0x7F3 -- BUTTONS 5 AND 6
 * ---------------------------------------------------------------------------
 * Payload length: 4 bytes.
 * Proposed transmission period: 20 ms, approximately 50 Hz.
 *
 *   Byte(s)   Signal       Encoding
 *   -------   ----------   -----------------------------------------------
 *   0-1       Button 5     uint16_t, 0 = released, 1 = pressed
 *   2-3       Button 6     uint16_t, 0 = released, 1 = pressed
 *
 * Example: buttons 5 and 6 pressed:
 *   00 01 00 01
 *
 * ---------------------------------------------------------------------------
 * TRANSMIT: 0x7F2 -- ALL FOUR ROTARY INPUTS
 * ---------------------------------------------------------------------------
 * Payload length: 8 bytes.
 * Proposed transmission period: 20 ms, approximately 50 Hz.
 * Sent only when the latest read attempt for EACH rotary succeeded.
 *
 *   Byte(s)   Signal       Encoding
 *   -------   ----------   -----------------------------------------------
 *   0-1       Rotary 1     uint16_t, raw ADC count
 *   2-3       Rotary 2     uint16_t, raw ADC count
 *   4-5       Rotary 3     uint16_t, raw ADC count
 *   6-7       Rotary 4     uint16_t, raw ADC count
 *
 * Nominal ADC range: 0-4095 with 12-bit resolution and no post-processing.
 *
 * These are RAW ADC COUNTS, not decoded rotary positions.
 * No 0.001 scale factor is applied by this firmware.
 * For a DBC displaying raw counts, use factor 1 and offset 0.
 *
 * Example: raw readings 0, 1024, 2048, 4095:
 *   00 00 04 00 08 00 0F FF
 *
 * ---------------------------------------------------------------------------
 * RECEIVE: EXISTING APPLICATION DECODING, RETAINED FROM ORIGINAL CODE
 * ---------------------------------------------------------------------------
 * Only standard, classic CAN data frames with eight-byte payloads are decoded.
 * These signal definitions have not been independently verified against a DBC.
 *
 *   ID       Byte(s)   Signal           Decoding
 *   ------   -------   ---------------  ----------------------------------
 *   0x640    0-1       Engine RPM       uint16_t big-endian
 *
 *   0x649    0         Coolant temp     byte value - 40 degrees C
 *   0x649    1         Oil temp         byte value - 40 degrees C
 *   0x649    5         ECU voltage      byte value * 0.1 volts
 *
 *   0x64D    6         Gear             low four bits: data[6] & 0x0F
 *
 * ---------------------------------------------------------------------------
 * TIMING AND DELIVERY NOTES
 * ---------------------------------------------------------------------------
 * - All six buttons are debounced using a 10 ms stable-state interval.
 * - One rotary is sampled every 5 ms; all four update in roughly 20 ms.
 * - Rotary readings are sequential, not simultaneous.
 * - Three outgoing frames are attempted every 20 ms.
 * - Frames are independent; they are not an atomic control snapshot.
 * - Button frames carry current states, not a queue of every button edge.
 * - HAL_OK from a transmit request means QUEUED, not confirmed delivered.
 * - Queue failures are counted and retried at the next scheduled interval.
 * - Actual bus timing depends on arbitration, bus load, and CAN configuration.
 *
 ******************************************************************************
 */

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

typedef struct
{
  uint8_t raw[6];
  uint8_t pressed[6];
} wheel_button_state_t;

typedef struct
{
  uint16_t raw[4];
  uint8_t valid[4];

  uint32_t read_count;
  uint32_t errors;
} wheel_rotary_state_t;

typedef struct
{
  uint32_t buttons_1_to_4_queued;
  uint32_t buttons_5_to_6_queued;
  uint32_t dials_queued;

  uint32_t buttons_1_to_4_errors;
  uint32_t buttons_5_to_6_errors;
  uint32_t dials_errors;

  uint32_t dials_skipped_invalid;
} wheel_tx_state_t;

/* Private define ------------------------------------------------------------*/
#define CAN_ID_ENGINE_DATA             0x640U
#define CAN_ID_TEMP_AND_VOLTAGE         0x649U
#define CAN_ID_STATES                   0x64DU

#define CAN_ID_STEERING_BUTTONS         0x7F1U
#define CAN_ID_STEERING_DIAL            0x7F2U
#define CAN_ID_STEERING_BUTTONS_EXTRA   0x7F3U

#define BUTTON_COUNT                   6U
#define BUTTON_DEBOUNCE_MS             10U

#define ROTARY_COUNT                   4U
#define ROTARY_SAMPLE_PERIOD_MS        5U
#define ROTARY_ADC_TIMEOUT_MS          2U

#define CONTROL_TX_PERIOD_MS           20U

/* Private variables ---------------------------------------------------------*/
static hal_fdcan_handle_t *wheel_can;
static hal_adc_handle_t *wheel_adc;

/* Inspect these variables in the debugger. */
static volatile wheel_vehicle_state_t vehicle = {0};
static volatile wheel_button_state_t buttons = {0};
static volatile wheel_rotary_state_t rotary = {0};
static volatile wheel_tx_state_t tx = {0};

/* Private function prototypes -----------------------------------------------*/
static uint16_t read_u16_be(const uint8_t *data);
static void write_u16_be(uint8_t *data, uint16_t value);

static void receive_can(void);
static void read_buttons(uint32_t now_ms);

static hal_status_t rotary_adc_startup(void);
static hal_status_t read_rotary_channel(hal_adc_channel_t channel,
                                       uint16_t *value);
static void read_rotaries(uint32_t now_ms);

static void transmit_controls(uint32_t now_ms);

hal_status_t steering_send_buttons(uint16_t button1,
                                   uint16_t button2,
                                   uint16_t button3,
                                   uint16_t button4);

static hal_status_t steering_send_buttons_extra(uint16_t button5,
                                                uint16_t button6);

static hal_status_t steering_send_dials(uint16_t dial1_raw,
                                       uint16_t dial2_raw,
                                       uint16_t dial3_raw,
                                       uint16_t dial4_raw);

/**
  * brief: Application entry point.
  */
int main(void)
{
  if (mx_system_init() != SYSTEM_OK)
  {
    return (-1);
  }

  wheel_can = mx_fdcan1_gethandle();

  if (wheel_can == NULL)
  {
    return (-2);
  }

  if (HAL_FDCAN_Start(wheel_can) != HAL_OK)
  {
    return (-3);
  }

  if (rotary_adc_startup() != HAL_OK)
  {
    return (-4);
  }

  while (1)
  {
    const uint32_t now_ms = HAL_GetTick();

    receive_can();
    read_buttons(now_ms);
    read_rotaries(now_ms);
    transmit_controls(now_ms);
  }
}

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
    fill_level = 0U;

    HAL_FDCAN_GetRxFifoFillLevel(wheel_can,
                                HAL_FDCAN_RX_FIFO_0,
                                &fill_level);

    if (fill_level == 0U)
    {
      break;
    }

    if (HAL_FDCAN_GetReceivedMessage(wheel_can,
                                     HAL_FDCAN_RX_FIFO_0,
                                     &header,
                                     data) != HAL_OK)
    {
      vehicle.rx_errors++;
      break;
    }

    vehicle.rx_count++;

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
  * brief: Read and debounce all six active-low buttons.
  *
  * Button 1: PA0
  * Button 2: PA5
  * Button 3: PA6
  * Button 4: PA7
  * Button 5: PB0
  * Button 6: PB15
  *
  * Values: 0 = released, 1 = pressed.
  */
static void read_buttons(uint32_t now_ms)
{
  static uint8_t candidate[BUTTON_COUNT] = {0};
  static uint32_t candidate_since_ms[BUTTON_COUNT] = {0};

  const uint8_t sampled[BUTTON_COUNT] =
  {
    (uint8_t)(HAL_GPIO_ReadPin(HAL_GPIOA, HAL_GPIO_PIN_0)
              == HAL_GPIO_PIN_RESET),

    (uint8_t)(HAL_GPIO_ReadPin(HAL_GPIOA, HAL_GPIO_PIN_5)
              == HAL_GPIO_PIN_RESET),

    (uint8_t)(HAL_GPIO_ReadPin(HAL_GPIOA, HAL_GPIO_PIN_6)
              == HAL_GPIO_PIN_RESET),

    (uint8_t)(HAL_GPIO_ReadPin(HAL_GPIOA, HAL_GPIO_PIN_7)
              == HAL_GPIO_PIN_RESET),

    (uint8_t)(HAL_GPIO_ReadPin(HAL_GPIOB, HAL_GPIO_PIN_0)
              == HAL_GPIO_PIN_RESET),

    (uint8_t)(HAL_GPIO_ReadPin(HAL_GPIOB, HAL_GPIO_PIN_15)
              == HAL_GPIO_PIN_RESET)
  };

  for (uint32_t i = 0U; i < BUTTON_COUNT; i++)
  {
    buttons.raw[i] = sampled[i];

    if (sampled[i] != candidate[i])
    {
      candidate[i] = sampled[i];
      candidate_since_ms[i] = now_ms;
    }

    if ((uint32_t)(now_ms - candidate_since_ms[i])
        >= BUTTON_DEBOUNCE_MS)
    {
      buttons.pressed[i] = candidate[i];
    }
  }
}

/**
  * brief: Activate, calibrate, then stop ADC1 before periodic reads.
  * note: mx_system_init() already calls mx_adc1_init().
  */
static hal_status_t rotary_adc_startup(void)
{
  hal_status_t status;
  hal_status_t stop_status;

  wheel_adc = mx_adc1_gethandle();

  if (wheel_adc == NULL)
  {
    return HAL_ERROR;
  }

  status = HAL_ADC_Start(wheel_adc);

  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_Calibrate(wheel_adc);
  stop_status = HAL_ADC_Stop(wheel_adc);

  return (status != HAL_OK) ? status : stop_status;
}

/**
  * brief: Select and read one rotary ADC channel.
  * note: ADC must be stopped on entry.
  */
static hal_status_t read_rotary_channel(hal_adc_channel_t channel,
                                       uint16_t *value)
{
  hal_adc_channel_config_t config = {0};
  hal_status_t status;
  hal_status_t cleanup_status;

  if ((wheel_adc == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  config.group = HAL_ADC_GROUP_REGULAR;
  config.sequencer_rank = 1U;
  config.sampling_time = HAL_ADC_SAMPLING_TIME_289CYCLES;
  config.input_mode = HAL_ADC_IN_SINGLE_ENDED;

  status = HAL_ADC_SetConfigChannel(wheel_adc, channel, &config);

  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_Start(wheel_adc);

  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_REG_StartConv(wheel_adc);

  if (status == HAL_OK)
  {
    status = HAL_ADC_REG_PollForConv(wheel_adc,
                                     ROTARY_ADC_TIMEOUT_MS);

    if (status == HAL_OK)
    {
      *value = (uint16_t)HAL_ADC_REG_ReadConversionData(wheel_adc);
    }

    cleanup_status = HAL_ADC_REG_StopConv(wheel_adc);

    if (status == HAL_OK)
    {
      status = cleanup_status;
    }
  }

  cleanup_status = HAL_ADC_Stop(wheel_adc);

  if (status == HAL_OK)
  {
    status = cleanup_status;
  }

  return status;
}

/**
  * brief: Read one rotary every 5 ms, cycling through all four.
  * note: Channel assignments match the companion mx_adc1.c patch.
  */
static void read_rotaries(uint32_t now_ms)
{
  static uint32_t last_read_ms = 0U;
  static uint32_t next_rotary = 0U;

  static const hal_adc_channel_t channels[ROTARY_COUNT] =
  {
    HAL_ADC_CHANNEL_1,
    HAL_ADC_CHANNEL_2,
    HAL_ADC_CHANNEL_3,
    HAL_ADC_CHANNEL_4
  };

  uint16_t value = 0U;

  if ((uint32_t)(now_ms - last_read_ms) < ROTARY_SAMPLE_PERIOD_MS)
  {
    return;
  }

  last_read_ms = now_ms;

  if (read_rotary_channel(channels[next_rotary], &value) == HAL_OK)
  {
    rotary.raw[next_rotary] = value;
    rotary.valid[next_rotary] = 1U;
    rotary.read_count++;
  }
  else
  {
    rotary.valid[next_rotary] = 0U;
    rotary.errors++;
  }

  next_rotary = (next_rotary + 1U) % ROTARY_COUNT;
}

/**
  * brief: Attempt all three control messages every 20 ms.
  * note: HAL_OK means queued, not confirmed delivered.
  */
static void transmit_controls(uint32_t now_ms)
{
  static uint32_t last_tx_ms = 0U;

  if ((uint32_t)(now_ms - last_tx_ms) < CONTROL_TX_PERIOD_MS)
  {
    return;
  }

  last_tx_ms = now_ms;

  if (steering_send_buttons(buttons.pressed[0],
                            buttons.pressed[1],
                            buttons.pressed[2],
                            buttons.pressed[3]) == HAL_OK)
  {
    tx.buttons_1_to_4_queued++;
  }
  else
  {
    tx.buttons_1_to_4_errors++;
  }

  if (steering_send_buttons_extra(buttons.pressed[4],
                                  buttons.pressed[5]) == HAL_OK)
  {
    tx.buttons_5_to_6_queued++;
  }
  else
  {
    tx.buttons_5_to_6_errors++;
  }

  if ((rotary.valid[0] != 0U) &&
      (rotary.valid[1] != 0U) &&
      (rotary.valid[2] != 0U) &&
      (rotary.valid[3] != 0U))
  {
    if (steering_send_dials(rotary.raw[0],
                            rotary.raw[1],
                            rotary.raw[2],
                            rotary.raw[3]) == HAL_OK)
    {
      tx.dials_queued++;
    }
    else
    {
      tx.dials_errors++;
    }
  }
  else
  {
    tx.dials_skipped_invalid++;
  }
}

/**
  * brief: Queue buttons 1 through 4 on CAN ID 0x7F1.
  */
hal_status_t steering_send_buttons(uint16_t button1,
                                   uint16_t button2,
                                   uint16_t button3,
                                   uint16_t button4)
{
  hal_fdcan_tx_header_t header = {0};
  uint8_t data[8];

  header.b.identifier = CAN_ID_STEERING_BUTTONS << 18;
  header.b.identifier_type = HAL_FDCAN_ID_STANDARD;
  header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  header.b.data_length = (hal_fdcan_data_length_code_t)8U;

  write_u16_be(&data[0], button1);
  write_u16_be(&data[2], button2);
  write_u16_be(&data[4], button3);
  write_u16_be(&data[6], button4);

  return HAL_FDCAN_ReqTransmitMsgFromFIFOQ(wheel_can,
                                          &header,
                                          data);
}

/**
  * brief: Queue buttons 5 and 6 on proposed CAN ID 0x7F3.
  */
static hal_status_t steering_send_buttons_extra(uint16_t button5,
                                                uint16_t button6)
{
  hal_fdcan_tx_header_t header = {0};
  uint8_t data[4];

  header.b.identifier = CAN_ID_STEERING_BUTTONS_EXTRA << 18;
  header.b.identifier_type = HAL_FDCAN_ID_STANDARD;
  header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  header.b.data_length = (hal_fdcan_data_length_code_t)4U;

  write_u16_be(&data[0], button5);
  write_u16_be(&data[2], button6);

  return HAL_FDCAN_ReqTransmitMsgFromFIFOQ(wheel_can,
                                          &header,
                                          data);
}

/**
  * brief: Queue all four raw rotary readings on CAN ID 0x7F2.
  * note: Eight-byte layout replaces the previous single-dial layout.
  */
static hal_status_t steering_send_dials(uint16_t dial1_raw,
                                       uint16_t dial2_raw,
                                       uint16_t dial3_raw,
                                       uint16_t dial4_raw)
{
  hal_fdcan_tx_header_t header = {0};
  uint8_t data[8];

  header.b.identifier = CAN_ID_STEERING_DIAL << 18;
  header.b.identifier_type = HAL_FDCAN_ID_STANDARD;
  header.b.frame_type = HAL_FDCAN_FRAME_DATA;
  header.b.data_length = (hal_fdcan_data_length_code_t)8U;

  write_u16_be(&data[0], dial1_raw);
  write_u16_be(&data[2], dial2_raw);
  write_u16_be(&data[4], dial3_raw);
  write_u16_be(&data[6], dial4_raw);

  return HAL_FDCAN_ReqTransmitMsgFromFIFOQ(wheel_can,
                                          &header,
                                          data);
}