/**
  * This file is part of the hoverboard-sideboard-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>


/* Rx Structures USART */
#ifdef SIDEBOARD_CONTROL
typedef struct{
  uint16_t  start;
  int16_t   roll;
  int16_t   pitch;
  int16_t   yaw;
  uint8_t	sensor1;
  uint8_t	sensor2;
  uint8_t 	mpuStatus;
  uint16_t  checksum;
} SerialSideboard;
#endif
#ifdef SERIAL_CONTROL
typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
typedef struct {
    uint16_t start;
    int16_t  speed;
    bool     s1Enabled;
    bool     s2Enabled;
    bool     sensorPressed;
    uint16_t checksum;
} OneWheelCommand;
typedef struct {
    OneWheelCommand commandL;
    OneWheelCommand commandR;
} TwoWheelCommand;
#endif
#ifdef SERIAL_FEEDBACK
typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;
  uint16_t  checksum;
} SerialFeedback;
/* typedef struct tag_sensor_lights {
  unsigned char colour; // this byte gets bit 8 set (on 9 bit serial);
  unsigned char unknown;
  unsigned char flashcount; // non zero-> red flash number of times with pause
  unsigned char unknown1;
  unsigned char unknown2;
  unsigned char unknown3;
} SENSOR_LIGHTS; */
#endif
#ifdef AUX45_USE_USART
typedef struct {
  uint16_t start;
#ifdef SIDEBOARD_CONTROL
  int16_t   roll;
  int16_t   pitch;
  int16_t   yaw;
  uint8_t	sensor1;
  uint8_t	sensor2;
  uint8_t 	mpuStatus;
#endif
#ifdef SERIAL_CONTROL
  int16_t  speed;
  bool     s1Enabled;
  bool     s2Enabled;
  bool     sensorPressed;
#endif
#ifdef SERIAL_FEEDBACK
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;
#endif
  uint16_t checksum;
} AUXSerialDebug;
#endif

/* general functions */
void consoleLog(char *message);
void toggle_led(uint32_t gpio_periph, uint32_t pin);
void intro_demo_led(uint32_t tDelay);

/* input initialization function */
void input_init(void);

/* usart read functions */
void usart_rx_check(void);
#ifdef SERIAL_DEBUG
void usart_process_debug(uint8_t *userCommand, uint32_t len);
#endif
#ifdef SERIAL_FEEDBACK
void usart_process_data(SerialFeedback *Feedback_in, SerialFeedback *Feedback_out);
#endif

/* i2c write/read functions */
int8_t i2c_writeBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t i2c_writeByte (uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
int8_t i2c_writeBit  (uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t  data);
int8_t i2c_readBytes (uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t i2c_readByte  (uint8_t slaveAddr, uint8_t regAddr, uint8_t *data);
int8_t i2c_readBit   (uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

#ifdef AUX45_USE_I2C
int8_t i2c_aux_writeBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t i2c_aux_readBytes (uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
#endif

#endif

