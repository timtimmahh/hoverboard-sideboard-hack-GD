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

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "gd32f1x0.h"
#include "systick.h"
#include "i2c_it.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "mpu6050.h"
#include "mpu6050_dmp.h"

#ifdef SIDEBOARD_CONTROL
extern SerialSideboard Sideboard;
#elif defined(SERIAL_CONTROL)
extern OneWheelCommand command;
#endif
#ifdef AUX45_USE_USART
extern AUXSerialDebug auxSerial;
#endif

#ifdef SERIAL_FEEDBACK
extern SerialFeedback Feedback;
extern uint16_t
	timeoutCntSerial;                        // Timeout counter for Rx Serial command
extern uint8_t
	timeoutFlagSerial;                        // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
#endif

extern MPU_Data mpu;                                // holds the MPU-6050 data
extern ErrStatus
	mpuStatus;                            // holds the MPU-6050 status: SUCCESS or ERROR

FlagStatus sensor1,
	sensor2;                    // holds the sensor1 and sensor 2 values
FlagStatus sensor1_read,
	sensor2_read;        // holds the instantaneous Read for sensor1 and sensor 2

static uint32_t
	main_loop_counter;                    // main loop counter to perform task squeduling inside main()


int main(void) {
  systick_config();                                    // SysTick config
  gpio_config();                                        // GPIO config
  usart_nvic_config();                                // USART interrupt configuration
  usart_config(USART_MAIN, USART_MAIN_BAUD);            // USART config
  #ifdef AUX45_USE_USART
  usart_config(USART_AUX, USART_AUX_BAUD);
  #endif
  i2c_config();                                        // I2C config
  i2c_nvic_config();                                    // I2C interrupt configuration
  input_init();                                        // Input initialization

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  while (1) {

	delay_1ms(DELAY_IN_MAIN_LOOP);

	// ==================================== LEDs Handling ====================================
	// toggle_led(LED4_GPIO_Port, LED4_Pin); 		// Toggle BLUE1 LED
#ifdef SERIAL_FEEDBACK
	if (!timeoutFlagSerial) {
	  if (Feedback.cmdLed & LED1_SET)
		gpio_bit_set(LED1_GPIO_Port, LED1_Pin);
	  else gpio_bit_reset(LED1_GPIO_Port, LED1_Pin);

	  if (Feedback.cmdLed & LED2_SET)
		gpio_bit_set(LED2_GPIO_Port, LED2_Pin);
	  else gpio_bit_reset(LED2_GPIO_Port, LED2_Pin);

	  if (Feedback.cmdLed & LED3_SET)
		gpio_bit_set(LED3_GPIO_Port, LED3_Pin);
	  else gpio_bit_reset(LED3_GPIO_Port, LED3_Pin);

	  if (Feedback.cmdLed & LED4_SET)
		gpio_bit_set(LED4_GPIO_Port, LED4_Pin);
	  else gpio_bit_reset(LED4_GPIO_Port, LED4_Pin);

	  if (Feedback.cmdLed & LED5_SET)
		gpio_bit_set(LED5_GPIO_Port, LED5_Pin);
	  else gpio_bit_reset(LED5_GPIO_Port, LED5_Pin);

	  if (Feedback.cmdLed & LED4_SET)
		gpio_bit_set(AUX3_GPIO_Port, AUX3_Pin);
	  else gpio_bit_reset(AUX3_GPIO_Port, AUX3_Pin);
	}
#endif


	// ==================================== MPU-6050 Handling ====================================
#ifdef MPU_SENSOR_ENABLE
// Get MPU data. Because the MPU-6050 interrupt pin is not wired we have to check DMP data by pooling periodically
	if (SUCCESS == mpuStatus) {
	  mpu_get_data();
	} else if (ERROR == mpuStatus && main_loop_counter % 100 == 0) {
	  toggle_led(LED1_GPIO_Port,
				 LED1_Pin);                    // Toggle the Red LED every 100 ms
	}
	// Print MPU data to Console
	if (main_loop_counter % 50 == 0) {
	  mpu_print_to_console();
	}
#endif

	// ==================================== SENSORS Handling ====================================
	sensor1_read = gpio_input_bit_get(SENSOR1_GPIO_Port, SENSOR1_Pin);
	sensor2_read = gpio_input_bit_get(SENSOR2_GPIO_Port, SENSOR2_Pin);

	// SENSOR1
	if (sensor1 == RESET && sensor1_read == SET) {
	  // Sensor ACTIVE: Do something here (one time task on activation)
	  sensor1 = SET;
	  gpio_bit_set(LED4_GPIO_Port, LED4_Pin);
	  consoleLog("-- SENSOR 1 Active --\n");
	} else if (sensor1 == SET && sensor1_read == RESET) {
	  // Sensor DEACTIVE: Do something here (one time task on
	  // deactivatiWon)
	  sensor1 = RESET;
	  gpio_bit_reset(LED4_GPIO_Port, LED4_Pin);
	  consoleLog("-- SENSOR 1 Deactive --\n");
	}

	// SENSOR2
	if (sensor2 == RESET && sensor2_read == SET) {
	  // Sensor ACTIVE: Do something here (one time task on activation)
	  sensor2 = SET;
	  gpio_bit_set(LED5_GPIO_Port, LED5_Pin);
	  consoleLog("-- SENSOR 2 Active --\n");
	} else if (sensor2 == SET && sensor2_read == RESET) {
	  // Sensor DEACTIVE: Do something here (one time task on deactivation)
	  sensor2 = RESET;
	  gpio_bit_reset(LED5_GPIO_Port, LED5_Pin);
	  consoleLog("-- SENSOR 2 Deactive --\n");
	}

	if (sensor1 == SET) {
	  // Sensor ACTIVE: Do something here (continuous task)
	}
	if (sensor2 == SET) {
	  // Sensor ACTIVE: Do something here (continuous task)
	}


	// ==================================== SERIAL Tx/Rx Handling ====================================
	#if defined(SERIAL_CONTROL) || defined(SIDEBOARD_CONTROL)
	// To transmit on USART
	// Check if DMA channel counter is 0 (meaning all data has been transferred)
	if (main_loop_counter % 5 == 0
		&& dma_transfer_number_get(DMA_CH3) == 0) {
	  #if defined(SIDEBOARD_CONTROL)
	  Sideboard.start = (uint16_t)SERIAL_START_FRAME;
	  Sideboard.roll = (int16_t)mpu.euler.roll;
	  Sideboard.pitch = (int16_t)mpu.euler.pitch;
	  Sideboard.yaw = (int16_t)mpu.euler.yaw;
	  Sideboard.sensor1 = (uint8_t)(sensor1 == SET);
	  Sideboard.sensor2 = (uint8_t)(sensor2 == SET);
	  Sideboard.mpuStatus = (uint8_t)(mpuStatus == SUCCESS);
	  Sideboard.checksum =
		  (uint16_t)(Sideboard.start ^ Sideboard.roll ^ Sideboard.pitch
			  ^ Sideboard.yaw ^ Sideboard.sensor1 ^ Sideboard.sensor2
			  ^ Sideboard.mpuStatus);
	  #elif defined(SERIAL_CONTROL)
	  command.start = (uint16_t)SERIAL_START_FRAME;
	  command.s1Enabled = sensor1 == SET;
	  command.s2Enabled = sensor2 == SET;
	  command.sensorPressed = sensor1 == SET && sensor2 == SET;
	  command.speed = (command.s1Enabled && command.s2Enabled)
					  ? (mpu.euler.pitch / 9.0) : 0;
	  command.checksum =
		  (uint16_t)command.start ^ command.s1Enabled ^ command.s2Enabled
			  ^ command.sensorPressed ^ command.speed;
	  #endif

	  dma_channel_disable(DMA_CH3);
	  #ifdef SIDEBOARD_CONTROL
	  DMA_CHCNT(DMA_CH3) = sizeof(Sideboard);
	  DMA_CHMADDR(DMA_CH3) = (uint32_t)&Sideboard;
	  #elif defined(SERIAL_CONTROL)
	  DMA_CHCNT(DMA_CH3) = sizeof(command);
	  DMA_CHMADDR(DMA_CH3) = (uint32_t)&command;
	  #endif
	  dma_channel_enable(DMA_CH3);
	}
	#endif

	#if defined(AUX45_USE_USART)
	// To transmit on USART
	// Check if DMA channel counter is 0 (meaning all data has been transferred)
	if (main_loop_counter % 5 == 0
		&& dma_transfer_number_get(DMA_CH1) == 0) {
	  auxSerial.start = (uint16_t)SERIAL_START_FRAME;
	  #if defined(SIDEBOARD_CONTROL)
	  auxSerial.roll = Sideboard.roll;
	  auxSerial.pitch = Sideboard.pitch;
	  auxSerial.yaw = Sideboard.yaw;
	  auxSerial.sensor1 = Sideboard.sensor1;
	  auxSerial.sensor2 = Sideboard.sensor2;
	  auxSerial.mpuStatus = Sideboard.mpuStatus;
	  auxSerial.checksum = (uint16_t)(auxSerial.start ^ auxSerial.roll
		  ^ auxSerial.pitch ^ auxSerial.yaw ^ auxSerial.sensor1
		  ^ auxSerial.sensor2 ^ auxSerial.mpuStatus);
	  #elif defined(SERIAL_CONTROL)
	  auxSerial.speed = command.speed;
	  auxSerial.s1Enabled = command.s1Enabled;
	  auxSerial.s2Enabled = command.s2Enabled;
	  auxSerial.sensorPressed = command.sensorPressed;
	  auxSerial.checksum = (uint16_t)(auxSerial.start ^ auxSerial.s1Enabled
		  ^ auxSerial.s2Enabled ^ auxSerial.sensorPressed ^ auxSerial.speed);
	  #endif
	  #if defined(SERIAL_FEEDBACK)
	  auxSerial.cmd1 = Feedback.cmd1;
	  auxSerial.cmd2 = Feedback.cmd2;
	  auxSerial.speedR_meas = Feedback.speedR_meas;
	  auxSerial.speedL_meas = Feedback.speedL_meas;
	  auxSerial.batVoltage = Feedback.batVoltage;
	  auxSerial.boardTemp = Feedback.boardTemp;
	  auxSerial.cmdLed = Feedback.cmdLed;
	  auxSerial.checksum ^= (uint16_t)(auxSerial.cmd1 ^ auxSerial.cmd2
		  ^ auxSerial.speedR_meas ^ auxSerial.speedL_meas
		  ^ auxSerial.batVoltage ^ auxSerial.boardTemp
		  ^ auxSerial.cmdLed);
	  #endif

	  dma_channel_disable(DMA_CH1);
	  DMA_CHCNT(DMA_CH1) = sizeof(auxSerial);
	  DMA_CHMADDR(DMA_CH1) = (uint32_t)&auxSerial;
	  dma_channel_enable(DMA_CH1);
	}
	#endif

#ifdef SERIAL_FEEDBACK
	if (timeoutCntSerial++
		>= SERIAL_TIMEOUT) {                // Timeout qualification
	  timeoutFlagSerial =
		  1;                                // Timeout detected
	  timeoutCntSerial =
		  SERIAL_TIMEOUT;                    // Limit timout counter value
	}
	if (timeoutFlagSerial && main_loop_counter % 100
		== 0) {    // In case of timeout bring the system to a Safe State and indicate error if desired
	  toggle_led(LED3_GPIO_Port,
				 LED3_Pin);                    // Toggle the Yellow LED every 100 ms
	}
#endif

	main_loop_counter++;

  }
#pragma clang diagnostic pop
}


