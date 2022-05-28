/*
 * power_handler.h
 *
 *  Created on: Jul 20, 2021
 *      Author: Javier.MORALES
 */

#ifndef INC_POWER_HANDLER_H_
#define INC_POWER_HANDLER_H_

#include "main.h"
#include <stdlib.h>

/* Starting date and time - Set manually */
#define SECONDS 0x00
#define MINUTES 0x00
#define HOURS	0x00

#define DAY		0x07
#define MONTH	0x02
#define YEAR	0X22

#define N_BURST_MAX 60

/**
 * @brief  power_handle_t struct
 */
typedef struct power_handle_t{
	RTC_HandleTypeDef *hrtc;
	UART_HandleTypeDef *huart;
} power_handle_t;


/**
 * @brief  Private time and date RTC variables
 */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;


/**
 * @brief  Creates power_handle_t variable
 * @param  *hrtc: RTC peripheral
 * @param  *huart: UART peripheral used for logging (can be NULL)
 * @retval power_handle_t pointer to power_handle structure
 */
power_handle_t* power_handle_new(RTC_HandleTypeDef *hrtc, UART_HandleTypeDef *huart);

/**
 * @brief  Initializes power handle struct
 * @param  *power_handle: pointer to power_handle structure
 * @param  *hrtc: RTC peripheral
 * @param  *huart: UART peripheral used for logging (can be NULL)
 * @retval none
 */
void power_handle_init(power_handle_t* power_handle, RTC_HandleTypeDef *hrtc, UART_HandleTypeDef *huart);

/**
 * @brief  Initializes the date and time stored in the RTC. Call once at reset
 * @param  *power_handle: pointer to power_handle structure
 * @retval none
 */
void ph_start_RTC_date(power_handle_t* power_handle);

/**
 * @brief  Prints date and time through UART for logging purposes
 * @param  *power_handle: pointer to power_handle structure
 * @retval none
 */
void ph_log_RTC_date(power_handle_t* power_handle);

/**
 * @brief  Sets a date and time at which the system will wake up when in standby mode
 * @note   Data is stored in RTC backup registers not lost during reset
 * @param  *power_handle: pointer to power_handle structure
 * @param  hours: Wake up hours
 * @param  minutes: Wake up minutes
 * @param  seconds: Wake up seconds
 * @retval none
 */
void ph_set_wakeup_date(power_handle_t* power_handle, uint8_t hours, uint8_t minutes, uint8_t seconds);

/**
 * @brief  Sets a period of time after which the system will wake up when in standby mode
 * @note   Data is stored in RTC backup registers not lost during reset
 * @param  *power_handle: pointer to power_handle structure
 * @param  period: time to wait from the actual wake up to the next one (seconds)
 * @retval none
 */
void ph_set_wakeup_period(power_handle_t* power_handle, uint32_t period);

/**
 * @brief  Sets a period of time after which the system will wake up from standby mode and transmit in burst mode
 * @note   Data is stored in RTC backup registers not lost during reset
 * @note   N_BURST_MAX defines the number of transmission of a burst
 * @param  *power_handle: pointer to power_handle structure
 * @param  period: time to wait between bursts (seconds)
 * @retval none
 */
void ph_set_wakeup_burst(power_handle_t* power_handle, uint32_t period);

/**
 * @brief  Gets the time (ms) to wait from the current moment to the next wake up date
 * @param  *power_handle: pointer to power_handle structure
 * @retval Number of milliseconds to wait
 */
uint32_t ph_get_time_delay(power_handle_t* power_handle);

/**
 * @brief  Prepares the system to enter standby mode
 * @param  *power_handle: pointer to power_handle structure
 * @retval none
 */
void ph_enter_standby_mode(power_handle_t* power_handle);


#endif /* INC_POWER_HANDLER_H_ */
