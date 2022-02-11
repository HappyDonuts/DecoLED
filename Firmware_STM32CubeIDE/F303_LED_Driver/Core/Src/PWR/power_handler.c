/*
 * power_handler.c
 *
 *  Created on: Jul 20, 2021
 *      Author: Javier.MORALES
 */

#include "power_handler.h"


/**
 * @brief  Creates power_handler_t variable
 * @param  *hrtc: RTC peripheral
 * @retval power_handler_t pointer to power_handler structure
 */
power_handle_t* power_handle_new(RTC_HandleTypeDef *hrtc)
{
	power_handle_t* power_handle = malloc(sizeof(*power_handle));
	power_handle_init(power_handle, hrtc);
	return power_handle;
}

/**
 * @brief  Initializes power handler struct
 * @param  *power_handler: pointer to power_handler structure
 * @param  *hrtc: RTC peripheral
 * @retval none
 */
void power_handle_init(power_handle_t* power_handle, RTC_HandleTypeDef *hrtc)
{
	power_handle->hrtc = hrtc;
}

/**
 * @brief  Initializes the date and time stored in the RTC. Call once at reset
 * @param  *power_handler: pointer to power_handler structure
 * @retval none
 */
void ph_start_RTC_date(power_handle_t* power_handle)
{
	sDate.Date = DAY;
	sDate.Month = MONTH;
	sDate.Year = YEAR;
	sTime.Hours = HOURS;
	sTime.Minutes = MINUTES;
	sTime.Seconds = SECONDS;
	HAL_RTC_SetTime(power_handle->hrtc, &sTime, RTC_FORMAT_BCD);
	HAL_RTC_SetDate(power_handle->hrtc, &sDate, RTC_FORMAT_BCD);

	uint8_t hours_dec = (10*(sTime.Hours / 16) + (sTime.Hours % 16));
	uint8_t minutes_dec = (10*(sTime.Minutes / 16) + (sTime.Minutes % 16));
	uint8_t seconds_dec = (10*(sTime.Seconds / 16) + (sTime.Seconds % 16));

	ph_set_wakeup_date(power_handle, hours_dec, minutes_dec, seconds_dec);

	HAL_RTCEx_BKUPWrite(power_handle->hrtc, RTC_BKP_DR3, N_BURST_MAX);
}


/**
 * @brief  Sets a date and time at which the system will wake up when in stanby mode
 * @note   Data is stored in RTC backup registers not lost during reset
 * @param  *power_handler: pointer to power_handler structure
 * @param  hours: Wake up hours
 * @param  minutes: Wake up minutes
 * @param  seconds: Wake up seconds
 * @retval none
 */
void ph_set_wakeup_date(power_handle_t* power_handle, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
	HAL_RTCEx_BKUPWrite(power_handle->hrtc, RTC_BKP_DR0, seconds);
	HAL_RTCEx_BKUPWrite(power_handle->hrtc, RTC_BKP_DR1, minutes);
	HAL_RTCEx_BKUPWrite(power_handle->hrtc, RTC_BKP_DR2, hours);
}

/**
 * @brief  Sets a period of time after which the system will wake up when in standby mode
 * @note   Data is stored in RTC backup registers not lost during reset
 * @param  *power_handler: pointer to power_handler structure
 * @param  period: time to wait from the actual wake up to the next one (seconds)
 * @retval none
 */
void ph_set_wakeup_period(power_handle_t* power_handle, uint32_t period)
{
	uint8_t prev_seconds = HAL_RTCEx_BKUPRead(power_handle->hrtc, RTC_BKP_DR0);
	uint8_t prev_minutes = HAL_RTCEx_BKUPRead(power_handle->hrtc, RTC_BKP_DR1);
	uint8_t prev_hours = HAL_RTCEx_BKUPRead(power_handle->hrtc, RTC_BKP_DR2);

	uint8_t p_hours = period / 3600;
	uint8_t p_minutes = period / 60 - p_hours*60;
	uint8_t p_seconds = period - p_hours*3600 - p_minutes*60;

	uint8_t new_hours = prev_hours + p_hours;
	uint8_t new_minutes = prev_minutes + p_minutes;
	uint8_t new_seconds = prev_seconds + p_seconds;

	if (new_seconds > 59) {
			new_seconds -= 60;
			new_minutes++;
	}

	if (new_minutes > 59) {
		new_minutes -= 60;
		new_hours++;
	}

	if (new_hours > 23) {
		new_hours = 0;
	}

	ph_set_wakeup_date(power_handle, new_hours, new_minutes, new_seconds);
}

/**
 * @brief  Sets a period of time after which the system will wake up from standby mode and transmit in burst mode
 * @note   Data is stored in RTC backup registers not lost during reset
 * @note   N_BURST_MAX defines the number of transmission of a burst
 * @param  *power_handler: pointer to power_handler structure
 * @param  period: time to wait between bursts (seconds)
 * @retval none
 */
void ph_set_wakeup_burst(power_handle_t* power_handle, uint32_t period)
{
	uint8_t n_burst = HAL_RTCEx_BKUPRead(power_handle->hrtc, RTC_BKP_DR3);
	if (n_burst == 1) {
		ph_set_wakeup_period(power_handle, period - 60*(N_BURST_MAX - 1));
		n_burst = N_BURST_MAX;
	} else {
		ph_set_wakeup_period(power_handle, 60);
		n_burst--;
	}
	HAL_RTCEx_BKUPWrite(power_handle->hrtc, RTC_BKP_DR3, n_burst);
}

/**
 * @brief  Gets the time (ms) to wait from the current moment to the next wake up date
 * @param  *power_handler: pointer to power_handler structure
 * @retval Number of milliseconds to wait
 */
uint32_t ph_get_time_delay(power_handle_t* power_handle)
{
	uint8_t seconds = HAL_RTCEx_BKUPRead(power_handle->hrtc, RTC_BKP_DR0);
	uint8_t minutes = HAL_RTCEx_BKUPRead(power_handle->hrtc, RTC_BKP_DR1);
	uint8_t hours = HAL_RTCEx_BKUPRead(power_handle->hrtc, RTC_BKP_DR2);

	HAL_RTC_GetTime(power_handle->hrtc, &sTime, RTC_FORMAT_BCD);
	int8_t dif_sec = seconds - (10*(sTime.Seconds / 16) + (sTime.Seconds % 16));
	int8_t dif_minutes = minutes - (10*(sTime.Minutes / 16) + (sTime.Minutes % 16));
	int8_t dif_hours = hours - (10*(sTime.Hours / 16) + (sTime.Hours % 16));
	if (dif_hours < 0) {
		dif_hours += 24;
	}
		int32_t n_seconds = dif_sec + dif_minutes*60 + dif_hours*3600;
		return (uint32_t)n_seconds;
}

/**
 * @brief  Prepares the system to enter standby mode
 * @param  seconds: Wake up seconds
 * @retval none
 */
void ph_enter_standby_mode(power_handle_t* power_handle)
{
	/* Calculate time until next wakeup */
	uint32_t standby_delay = ph_get_time_delay(power_handle);

	/* Disable all used wakeup sources*/
	HAL_RTCEx_DeactivateWakeUpTimer(power_handle->hrtc);

	/* Clear all related wakeup flags */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	/* Re-enable all used wakeup sources*/
	HAL_RTCEx_SetWakeUpTimer_IT(power_handle->hrtc, standby_delay, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

	/* Enter the Standby mode */
	HAL_PWR_EnterSTANDBYMode();
}
