/*
 * RM3100MAG.c
 *
 *  Created on: May 12, 2024
 *      Author: ASUS
 */

#include "main.h"
#include "RM3100MAG.h"
#include "math.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"
#include "string.h"


#define SPI_SSN_HIGH 1
#define SPI_SSN_LOW 0

float gain = 38.00;  //Gain for 100 cycle count

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef currentTime;

extern uint32_t mag_X;
extern uint32_t mag_Y;
extern uint32_t mag_Z;
extern uint32_t magnitude;

extern int32_t signed_mag_X, signed_mag_Y, signed_mag_Z;

extern float X_axis;
extern float Y_axis;
extern float Z_axis;
extern float Magnitude;


uint8_t rm3100_MSB_X_buf[] = { rm3100_MSB_X };
uint8_t rm3100_MSB_Y_buf[] = { rm3100_MSB_Y };
uint8_t rm3100_MSB_Z_buf[] = { rm3100_MSB_Z };
uint8_t MSB_X_buf[] = { MSB_X };
uint8_t LSB_X_buf[] = { LSB_X };
uint8_t MSB_Y_buf[] = { MSB_Y };
uint8_t LSB_Y_buf[] = { LSB_Y };
uint8_t MSB_Z_buf[] = { MSB_Z };
uint8_t LSB_Z_buf[] = { LSB_Z };

uint8_t MSBX_r = 0x84;
uint8_t LSBX_r = 0x85;
uint8_t MSBY_r = 0x86;
uint8_t LSBY_r = 0x87;
uint8_t MSBZ_r = 0x88;
uint8_t LSBZ_r = 0x89;

uint8_t rm3100_cmm_buf[] = { rm3100_cmm };
uint8_t rm3100_cmm_mod_buf[] = { rm3100_cmm_Val };
uint8_t rm3100_tmrc_add_buf[] = { rm3100_tmrc_add };
uint8_t rm3100_tmrc_Val_buf[] = { rm3100_tmrc_Val };
uint8_t rm3100_tmrc_read_buf[] = { rm3100_tmrc_read };

uint8_t MRRW2_X_buf[] = { MRRW2_X };
uint8_t MRRW1_X_buf[] = { MRRW1_X };
uint8_t MRRW0_X_buf[] = { MRRW0_X };
uint8_t MRRW2_Y_buf[] = { MRRW2_Y };
uint8_t MRRW1_Y_buf[] = { MRRW1_Y };
uint8_t MRRW0_Y_buf[] = { MRRW0_Y };
uint8_t MRRW2_Z_buf[] = { MRRW2_Z };
uint8_t MRRW1_Z_buf[] = { MRRW1_Z };
uint8_t MRRW0_Z_buf[] = { MRRW0_Z };

extern uint8_t read_X2, read_X1, read_X0;
extern uint8_t read_Y2, read_Y1, read_Y0;
extern uint8_t read_Z2, read_Z1, read_Z0;

//Initiate Cycle Count
void SET_COUNT() {
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_HIGH); //SSN HIGN
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_LOW); //SSN LOW TO COMMUNICATE WITH SENSOR
	HAL_SPI_Transmit(&hspi1, rm3100_MSB_X_buf, sizeof(rm3100_MSB_X_buf), 1000); //Not need to necessary to send multiple value or register
	//Initiate Count cycle
	HAL_SPI_Transmit(&hspi1, MSB_X_buf, sizeof(MSB_X_buf), 1000);
	HAL_SPI_Transmit(&hspi1, LSB_X_buf, sizeof(LSB_X_buf), 1000);

	HAL_SPI_Transmit(&hspi1, MSB_Y_buf, sizeof(MSB_Y_buf), 1000);
	HAL_SPI_Transmit(&hspi1, LSB_Y_buf, sizeof(LSB_Y_buf), 1000);

	HAL_SPI_Transmit(&hspi1, MSB_Z_buf, sizeof(MSB_Z_buf), 1000);
	HAL_SPI_Transmit(&hspi1, LSB_Z_buf, sizeof(LSB_Z_buf), 1000);
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_HIGH);
}
//Initiate Continuous Measurements Mode

void Continuous_Mode() {
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_HIGH); //SSN HIGN
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_LOW); //SSN LOW TO COMMUNICATE WITH SENSOR
	//Initiate Continuous Measurements Mode
	HAL_SPI_Transmit(&hspi1, rm3100_cmm_buf, sizeof(rm3100_cmm_buf), 1000);
	HAL_SPI_Transmit(&hspi1, rm3100_cmm_mod_buf, sizeof(rm3100_cmm_mod_buf),
			1000);
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_HIGH);
}
//Initiate Time Measurements and Rate Control

void TMRC_Mode() {
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_HIGH);
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_LOW);
	//initiate Time Measurements and Reset Control
	HAL_SPI_Transmit(&hspi1, rm3100_tmrc_add_buf, sizeof(rm3100_tmrc_add_buf),
			1000);
	HAL_SPI_Transmit(&hspi1, rm3100_tmrc_Val_buf, sizeof(rm3100_tmrc_Val_buf),
			1000);
//	HAL_SPI_Transmit(&hspi1, rm3100_tmrc_read_buf, sizeof(rm3100_tmrc_read_buf),
//			1000);
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_HIGH);
}

//Read Measurements Results
void Mea_Result() {
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_HIGH);
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_LOW);

	HAL_SPI_Transmit(&hspi1, MRRW2_X_buf, sizeof(MRRW2_X_buf), 1000);
	HAL_SPI_Receive(&hspi1, &read_X2, sizeof(read_X2), 1000);
	HAL_SPI_Receive(&hspi1, &read_X1, sizeof(read_X1), 1000);
	HAL_SPI_Receive(&hspi1, &read_X0, sizeof(read_X0), 1000);
	HAL_SPI_Receive(&hspi1, &read_Y2, sizeof(read_Y2), 1000);
	HAL_SPI_Receive(&hspi1, &read_Y1, sizeof(read_Y1), 1000);
	HAL_SPI_Receive(&hspi1, &read_Y0, sizeof(read_Y0), 1000);
	HAL_SPI_Receive(&hspi1, &read_Z2, sizeof(read_Z2), 1000);
	HAL_SPI_Receive(&hspi1, &read_Z1, sizeof(read_Z1), 1000);
	HAL_SPI_Receive(&hspi1, &read_Z0, sizeof(read_Z0), 1000);
	HAL_GPIO_WritePin(GPIOB, SS4_Pin, SPI_SSN_HIGH);
}

void UART_TransmitString(char *str) {
    while (*str != '\0') {
        HAL_UART_Transmit(&huart2, (uint8_t*)str, 1, HAL_MAX_DELAY);
        str++;
    }
}

//Set time
void set_time (void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    sTime.Hours = 0x04; // set hours
    sTime.Minutes = 0x09; // set minutes
    sTime.Seconds = 0x30; // set seconds
//    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE; // Remove this line
//    sTime.TimeFormat = RTC_HOURFORMAT12_AM; // Remove this line
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
    sDate.Month = RTC_MONTH_MAY;
    sDate.Date = 0x15; // date
    sDate.Year = 0x24; // year
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
}


//Get time

void get_time(void)
{
    RTC_DateTypeDef gDate;
    RTC_TimeTypeDef gTime;
    /* Get the RTC current Time */
    HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
}

//Combine Measurements
void Comb_measurement() {

    // Get current time from RTC
    RTC_DateTypeDef gDate;
    RTC_TimeTypeDef gTime;
    // Get current time from RTC
    HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

    // Combine bytes for X-axis
    mag_X = (read_X2 << 16) | (read_X1 << 8) | read_X0;
    // Extend the sign bit for negative values
    signed_mag_X = (mag_X & 0x800000) ? (mag_X | 0xFF000000) : mag_X;

    // Combine bytes for Y-axis
    mag_Y = (read_Y2 << 16) | (read_Y1 << 8) | read_Y0;
    // Extend the sign bit for negative values
    signed_mag_Y = (mag_Y & 0x800000) ? (mag_Y | 0xFF000000) : mag_Y;

    // Combine bytes for Z-axis
    mag_Z = (read_Z2 << 16) | (read_Z1 << 8) | read_Z0;
    // Extend the sign bit for negative values
    signed_mag_Z = (mag_Z & 0x800000) ? (mag_Z | 0xFF000000) : mag_Z;

    // Calculate magnitude
    magnitude = sqrt((float)(signed_mag_X * signed_mag_X) + (signed_mag_Y * signed_mag_Y) + (signed_mag_Z * signed_mag_Z));

    // Calculate axis values
    X_axis = (float)signed_mag_X / gain;
    Y_axis = (float)signed_mag_Y / gain;
    Z_axis = (float)signed_mag_Z / gain;
    Magnitude = (float)magnitude / gain;




    //Debug print statements for results
    myDebug("%02d:%02d:%02d",gTime.Hours, gTime.Minutes, gTime.Seconds);
    myDebug(" %.2f  \t", X_axis);
    myDebug(" %.2f  \t", Y_axis);
    myDebug(" %.2f  \t", Z_axis);
    myDebug(" %.2f  \n", Magnitude);
}

void myDebug(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buffer[100];
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
    va_end(args);
						}

int bufferSize(char *buffer) {
    int i = 0;
    while (*buffer++ != '\0')
        i++;
    return i;
}




