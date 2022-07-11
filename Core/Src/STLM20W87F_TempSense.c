/*
 * STLM20W87F_TempSense.c
 *
 *  Created on: Jul 11, 2022
 *      Author: vkaiser
 *
 * Ultra-low current 2.4 V precision analog temperature sensor
 * Product Page:
 * https://www.st.com/en/mems-and-sensors/stlm20.html
 */

#include "STLM20W87F_TempSense.h"
#include "main.h"

ADC_HandleTypeDef adcHandle1;


// Read Value
// Convert
// Close

/*
 * Transfer Function - near 25C
 * VO = (–11.69mV) ⁄°C x T + 1.8663V
 * Solve for T
 *
 *
 *
 */

/******************************************************************************************************
* @fn					- Read_ADC_TempV
* @brief				- Read ADC Voltage
* @param[in]			- N/A
* @return				- ADC_Voltage
*
* @note					-
******************************************************************************************************/
float Read_ADC_TempV()
{
	// Temp Sensor Buffer:
	uint16_t ADC_VAL = 0;
	float ADC_Voltage = 0.0;

	HAL_ADC_Start(&adcHandle1);
	HAL_ADC_PollForConversion(&adcHandle1, HAL_MAX_DELAY);
	ADC_VAL = HAL_ADC_GetValue(&adcHandle1);

	HAL_ADC_Stop(&adcHandle1);

	//Divide by Scaling Factor to get Voltage
	ADC_Voltage = ADC_VAL / ADC_SCALING_FACTOR;

	return ADC_Voltage;
}

/******************************************************************************************************
* @fn					- Xvert_ADC_Temp
* @brief				- Convert ADC Voltage to Temperature deg F
* @param[in]			- N/A
* @return				- ADC_Temp:	deg F Temperature
*
* @note					-
******************************************************************************************************/
float Xvert_ADC_Temp(float ADC_Voltage)
{
	float ADC_Temp = 0.0;

	//Voltage to deg C
	ADC_Temp = ((ADC_Voltage - 1.8663)/(-0.01169));

	//Deg C to deg F
	ADC_Temp = ((ADC_Voltage * (9.0 / 5.0)) + 32.0);

	return ADC_Temp;
}

/******************************************************************************************************
* @fn					- Get_ADC_Temp
* @brief				- Get and Convert ADC Temperature
* @param[in]			- N/A
* @return				- Temperature def F
*
* @note					-
******************************************************************************************************/
float Get_ADC_Temp()
{
	float ADC_Voltage = 0.0;
	float ADC_Temp = 0.0;

	ADC_Voltage = Read_ADC_TempV();
	ADC_Temp = Xvert_ADC_Temp(ADC_Voltage);

	return ADC_Temp;

}

