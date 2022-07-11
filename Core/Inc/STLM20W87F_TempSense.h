/*
 * STLM20W87F_TempSense.h
 *
 *  Created on: Jul 11, 2022
 *      Author: vkaiser
 */

#ifndef INC_STLM20W87F_TEMPSENSE_H_
#define INC_STLM20W87F_TEMPSENSE_H_

// Enabled VRefInt Channel = 3.6V in CubeMX
//12bit ADC = 4095 / 3.6VRef
#define ADC_SCALING_FACTOR	1137.5


float Get_ADC_Temp();

#endif /* INC_STLM20W87F_TEMPSENSE_H_ */
