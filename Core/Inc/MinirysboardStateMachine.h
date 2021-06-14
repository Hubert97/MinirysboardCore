/*
 * MinirysboardStateMachine.h
 *
 *  Created on: May 14, 2021
 *      Author: huber
 */

#ifndef INC_MINIRYSBOARDSTATEMACHINE_H_
#define INC_MINIRYSBOARDSTATEMACHINE_H_
#include "main.h"
#define analog_data uint16_t




struct MSM_InputsDataType
    {
	analog_data ADC_Input[12];
	/*
	0-2 Cell1Voltage[3];
	3-4 ChasisTmp[2];
	5-7 BoardTemp[3];
	8 RPILineVoltage;
	9 VBatLineVoltage;
	10 BatCurrentVoltage;
	*/

    } AnalogInputsData;

struct MSM_OutputsDataType
    {
	uint8_t Diode_PWM [3]; // 0-R 1-G 2-B
	uint8_t FanSpeed;	//set value from outside
	uint8_t FanPWM;		//Real speed sent to fan



    } OutputsDataType;


struct MSM_StateDataType
    {
	struct MSM_InputsDataType AnalogInputs;
	uint16_t FanSpeedRPM;
	struct MSM_OutputsDataType OutputData;

    } StateData;



    /**
      * @brief  Blindly copies data from one container to different - something like memcpy. Data type is analog_data
      * @note   This function will not check if its out of bounds caution required
      *
      * @param hadc ADC handle
      * @param pData Destination Buffer address.
      * @param Length Number of data to be transferred from ADC peripheral to memory
      *k
      */
void MSM_DataCopy(analog_data * Dest,const analog_data * Source, uint8_t NoOfBytes )
    {
    for(int i =0; i<NoOfBytes;++i)
	{
	Dest[i]=Source[i];
	}


    }



#endif /* INC_MINIRYSBOARDSTATEMACHINE_H_ */
