/*
 * ADXL355.c
 *
 *  Created on: Jan 14, 2019
 *      Author: rechegaray
 */
/*****************************************************************************
 * @file:    ADXL355.c
 * @brief:   ADXL355 accelerometer IC
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2016-2017 Analog Devices, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.
THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *****************************************************************************/

/***************************** Include Files **********************************/
#include <stdio.h>
#include "ADXL355.h"

#include "main.h"
/****************************** Global Data ***********************************/

int32_t volatile i32SensorX;
int32_t volatile i32SensorY;
int32_t volatile i32SensorZ;
int32_t volatile i32SensorT;
uint32_t volatile ui32SensorX;
uint32_t volatile ui32SensorY;
uint32_t volatile ui32SensorZ;
uint32_t volatile ui32SensorT;
int debug = 0;

volatile uint32_t ui32timer_counter = 0;

UART_HandleTypeDef huart2;



/************************* Global scope functions *****************************/

/**
   @brief Initialization the accelerometer sensor
   @return none
**/
void ADXL355_Init(void) {
//   DioPulPin(CSACC_PORT, CSACC_PIN_NUMBER, 0);          /* Disable the internal pull up on CSACC pin */
//   DioOenPin(CSACC_PORT, CSACC_PIN_NUMBER, 1);          /* Set CSACC pin as output */

//   DioPulPin(INT1ACC_PORT, INT1ACC_PIN_NUMBER, 0);         /* Disable the internal pull up on INT1ACC pin */
//   DioOenPin(INT1ACC_PORT, INT1ACC_PIN_NUMBER, 0);         /* Set INT1ACC pin as input */

//   DioPulPin(INT2ACC_PORT, INT2ACC_PIN_NUMBER, 0);         /* Disable the internal pull up on INT2ACC pin */
//   DioOenPin(INT2ACC_PORT, INT2ACC_PIN_NUMBER, 0);         /* Set INT2ACC pin as input */

//   DioPulPin(DATARDYACC_PORT, DATARDYACC_PIN_NUMBER, 0);         /* Disable the internal pull up on INT2ACC pin */
//   DioOenPin(DATARDYACC_PORT, DATARDYACC_PIN_NUMBER, 0);         /* Set INT2ACC pin as input */

	/* Quick verification test for boards */

   printf("\n");
   uint32_t volatile ui32test = ADXL355_SPI_Read(DEVID_AD);                  /* Read the ID register */
   uint32_t volatile ui32test2 = ADXL355_SPI_Read(DEVID_MST);                  /* Read the ID register */
   uint32_t volatile ui32test3 = ADXL355_SPI_Read(PARTID);                  /* Read the ID register */
   uint32_t volatile ui32test4 = ADXL355_SPI_Read(REVID);                 /* Read the ID register */

   if ((ui32test == 0xAD) && (ui32test2 == 0x1D) && (ui32test3 == 0xED) && (ui32test4 == 0x01)) {
	   printf("\n\rReset and initialized.\n\r");
	   ADXL355_SPI_Write(0x2F, 0x52, 1); //reset
   }
   else
	   printf("Error initializing\n\r");
}

/**
   @brief Turns on accelerometer measurement mode.
   @return none
**/
void ADXL355_Start_Sensor(void) {
   uint8_t ui8temp;

   ui8temp = (uint8_t)ADXL355_SPI_Read(POWER_CTL);       /* Read POWER_CTL register, before modifying it */

   ui8temp = ui8temp & 0xFE;                                          /* Set measurement bit in POWER_CTL register */

   ADXL355_SPI_Write(POWER_CTL, ui8temp, SPI_WRITE_ONE_REG);                    /* Write the new value to POWER_CTL register */

   printf("\n\rSensors started.\n\r");
}


/**
   @brief Puts the accelerometer into standby mode.
   @return none
**/
void ADXL355_Stop_Sensor(void) {
   uint8_t ui8temp;

   ui8temp = (uint8_t)ADXL355_SPI_Read(POWER_CTL);        /*Read POWER_CTL register, before modifying it */

   ui8temp = ui8temp | 0x01;                                      /* Clear measurement bit in POWER_CTL register */

   ADXL355_SPI_Write(POWER_CTL, ui8temp, SPI_WRITE_ONE_REG);                 /* Write the new value to POWER_CTL register */

   printf("\n\rSensors stopped.\n\r");
}

/**
   @brief Reads the accelerometer data.
   @return none
**/
void ADXL355_Data_Scan(void) {
      ui32SensorX = ADXL355_SPI_Read(XDATA3);
      ui32SensorY = ADXL355_SPI_Read(YDATA3);
      ui32SensorZ = ADXL355_SPI_Read(ZDATA3);
      ui32SensorT = ADXL355_SPI_Read(TEMP2);
      i32SensorX = ADXL355_Acceleration_Data_Conversion(ui32SensorX);
      i32SensorY = ADXL355_Acceleration_Data_Conversion(ui32SensorY);
      i32SensorZ = ADXL355_Acceleration_Data_Conversion(ui32SensorZ);
}


/**
   @brief Convert the two's complement data in X,Y,Z registers to signed integers
   @param ui32SensorData - raw data from register
   @return int32_t - signed integer data
**/
int32_t ADXL355_Acceleration_Data_Conversion (uint32_t ui32SensorData)
{
   int32_t volatile i32Conversion = 0;

   ui32SensorData = ( ui32SensorData >> 4);

   ui32SensorData = (ui32SensorData & 0x000FFFFF);

   if((ui32SensorData & 0x00080000)  == 0x00080000){ //checking if most sig bit is set
         i32Conversion = (ui32SensorData | 0xFFF00000); //if its set, we try to make it negative
   }
   else{
         i32Conversion = ui32SensorData;
   }

   //printf("\n0x%lX\n\r", ui32SensorData);

   return i32Conversion;
}

void ADXL355_Set_Range (uint8_t range) {

	uint32_t val = ADXL355_SPI_Read(0x2C);

	switch ( range ) {

	case 2 :
		val = (val & 0xFFFFFFFC) | 1;
		break;

	case 4 :
		val = (val & 0xFFFFFFFC) | 2;
		break;

	case 8:
		val = (val & 0xFFFFFFFC) | 3;
		break;

	default :
		printf("\n\rInvalid input - only 2, 4, or 8 for the range\n\r");
		return;
	}
	ADXL355_SPI_Write(0x2C, val, 1);
}

uint32_t ADXL355_Read_Range (void) {
	uint32_t range;
	range = ADXL355_SPI_Read(0x2C);
	range = range & 3;

	if ( range == 1 )
		return 2;
	else if ( range == 2 )
		return 4;
	else if ( range == 3 )
		return 8;
	else return 0; //error
}




