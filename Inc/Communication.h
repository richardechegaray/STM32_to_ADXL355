/*!
 *****************************************************************************
 * @file:    AD7798.h
 * @brief:   AD7798 Driver
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2015-2017 Analog Devices, Inc.
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
 *
 *****************************************************************************/

#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

/* -------------------------------------------------------------------------*/
/* UART available modes */
//#define UART_INT_MODE       1 /* Enables using both RX & TX interrupts */
//#define UART_TX_INT_MODE      2 /* Enables using TX interrupts */
//#define UART_RX_INT_MODE      3 /* Enables using RX interrupts */


/* The serial port may be used in polling or interrupt mode */
//#define UART_MODE UART_TX_INT_MODE
/* -------------------------------------------------------------------------*/

/* Execution status */
//#define UART_SUCCESS             0
//#define UART_FAILURE            -1
//#define UART_NO_TX_SPACE        -2
//#define UART_NO_RX_SPACE        -3

/* UART status */
//#define UART_TRUE               1
//#define UART_FALSE              0

/* Buffer size for UART Tx and Rx */
//#define UART_TX_BUFFER_SIZE      1024       // UART transmit buffer size
//#define UART_RX_BUFFER_SIZE      256        // UART receive buffer size

//extern unsigned int       uart_rpos, uart_rcnt, uart_tpos, uart_tcnt;
//extern unsigned int       uart_echo, uart_cmd, uart_ctrlc, uart_tbusy;

//extern unsigned char      uart_rx_buffer[UART_RX_BUFFER_SIZE];
//extern unsigned char      uart_tx_buffer[UART_TX_BUFFER_SIZE];


/*******************************************************************************
**************************** Internal types ************************************
********************************************************************************/

/* Write data mode */
typedef enum {
   SPI_WRITE_ONE_REG = 1,         /* Write 1 ACC register */
   SPI_WRITE_TWO_REG,             /* Write 2 ACC register */
//   UART_WRITE_NO_INT,             /* Write data when interrupts are disabled */
//   UART_WRITE_IN_INT,             /* Write data while in an interrupt routine */
//   UART_WRITE
   LAST
} enWriteData;

typedef enum {
   SPI_READ_ONE_REG = 1,            /* Read one ACC register */
   SPI_READ_TWO_REG,                /* Read two ACC registers */
   SPI_READ_THREE_REG,              /* Read X,Y,Z ACC registers */

} enRegsNum;


/*******************************************************************************
**************************** Internal definitions ******************************
********************************************************************************/

/* Accelerometer write command */
#define ADXL355_WRITE         0x0

/* Accelerometer read command */
#define ADXL355_READ          0x1

/*******************************************************************************
**************************** Functions declarations *****************************
********************************************************************************/

/* SPI Functions */
//void SPI_Init(void);
//void ADXL355_SPI_Write(uint8_t ui8address, uint8_t ui8Data, uint8_t ui8Data2, enWriteData enMode);
//uint32_t ADXL355_SPI_Read(uint8_t ui8address, uint8_t enRegs);

/* UART Functions */
/*void UART_Init(long lBaudrate, int iBits);
int UART_WriteChar(char data, enWriteData mode);
int UART_WriteString(char *string);
void UART_ReadChar(char *data);
void UART_Printf(const char *fmt, ...);*/

/*******************************************************************************
**************************** Configuration settings ****************************
********************************************************************************/


#endif /* _COMMUNICATION_H_ */
