/*
* This file is part of the hoverboard-firmware-hack-V2 project. The 
* firmware is used to hack the generation 2 board of the hoverboard.
* These new hoverboards have no mainboard anymore. They consist of 
* two Sensorboards which have their own BLDC-Bridge per Motor and an
* ARM Cortex-M3 processor GD32F130C8.
*
* Copyright (C) 2018 Florian Staeblein
* Copyright (C) 2018 Jakob Broemauer
* Copyright (C) 2018 Kai Liebich
* Copyright (C) 2018 Christoph Lehnert
*
* The program is based on the hoverboard project by Niklas Fauth. The 
* structure was tried to be as similar as possible, so that everyone 
* could find a better way through the code.
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

#include "gd32f1x0.h"
#include "../Inc/it.h"
#include "../Inc/comms.h"
#include "../Inc/commsSbus.h"
#include "../Inc/setup.h"
#include "../Inc/config.h"
#include "../Inc/defines.h"
#include "../Inc/bldc.h"
#include "stdio.h"
#include "string.h"

// Only master communicates with steerin device
#ifdef MASTER
#ifdef ENABLE_SBUS

#define SBUS_THROTTLE_CH 1
#define SBUS_STEER_CH 0
#define USART_STEER_TX_BYTES 2

extern uint8_t usartSteer_COM_rx_buf[USART_STEER_COM_RX_BUFFERSIZE];
// static uint8_t sSteerRecord = 0;
// static uint8_t sUSARTSteerRecordBuffer[USART_STEER_RX_BYTES];
// static uint8_t sUSARTSteerRecordBufferCounter = 0;

	uint8_t buffer[USART_STEER_TX_BYTES];
void SendSteerDevice(void)
{
	int index = 0;
	
	// // Ask for steer input
	// buffer[index++] = 0x0f; //'/';
	// buffer[index++] = '\n';
	buffer[0] = 170; //'/';
	SendBuffer(USART_STEER_COM, buffer, 1);
}

void CheckUSARTSteerInput(void);

extern int32_t steer;
extern int32_t speed;
int8_t state_ = 0;
uint8_t prev_byte_ = FOOTER_;
uint8_t cur_byte_;

bool UpdateUSARTSteerInput() {
  /* Parse messages */
  
    cur_byte_ = usartSteer_COM_rx_buf[0];

//  if (cur_byte_ ==  0x0f){
  
  
//   SendSteerDevice();
// }
    if (state_ == 0) {
      if ((cur_byte_ == HEADER_) && ((prev_byte_ == FOOTER_) ||
         ((prev_byte_ & 0x0F) == FOOTER2_))) {
        buf_[state_++] = cur_byte_;
      } else {
        state_ = 0;
      }
    } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_) {
        buf_[state_++] = cur_byte_;
    } else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_ + FOOTER_LEN_) {
      state_ = 0;
      
      prev_byte_ = cur_byte_;
      if ((cur_byte_ == FOOTER_) || ((cur_byte_ & 0x0F) == FOOTER2_)) {
        /* Grab the channel data */
        ch[0]  = (int16_t)(buf_[1] | ((buf_[2] << 8) & 0x07FF));
        ch[1]  = (int16_t)((buf_[2] >> 3) |
                                            ((buf_[3] << 5) & 0x07FF));
        ch[2]  = (int16_t)((buf_[3] >> 6) |
                                            (buf_[4] << 2) |
                                            ((buf_[5] << 10) & 0x07FF));
        ch[3]  = (int16_t)((buf_[5] >> 1) |
                                            ((buf_[6] << 7) & 0x07FF));
        ch[4]  = (int16_t)((buf_[6] >> 4) |
                                            ((buf_[7] << 4) & 0x07FF));
        ch[5]  = (int16_t)((buf_[7] >> 7) |
                                            (buf_[8] << 1) |
                                            ((buf_[9] << 9) & 0x07FF));
        ch[6]  = (int16_t)((buf_[9] >> 2) |
                                            ((buf_[10] << 6) & 0x07FF));
        ch[7]  = (int16_t)((buf_[10] >> 5) |
                                            ((buf_[11] << 3) & 0x07FF));
        ch[8]  = (int16_t)(buf_[12] |
                                             ((buf_[13] << 8) & 0x07FF));
        ch[9]  = (int16_t)((buf_[13] >> 3) |
                                            ((buf_[14] << 5) & 0x07FF));
        ch[10] = (int16_t)((buf_[14] >> 6) |
                                            (buf_[15] << 2) |
                                            ((buf_[16] << 10) & 0x07FF));
        ch[11] = (int16_t)((buf_[16] >> 1) |
                                            ((buf_[17] << 7) & 0x07FF));
        ch[12] = (int16_t)((buf_[17] >> 4) |
                                            ((buf_[18] << 4) & 0x07FF));
        ch[13] = (int16_t)((buf_[18] >> 7) |
                                            (buf_[19] << 1) |
                                            ((buf_[20] << 9) & 0x07FF));
        ch[14] = (int16_t)((buf_[20] >> 2) |
                                            ((buf_[21] << 6) & 0x07FF));
        ch[15] = (int16_t)((buf_[21] >> 5) |
                                            ((buf_[22] << 3) & 0x07FF));
        /* CH 17 */
        ch17 = buf_[23] & CH17_MASK_;
        // /* CH 18 */
        ch18 = buf_[23] & CH18_MASK_;
        // /* Grab the lost frame */
        lost_frame = buf_[23] & LOST_FRAME_MASK_;
        // /* Grab the failsafe */
        failsafe = buf_[23] & FAILSAFE_MASK_;
		    CheckUSARTSteerInput();
        return TRUE;
      } else {
        return FALSE;
      }
    } else {
      state_ = 0;
    }
    prev_byte_ = cur_byte_;
    return FALSE;
}

void CheckUSARTSteerInput(void)
{
	if(failsafe==FALSE && lost_frame==FALSE){
// SendSteerDevice();
		// Calculate result speed value -1000 to 1000
		speed = 1000-(int16_t)(ch[SBUS_THROTTLE_CH]);
		
		// Calculate result steering value -1000 to 1000
		steer = 1000-(int16_t)(ch[SBUS_STEER_CH]);

  	ResetTimeout();

	}
	return;
}
#endif
#endif
