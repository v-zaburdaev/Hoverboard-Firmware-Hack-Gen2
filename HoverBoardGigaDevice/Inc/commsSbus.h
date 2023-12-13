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

#ifndef COMMSBUS_H
#define COMMSBUS_H

#include "gd32f1x0.h"
#include "../Inc/config.h"

// Only master communicates with steering device
#ifdef MASTER
#ifdef ENABLE_SBUS

  #define REMOTE_BAUD 100000
  #define REMOTE_PARITY USART_PM_EVEN
  #define REMOTE_WORD_LENGTH USART_WL_9BIT
  #define REMOTE_STOP_BIT USART_STB_2BIT

  #define SBUS_THROTTLE_CH 1
  #define SBUS_STEER_CH 0


  #define NUM_CH 16
  #define PAYLOAD_LEN_ 23
  #define HEADER_LEN_ 1
  #define FOOTER_LEN_ 1
  /* SBUS message defs */
  #define NUM_SBUS_CH_ 16
  #define HEADER_ 0x0F
  #define FOOTER_ 0x00
  #define FOOTER2_ 0x04
  #define CH17_MASK_ 0x01
  #define CH18_MASK_ 0x02
  #define LOST_FRAME_MASK_ 0x04
  #define FAILSAFE_MASK_ 0x08
  
  bool lost_frame;
  bool failsafe;
  bool ch17, ch18;
  int16_t ch[NUM_CH];
  uint8_t buf_[25];
  bool new_data_;
  bool Parse();
//----------------------------------------------------------------------------
// Update USART steer input
//----------------------------------------------------------------------------
bool UpdateUSARTSbusInput(void);

//----------------------------------------------------------------------------
// Send frame to steer device
//----------------------------------------------------------------------------
// void SendSteerDevice(void);
#endif
#endif
#endif
