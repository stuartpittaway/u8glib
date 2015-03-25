/*
  
  u8g_com_arduino_fast_parallel_3chipsel.c

  Universal 8bit Graphics Library
  
  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  **	Stuart Pittaway, March 2015, cloned u8g_com_arduino_fast_parallel.c to provide a third chip select
  ** 	for displays which use 3 KS0108 chips and 3 select pins.

  
  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/

#include "u8g.h"

#if  defined(ARDUINO)

#if ARDUINO < 100 
//#include <WProgram.h> 
#include <wiring_private.h> 
#include <pins_arduino.h> 
#else 
#include <Arduino.h> 
#endif


#define PIN_D0 8
#define PIN_D1 9
#define PIN_D2 10
#define PIN_D3 11
#define PIN_D4 4
#define PIN_D5 5
#define PIN_D6 6
#define PIN_D7 7

#define PIN_CS1 14
#define PIN_CS2 15
#define PIN_RW 16
#define PIN_DI 17
#define PIN_EN 18

//#define PIN_RESET


#if defined(__PIC32MX)
/* CHIPKIT PIC32 */
static volatile uint32_t *u8g_data_port[8];
static uint32_t u8g_data_mask[8];
#else
static volatile uint8_t *u8g_data_port[8];
static uint8_t u8g_data_mask[8];
#endif



static void u8g_com_arduino_fast_parallel_3chipsel_init(u8g_t *u8g)
{
  u8g_data_port[0] =  portOutputRegister(digitalPinToPort(u8g->pin_list[U8G_PI_D0]));
  u8g_data_mask[0] =  digitalPinToBitMask(u8g->pin_list[U8G_PI_D0]);
  u8g_data_port[1] =  portOutputRegister(digitalPinToPort(u8g->pin_list[U8G_PI_D1]));
  u8g_data_mask[1] =  digitalPinToBitMask(u8g->pin_list[U8G_PI_D1]);
  u8g_data_port[2] =  portOutputRegister(digitalPinToPort(u8g->pin_list[U8G_PI_D2]));
  u8g_data_mask[2] =  digitalPinToBitMask(u8g->pin_list[U8G_PI_D2]);
  u8g_data_port[3] =  portOutputRegister(digitalPinToPort(u8g->pin_list[U8G_PI_D3]));
  u8g_data_mask[3] =  digitalPinToBitMask(u8g->pin_list[U8G_PI_D3]);
  
  u8g_data_port[4] =  portOutputRegister(digitalPinToPort(u8g->pin_list[U8G_PI_D4]));
  u8g_data_mask[4] =  digitalPinToBitMask(u8g->pin_list[U8G_PI_D4]);
  u8g_data_port[5] =  portOutputRegister(digitalPinToPort(u8g->pin_list[U8G_PI_D5]));
  u8g_data_mask[5] =  digitalPinToBitMask(u8g->pin_list[U8G_PI_D5]);
  u8g_data_port[6] =  portOutputRegister(digitalPinToPort(u8g->pin_list[U8G_PI_D6]));
  u8g_data_mask[6] =  digitalPinToBitMask(u8g->pin_list[U8G_PI_D6]);
  u8g_data_port[7] =  portOutputRegister(digitalPinToPort(u8g->pin_list[U8G_PI_D7]));
  u8g_data_mask[7] =  digitalPinToBitMask(u8g->pin_list[U8G_PI_D7]);  
}

/* atomic protection must be done by calling function */
static void u8g_com_arduino_fast_write_data_pin_3c(uint8_t pin, uint8_t val)
{
  if ( val != 0 )
    *u8g_data_port[pin] |= u8g_data_mask[pin];
  else
    *u8g_data_port[pin] &= ~u8g_data_mask[pin];
}


void u8g_com_arduino_fast_parallel_write_3c(u8g_t *u8g, uint8_t val)
{
  U8G_ATOMIC_START();
  u8g_com_arduino_fast_write_data_pin_3c( 0, val&1 );
  val >>= 1;
  u8g_com_arduino_fast_write_data_pin_3c( 1, val&1 );
  val >>= 1;
  u8g_com_arduino_fast_write_data_pin_3c( 2, val&1 );
  val >>= 1;
  u8g_com_arduino_fast_write_data_pin_3c( 3, val&1 );
  val >>= 1;

  u8g_com_arduino_fast_write_data_pin_3c( 4, val&1 );
  val >>= 1;
  u8g_com_arduino_fast_write_data_pin_3c( 5, val&1 );
  val >>= 1;
  u8g_com_arduino_fast_write_data_pin_3c( 6, val&1 );
  val >>= 1;
  u8g_com_arduino_fast_write_data_pin_3c( 7, val&1 );
  val >>= 1;
  U8G_ATOMIC_END();
  
  /* EN cycle time must be 1 micro second */
  u8g_com_arduino_digital_write(u8g, U8G_PI_EN, HIGH);
  u8g_MicroDelay(); /* delay by 1000ns, reference: ST7920: 140ns, SBN1661: 100ns */
  u8g_com_arduino_digital_write(u8g, U8G_PI_EN, LOW);
  u8g_10MicroDelay(); /* ST7920 commands: 72us */
  u8g_10MicroDelay(); /* ST7920 commands: 72us */
}

uint8_t u8g_com_arduino_fast_parallel_3chipsel_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  switch(msg)
  {
    case U8G_COM_MSG_INIT:
      u8g_com_arduino_fast_parallel_3chipsel_init(u8g);
      /* setup the RW pin as output and force it to low */
      if ( u8g->pin_list[U8G_PI_RW] != U8G_PIN_NONE )
      {
        pinMode(u8g->pin_list[U8G_PI_RW], OUTPUT);
        u8g_com_arduino_digital_write(u8g, U8G_PI_RW, LOW);
      }
      /* set all pins (except RW pin) */
      u8g_com_arduino_assign_pin_output_high(u8g);  
  
      break;
    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_CHIP_SELECT:
      if ( arg_val == 0 )
      {
        /* disable all */
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS1, HIGH);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS2, HIGH);
		u8g_com_arduino_digital_write(u8g, U8G_PI_CS3, HIGH);
      }
      else if ( arg_val == 1 )
      {
        /* enable */
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS1, LOW);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS2, HIGH);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS3, HIGH);
      }
      else if ( arg_val == 2 )
      {
        /* enable */
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS1, HIGH);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS2, LOW);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS3, HIGH);
      }
      else if ( arg_val == 3 )
      {
        /* enable */
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS1, HIGH);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS2, HIGH);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS3, LOW);
      }
      else
      {
        /* enable all */
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS1, LOW);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS2, LOW);
        u8g_com_arduino_digital_write(u8g, U8G_PI_CS3, LOW);
      }
      break;
	  
    case U8G_COM_MSG_WRITE_BYTE:
      u8g_com_arduino_fast_parallel_write_3c(u8g, arg_val);
      break;
	  
    case U8G_COM_MSG_WRITE_SEQ:
      {
        register uint8_t *ptr = arg_ptr;
        while( arg_val > 0 )
        {
          u8g_com_arduino_fast_parallel_write_3c(u8g, *ptr++);
          arg_val--;
        }
      }
      break;
	  
    case U8G_COM_MSG_WRITE_SEQ_P:
      {
        register uint8_t *ptr = arg_ptr;
        while( arg_val > 0 )
        {
          u8g_com_arduino_fast_parallel_write_3c(u8g, u8g_pgm_read(ptr));
          ptr++;
          arg_val--;
        }
      }
      break;
    case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g_com_arduino_digital_write(u8g, U8G_PI_DI, arg_val);
      break;
    case U8G_COM_MSG_RESET:
      if ( u8g->pin_list[U8G_PI_RESET] != U8G_PIN_NONE )
        u8g_com_arduino_digital_write(u8g, U8G_PI_RESET, arg_val);
      break;
      
  }
  return 1;
}

#else
//For non Arduino devices
uint8_t u8g_com_arduino_fast_parallel_3chipsel_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  return 1;
}
#endif /* ARDUINO */

