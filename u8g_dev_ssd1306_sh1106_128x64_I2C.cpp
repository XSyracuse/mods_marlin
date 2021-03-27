/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * Based on u8g_dev_ssd1306_128x64.c
 *
 * Universal 8bit Graphics Library
 *
 * Copyright (c) 2015, olikraus@gmail.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list
 *    of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * These routines are meant for two wire I2C interfaces.
 *
 * Three and four wire I2C interfaces have an A0 line.  That line is
 * used to switch between command and data modes.
 *
 * The two wire LCDs use an instruction byte to signal if data or
 * command info is to follow.  The command stream needs the instruction
 * byte between eack command byte.  The data stream needs one at the
 * beginning.
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_U8GLIB

#include "HAL_LCD_com_defines.h"

#define WIDTH 128
#define HEIGHT 64
#define PAGE_HEIGHT 8

uint8_t u8g_WriteEscSeqP_2_wire(u8g_t *u8g, u8g_dev_t *dev, const uint8_t *esc_seq);

// The sh1106 is compatible to the ssd1306, but is 132x64. 128x64 display area is centered within
// the 132x64.

static const uint8_t u8g_dev_sh1106_128x64_data_start_2_wire[] PROGMEM = {
  0x010,          // set upper 4 bit of the col adr to 0
  0x002,          // set lower 4 bit of the col adr to 2 (centered display with ssd1306)
  U8G_ESC_END     // end of sequence
};

#define SH1106_PAGE_ADR(N)       (0x20), (N)
#define SH1106_COLUMN_RANGE(N)   (0x21), (((N) >> 8) & 0xFF), ((N) & 0xFF)
#define SH1106_PAGE_RANGE(N,O)   (0x22), (N), (O)
#define SH1106_SCROLL(N)         ((N) ? 0x2F : 0x2E)
#define SH1106_START_LINE(N)     (0x40 | (N))
#define SH1106_CONTRAST(N)       (0x81), (N)
#define SH1106_CHARGE_PUMP(N)    (0x8D), ((N) ? 0x14 : 0x10)
#define SH1106_ADC_REVERSE(N)    ((N) ? 0xA1 : 0xA0)
#define SH1106_ALL_PIX(N)        ((N) ? 0xA5 : 0xA4)
#define SH1106_INVERTED(N)       ((N) ? 0xA7 : 0xA6)
#define SH1106_MUX_RATIO(N)      (0xA8), (N)
#define SH1106_ON(N)             ((N) ? 0xAF : 0xAE)
#define SH1106_OUT_MODE(N)       ((N) ? 0xC8 : 0xC0)
#define SH1106_DISP_OFFS(N)      (0xD3), (N)
#define SH1106_OSC_FREQ(R,F)     (0xD5), ((F) << 4 | (R))
#define SH1106_CHARGE_PER(P,D)   (0xD9), ((D) << 4 | (P))
#define SH1106_COM_CONFIG(N)     (0xDA), ((N) ? 0x12 : 0x02)
#define SH1106_VCOM_DESEL(N)     (0xDB), (N)
#define SH1106_NOOP()            (0xE3)

static const uint8_t u8g_dev_sh1106_128x64_init_seq_2_wire[] PROGMEM = {
  U8G_ESC_ADR(0),               // Initiate command mode
  SH1106_ON(0),                 // Display off, sleep mode
  SH1106_MUX_RATIO(0x3F),       // Mux ratio
  SH1106_DISP_OFFS(0),          // Display offset
  SH1106_START_LINE(0),         // Start line
  SH1106_ADC_REVERSE(1),        // Segment remap A0/A1
  SH1106_OUT_MODE(1),           // C0: scan dir normal, C8: reverse
  SH1106_COM_CONFIG(1),         // Com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5)
  SH1106_CONTRAST(0xCF),        // [2] set contrast control
  SH1106_PAGE_ADR(0x02),        // 2012-05-27: page addressing mode
  SH1106_COLUMN_RANGE(0x281),   // Set column range from 0 through 131
  SH1106_PAGE_RANGE(0, 7),      // Set page range from 0 through 7
  SH1106_CHARGE_PER(0x1, 0xF),  // [2] pre-charge period 0x22/F1
  SH1106_VCOM_DESEL(0x40),      // Vcomh deselect level
  SH1106_ALL_PIX(0),            // Output ram to display
  SH1106_INVERTED(0),           // Normal display mode
  SH1106_OSC_FREQ(0, 8),        // Clock divide ratio (0:1) and oscillator frequency (8)
  SH1106_CHARGE_PUMP(1),        // [2] charge pump setting (P62): 0x14 enable, 0x10 disable
  SH1106_SCROLL(0),             // 2012-05-27: Deactivate scroll
  SH1106_ON(1),                 // Display on
  U8G_ESC_END                   // End of sequence
};

uint8_t u8g_dev_sh1106_128x64_2x_2_wire_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
  switch (msg) {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_300NS);
      u8g_WriteEscSeqP_2_wire(u8g, dev, u8g_dev_sh1106_128x64_init_seq_2_wire);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT: {
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
        u8g_SetAddress(u8g, dev, 0);           // instruction mode
        u8g_WriteEscSeqP_2_wire(u8g, dev, u8g_dev_sh1106_128x64_data_start_2_wire);
        u8g_WriteByte(u8g, dev, 0x0B0 | (pb->p.page*2)); // select current page
        u8g_SetAddress(u8g, dev, 1);           // data mode
        u8g_WriteSequence(u8g, dev, pb->width, (uint8_t *) pb->buf);
        u8g_SetChipSelect(u8g, dev, 0);
        u8g_SetAddress(u8g, dev, 0);           // instruction mode
        u8g_WriteEscSeqP_2_wire(u8g, dev, u8g_dev_sh1106_128x64_data_start_2_wire);
        u8g_WriteByte(u8g, dev, 0x0B0 | (pb->p.page*2+1)); // select current page
        u8g_SetAddress(u8g, dev, 1);           // data mode
        u8g_WriteSequence(u8g, dev, pb->width, (uint8_t *)(pb->buf)+pb->width);
        u8g_SetChipSelect(u8g, dev, 0);
      }
      break;
    case U8G_DEV_MSG_SLEEP_ON:
      return 1;
    case U8G_DEV_MSG_SLEEP_OFF:
      return 1;
  }
  return u8g_dev_pb16v1_base_fn(u8g, dev, msg, arg);
}

uint8_t u8g_dev_sh1106_128x64_2x_i2c_2_wire_buf[WIDTH*2] U8G_NOCOMMON ;
u8g_pb_t u8g_dev_sh1106_128x64_2x_i2c_2_wire_pb = { {16, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_sh1106_128x64_2x_i2c_2_wire_buf};
u8g_dev_t u8g_dev_sh1106_128x64_2x_i2c_2_wire = { u8g_dev_sh1106_128x64_2x_2_wire_fn, &u8g_dev_sh1106_128x64_2x_i2c_2_wire_pb, U8G_COM_SSD_I2C_HAL };

/////////////////////////////////////////////////////////////////////////////////////////////

static const uint8_t u8g_dev_ssd1306_128x64_data_start_2_wire[] PROGMEM = {
  0x010,          // set upper 4 bit of the col adr to 0
  0x000,          // set lower 4 bit of the col adr to 0
  U8G_ESC_END     // end of sequence
};

static const uint8_t u8g_dev_ssd1306_128x64_init_seq_2_wire[] PROGMEM = {
  U8G_ESC_ADR(0), // initiate command mode
  0x0AE,          // display off, sleep mode
  0x0A8, 0x03F,   // mux ratio
  0x0D3, 0x00,    // display offset
  0x040,          // start line
  0x0A1,          // segment remap a0/a1
  0x0C8,          // c0: scan dir normal, c8: reverse
  0x0DA, 0x012,   // com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5)
  0x081, 0x0CF,   // [2] set contrast control
  0x020, 0x002,   // 2012-05-27: page addressing mode
  0x21, 0, 0x7F,  // set column range from 0 through 127
  0x22, 0, 7,     // set page range from 0 through 7
  0x0D9, 0x0F1,   // [2] pre-charge period 0x022/f1
  0x0DB, 0x040,   // vcomh deselect level
  0x0A4,          // output ram to display
  0x0A6,          // none inverted normal display mode
  0x0D5, 0x080,   // clock divide ratio (0x00=1) and oscillator frequency (0x8)
  0x08D, 0x014,   // [2] charge pump setting (p62): 0x014 enable, 0x010 disable
  0x02E,          // 2012-05-27: Deactivate scroll
  0x0AF,          // display on
  U8G_ESC_END     // end of sequence
};

uint8_t u8g_dev_ssd1306_128x64_2x_2_wire_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg) {
  switch (msg) {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_300NS);
      u8g_WriteEscSeqP_2_wire(u8g, dev, u8g_dev_ssd1306_128x64_init_seq_2_wire);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT: {
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
        u8g_SetAddress(u8g, dev, 0);           // instruction mode
        u8g_WriteEscSeqP_2_wire(u8g, dev, u8g_dev_ssd1306_128x64_data_start_2_wire);
        u8g_WriteByte(u8g, dev, 0x0B0 | (pb->p.page*2)); // select current page
        u8g_SetAddress(u8g, dev, 1);           // data mode
        u8g_WriteSequence(u8g, dev, pb->width, (uint8_t *) pb->buf);
        u8g_SetChipSelect(u8g, dev, 0);
        u8g_SetAddress(u8g, dev, 0);           // instruction mode
        u8g_WriteEscSeqP_2_wire(u8g, dev, u8g_dev_ssd1306_128x64_data_start_2_wire);
        u8g_WriteByte(u8g, dev, 0x0B0 | (pb->p.page*2+1)); // select current page
        u8g_SetAddress(u8g, dev, 1);           // data mode
        u8g_WriteSequence(u8g, dev, pb->width, (uint8_t *)(pb->buf)+pb->width);
        u8g_SetChipSelect(u8g, dev, 0);
      }
      break;
    case U8G_DEV_MSG_SLEEP_ON:
      return 1;
    case U8G_DEV_MSG_SLEEP_OFF:
      return 1;
  }
  return u8g_dev_pb16v1_base_fn(u8g, dev, msg, arg);
}


uint8_t u8g_dev_ssd1306_128x64_2x_i2c_2_wire_buf[WIDTH*2] U8G_NOCOMMON ;
u8g_pb_t u8g_dev_ssd1306_128x64_2x_i2c_2_wire_pb = { {16, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_ssd1306_128x64_2x_i2c_2_wire_buf};
u8g_dev_t u8g_dev_ssd1306_128x64_2x_i2c_2_wire = { u8g_dev_ssd1306_128x64_2x_2_wire_fn, &u8g_dev_ssd1306_128x64_2x_i2c_2_wire_pb, U8G_COM_SSD_I2C_HAL };


/////////////////////////////////////////////////////////////////////////////////////////////

// This routine adds the instruction byte in between the command bytes.  This makes the init
// sequences a lot easier to read.

#define I2C_CMD_MODE    0x080

uint8_t u8g_WriteEscSeqP_2_wire(u8g_t *u8g, u8g_dev_t *dev, const uint8_t *esc_seq) {
  uint8_t is_escape = 0;
  for (;;) {
    uint8_t value = u8g_pgm_read(esc_seq);
    if (is_escape == 0) {
      if (value != 255) {
        if (u8g_WriteByte(u8g, dev, value) == 0 )
          return 0;
        if (u8g_WriteByte(u8g, dev, I2C_CMD_MODE) == 0 )
          return 0;
      }
      else {
        is_escape = 1;
      }
    }
    else {
      if (value == 255) {
        if (u8g_WriteByte(u8g, dev, value) == 0 )
          return 0;
        if (u8g_WriteByte(u8g, dev, I2C_CMD_MODE) == 0 )
          return 0;
      }
      else if (value == 254) {
        break;
      }
      else if (value >= 0x0F0) {
        /* not yet used, do nothing */
      }
      else if (value >= 0xE0 ) {
        u8g_SetAddress(u8g, dev, value & 0x0F);
      }
      else if (value >= 0xD0) {
        u8g_SetChipSelect(u8g, dev, value & 0x0F);
      }
      else if (value >= 0xC0) {
        u8g_SetResetLow(u8g, dev);
        value &= 0x0F;
        value <<= 4;
        value+=2;
        u8g_Delay(value);
        u8g_SetResetHigh(u8g, dev);
        u8g_Delay(value);
      }
      else if (value >= 0xBE) {                       /* not yet implemented */
        /* u8g_SetVCC(u8g, dev, value & 0x01); */
      }
      else if (value <= 127) {
        u8g_Delay(value);
      }
      is_escape = 0;
    }
    esc_seq++;
  }
  return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Added device specific code here, because could not get to link from a HAL in Arduino:)
//
// After some experimenting I found out that during configuration
// if I use I2C <address> <0x80> <setting>, then there must be a I2C stop after one byte.
// however, if use <addess><0x00><setting><setting> is possible.
// this is not implemented here, info for future reference. MDF
/////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <pgmspace.h>
#define SSD1306_ADDR7    (0x3c)
#define SSD1306_SLA_CMD  (0x00)
#define SSD1306_SLA_DATA (0x40)

static const uint8_t config_ssd1306[] PROGMEM = {
  0x0AE,          // display off, sleep mode
  0x0A8, 0x03F,   // mux ratio
  0x0D3, 0x00,    // display offset
  0x040,          // start line
  0x0A1,          // segment remap a0/a1
  0x0C8,          // c0: scan dir normal, c8: reverse
  0x0DA, 0x012,   // com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5)
  0x081, 0x0CF,   // [2] set contrast control
  0x020, 0x002,   // 2012-05-27: page addressing mode
  0x21, 0, 0x7F,  // set column range from 0 through 127
  0x22, 0, 7,     // set page range from 0 through 7
  0x0D9, 0x0F1,   // [2] pre-charge period 0x022/f1
  0x0DB, 0x040,   // vcomh deselect level
  0x0A4,          // output ram to display
  0x0A6,          // none inverted normal display mode
  0x0D5, 0x080,   // clock divide ratio (0x00=1) and oscillator frequency (0x8)
  0x08D, 0x014,   // [2] charge pump setting (p62): 0x014 enable, 0x010 disable
  0x02E,          // 2012-05-27: Deactivate scroll
  0x0AF,          // display on
  0xFF            // end of sequence
};
uint8_t u8g_com_ssd_I2C_start_sequence(u8g_t *u8g)
{
  /* are we requested to set the a0 state? */
  if ( u8g->pin_list[U8G_PI_SET_A0] == 0 )
    return 1;

  /*start address*/
  Wire.beginTransmission(SSD1306_ADDR7);

  if ( u8g->pin_list[U8G_PI_A0] == 0 )
  {
    if ( Wire.write(SSD1306_SLA_CMD) == 0 )
      return 0;
  }
  else
  {
    if ( Wire.write(SSD1306_SLA_DATA) == 0 )
      return 0;
  }

  u8g->pin_list[U8G_PI_SET_A0] = 0;
  return 1;
}

uint8_t u8g_com_HAL_ESP32_ssd_hw_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  uint8_t *ptr;
  switch(msg)
  {
    case U8G_COM_MSG_INIT:

      Wire.begin();
      Wire.setClock(400000);

      for(uint8_t i=0;i<32;i++){

        Wire.beginTransmission(SSD1306_ADDR7);
        Wire.write(0x80);
        Wire.write(pgm_read_byte(config_ssd1306+i));
        Wire.endTransmission();
      
      }
      break;
    
    case U8G_COM_MSG_STOP:
      break;

    case U8G_COM_MSG_RESET:
      /* Currently disabled, but it could be enable. Previous restrictions have been removed */
      break;
      
    case U8G_COM_MSG_CHIP_SELECT:
      /* Use address ONLY to set command byte value */
      
      //u8g->pin_list[U8G_PI_A0_STATE] = 0;
      //u8g->pin_list[U8G_PI_SET_A0] = 1;		/* force a0 to set again, also forces start condition */
      
      //if ( arg_val == 0 )
      //{
        /* disable chip, send stop condition */
	//Wire.endTransmission();
      //}
      //else
      //{
        /* enable, do nothing: any byte writing will trigger the i2c start */
      //}
      
      break;

    case U8G_COM_MSG_WRITE_BYTE:
      
      if ( u8g_com_ssd_I2C_start_sequence(u8g) == 0 )
      {
        Wire.endTransmission();
        return 1;
      }
      if ( Wire.write(arg_val) == 0 )
      {
        Wire.endTransmission();
        return 1;
      }
      Wire.endTransmission();
      break;
    
    case U8G_COM_MSG_WRITE_SEQ:
      //u8g->pin_list[U8G_PI_SET_A0] = 1;
      if (u8g_com_ssd_I2C_start_sequence(u8g) == 0) 
      {
        Wire.endTransmission();
        return 0;
      }

      ptr = (uint8_t *)arg_ptr;
      while (arg_val > 0) {
        if (Wire.write(*ptr++) == 0) 
        {
          Wire.endTransmission();
          return 0;
        }
        arg_val--;
      }
      
      Wire.endTransmission();
      break;

    case U8G_COM_MSG_WRITE_SEQ_P:
      //u8g->pin_list[U8G_PI_SET_A0] = 1;
      if (u8g_com_ssd_I2C_start_sequence(u8g) == 0) 
      {
        Wire.endTransmission();
        return 0;
      }

      ptr = (uint8_t *)arg_ptr;
      while (arg_val > 0) {
        if (Wire.write(u8g_pgm_read(ptr)) == 0)
        {
          Wire.endTransmission();
          return 0;
        }
        ptr++;
        arg_val--;
        
      }
      Wire.endTransmission();
      break;
      
    case U8G_COM_MSG_ADDRESS:                     /* define cmd (arg_val = 0) or data mode (arg_val = 1) */
      u8g->pin_list[U8G_PI_A0_STATE] = arg_val;
      u8g->pin_list[U8G_PI_SET_A0] = 1;		/* force a0 to set again */
  
      break;
    default:
      break;
  }//switch

  return 1;
}


#endif // HAS_MARLINUI_U8GLIB
