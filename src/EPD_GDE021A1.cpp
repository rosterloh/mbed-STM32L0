/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "EPD_GDE021A1.h"

/**
  * @brief  GDE021A1 Size
  */
#define  GDE021A1_EPD_PIXEL_WIDTH    ((uint16_t)172)
#define  GDE021A1_EPD_PIXEL_HEIGHT   ((uint16_t)18)

/**
  * @brief  GDE021A1 Registers
  */
#define EPD_REG_0             0x00   /* Status Read */
#define EPD_REG_1             0x01   /* Driver Output Control */
#define EPD_REG_3             0x03   /* Gate driving voltage control */
#define EPD_REG_4             0x04   /* Source driving coltage control */
#define EPD_REG_7             0x07   /* Display Control */
#define EPD_REG_11            0x0B   /* Gate and Sorce non overlap period COntrol */
#define EPD_REG_15            0x0F   /* Gate scan start */
#define EPD_REG_16            0x10   /* Deep Sleep mode setting */
#define EPD_REG_17            0x11   /* Data Entry Mode Setting */
#define EPD_REG_18            0x12   /* SWRESET */
#define EPD_REG_26            0x1A   /* Temperature Sensor Control (Write to Temp Register) */
#define EPD_REG_27            0x1B   /* Temperature Sensor Control(Read from Temp Register) */
#define EPD_REG_28            0x1C   /* Temperature Sensor Control(Write Command  to Temp sensor) */
#define EPD_REG_29            0x1D   /* Temperature Sensor Control(Load temperature register with temperature sensor reading) */
#define EPD_REG_32            0x20   /* Master activation */
#define EPD_REG_33            0x21   /* Display update */
#define EPD_REG_34            0x22   /* Display update control 2 */
#define EPD_REG_36            0x24   /* write RAM */
#define EPD_REG_37            0x25   /* Read RAM */
#define EPD_REG_40            0x28   /* VCOM sense */
#define EPD_REG_41            0x29   /* VCOM Sense duration */
#define EPD_REG_42            0x2A   /* VCOM OTP program */
#define EPD_REG_44            0x2C   /* Write VCOMregister */
#define EPD_REG_45            0x2D   /* Read OTP registers */
#define EPD_REG_48            0x30   /* Program WS OTP */
#define EPD_REG_50            0x32   /* Write LUT register */
#define EPD_REG_51            0x33   /* Read LUT register */
#define EPD_REG_54            0x36   /* Program OTP selection */
#define EPD_REG_55            0x37   /* Proceed OTP selection */
#define EPD_REG_58            0x3A   /* Set dummy line pulse period */
#define EPD_REG_59            0x3B   /* Set Gate line width */
#define EPD_REG_60            0x3C   /* Select Border waveform */
#define EPD_REG_68            0x44   /* Set RAM X - Address Start / End Position */
#define EPD_REG_69            0x45   /* Set RAM Y - Address Start / End Position */
#define EPD_REG_78            0x4E   /* Set RAM X Address Counter */
#define EPD_REG_79            0x4F   /* Set RAM Y Address Counter */
#define EPD_REG_240           0xF0   /* Booster Set Internal Feedback Selection */
#define EPD_REG_255           0xFF   /* NOP */

/* Look-up table for the epaper (90 bytes) */
const unsigned char WF_LUT[]={
  0x82,0x00,0x00,0x00,0xAA,0x00,0x00,0x00,
  0xAA,0xAA,0x00,0x00,0xAA,0xAA,0xAA,0x00,
  0x55,0xAA,0xAA,0x00,0x55,0x55,0x55,0x55,
  0xAA,0xAA,0xAA,0xAA,0x55,0x55,0x55,0x55,
  0xAA,0xAA,0xAA,0xAA,0x15,0x15,0x15,0x15,
  0x05,0x05,0x05,0x05,0x01,0x01,0x01,0x01,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x41,0x45,0xF1,0xFF,0x5F,0x55,0x01,0x00,
  0x00,0x00,};

// Constructor
EPD_GDE021A1::EPD_GDE021A1(PinName cs, PinName dc, PinName rst, PinName bsy, PinName pwr, PinName spi_mosi, PinName spi_miso, PinName spi_sck) :
  _cs(cs, 1),
  _dc(dc, 1),
  _rst(rst, 1),
  _bsy(bsy, PinMode::PullDown),
  _pwr(pwr, 1),
  _spi(spi_mosi, spi_miso, spi_sck)
{
  _pwr = 0;
  _cs = 0;
  _cs = 1;
  _rst = 1;

  wait_ms(10);

  _spi.format(8);
  _spi.frequency(1000000);

  gde021a1_Init();

  Clear(EPD_COLOR_WHITE);

  SetFont(&Font12);
}

// Destructor
EPD_GDE021A1::~EPD_GDE021A1() { }

//=================================================================================================================
// Public methods
//=================================================================================================================

uint32_t EPD_GDE021A1::GetXSize(void)
{
  return(gde021a1_GetEpdPixelWidth());
}

uint32_t EPD_GDE021A1::GetYSize(void)
{
  return(gde021a1_GetEpdPixelHeight());
}

void EPD_GDE021A1::SetFont(sFONT *pFonts)
{
  pFont = pFonts;
}

sFONT *EPD_GDE021A1::GetFont(void)
{
  return pFont;
}

void EPD_GDE021A1::Clear(uint16_t Color)
{
  uint32_t index = 0;

  gde021a1_SetDisplayWindow(0, 0, 171, 17);

  for(index = 0; index < 3096; index++)
  {
      gde021a1_WritePixel(Color);
  }
}

void EPD_GDE021A1::DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii)
{
  Ascii -= 32;

  DrawChar(Xpos, Ypos, &pFont->table[Ascii * ((pFont->Height) * (pFont->Width))]);
}

void EPD_GDE021A1::DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode)
{
  uint16_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t  *ptr = Text;

  /* Get the text size */
  while (*ptr++) size ++ ;

  /* Characters number per line */
  xsize = (GetXSize()/pFont->Width);

  switch (Mode)
  {
  case CENTER_MODE:
    {
      refcolumn = Xpos + ((xsize - size)* pFont->Width) / 2;
      break;
    }
  case LEFT_MODE:
    {
      refcolumn = Xpos;
      break;
    }
  case RIGHT_MODE:
    {
      refcolumn =  - Xpos + ((xsize - size)*pFont->Width);
      break;
    }
  default:
    {
      refcolumn = Xpos;
      break;
    }
  }

  /* Send the string character by character on EPD */
  while ((*Text != 0) & (((GetXSize() - (i*pFont->Width)) & 0xFFFF) >= pFont->Width))
  {
    /* Display one character on EPD */
    DisplayChar(refcolumn, Ypos, *Text);
    /* Decrement the column position by 16 */
    refcolumn += pFont->Width;
    /* Point on the next character */
    Text++;
    i++;
  }
}

void EPD_GDE021A1::DisplayStringAtLine(uint16_t Line, uint8_t *ptr, Text_AlignModeTypdef Mode)
{
  DisplayStringAt(0, LINE(Line), ptr, Mode);
}

void EPD_GDE021A1::DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t index = 0;

  gde021a1_SetDisplayWindow(Xpos, Ypos, Xpos + Length, Ypos);

  for(index = 0; index < Length; index++)
  {
    /* Prepare the register to write data on the RAM */
    gde021a1_WritePixel(0x3F);
  }
}

void EPD_GDE021A1::DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint32_t index = 0;

  gde021a1_SetDisplayWindow(Xpos, Ypos, Xpos, Ypos + Length);

  for(index = 0; index < Length; index++)
  {
    /* Prepare the register to write data on the RAM */
    gde021a1_WritePixel(0x00);
  }
}

void EPD_GDE021A1::DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Draw horizontal lines */
  DrawHLine(Xpos, Ypos, Width);
  DrawHLine(Xpos, (Ypos + Height), (Width + 1));

  /* Draw vertical lines */
  DrawVLine(Xpos, Ypos, Height);
  DrawVLine((Xpos + Width), Ypos , Height);
}

void EPD_GDE021A1::FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint16_t index = 0;

  /* Set the rectangle */
  gde021a1_SetDisplayWindow(Xpos, Ypos, (Xpos + Width), (Ypos + Height));

  for(index = 0; index < 3096; index++)
  {
    gde021a1_WritePixel(0xFF);
  }
}

void EPD_GDE021A1::DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{
  /* Set display window */
  gde021a1_SetDisplayWindow(Xpos, Ypos, (Xpos+Ysize-1), (Ypos+(Xsize/4)-1));

  gde021a1_DrawImage(Xpos, Ypos, Xsize, Ysize, pdata);

  gde021a1_SetDisplayWindow(0, 0, GetXSize(), GetYSize());
}

void EPD_GDE021A1::CloseChargePump(void)
{
  /* Close charge pump */
  gde021a1_CloseChargePump();

  wait_ms(400);
}

/**
  * @brief  Updates the display from the data located into the RAM.
  * @param  None
  * @retval None
  */
void EPD_GDE021A1::RefreshDisplay(void)
{
  gde021a1_RefreshDisplay();

  /* Poll on the BUSY signal and wait for the EPD to be ready */
  while (_bsy != 0);

  _rst = 1;

  wait_ms(10);
}

//=================================================================================================================
// Private methods
//=================================================================================================================

void EPD_GDE021A1::EPD_IO_WriteData(uint16_t RegValue)
{
  _cs = 0;
  _dc = 1;
  _spi.write(RegValue);
  _cs = 1;
}

void EPD_GDE021A1::EPD_IO_WriteReg(uint8_t Reg)
{
  _cs = 0;
  _dc = 0;
  _spi.write(Reg);
  _cs = 1;
}

uint16_t EPD_GDE021A1::EPD_IO_ReadData(void)
{
  _cs = 0;
  _cs = 1;
  return _spi.write(0xFF);
}

//=================================================================================================================

/**
  * @brief  Initialize the GDE021A1 EPD Component.
  * @param  None
  * @retval None
  */
void EPD_GDE021A1::gde021a1_Init(void)
{
  uint8_t nb_bytes = 0;

  EPD_IO_WriteReg(EPD_REG_16);  /* Deep sleep mode disable */
  EPD_IO_WriteData(0x00);
  EPD_IO_WriteReg(EPD_REG_17);  /* Data Entry Mode Setting */
  EPD_IO_WriteData(0x03);
  EPD_IO_WriteReg(EPD_REG_68);  /* Set the RAM X start/end address */
  EPD_IO_WriteData(0x00);       /* RAM X address start = 00h */
  EPD_IO_WriteData(0x11);       /* RAM X adress end = 11h (17 * 4pixels by address = 72 pixels) */
  EPD_IO_WriteReg(EPD_REG_69);  /* Set the RAM Y start/end address */
  EPD_IO_WriteData(0x00);       /* RAM Y address start = 0 */
  EPD_IO_WriteData(0xAB);       /* RAM Y adress end = 171 */
  EPD_IO_WriteReg(EPD_REG_78);  /* Set RAM X Address counter */
  EPD_IO_WriteData(0x00);
  EPD_IO_WriteReg(EPD_REG_79);  /* Set RAM Y Address counter */
  EPD_IO_WriteData(0x00);
  EPD_IO_WriteReg(EPD_REG_240); /* Booster Set Internal Feedback Selection */
  EPD_IO_WriteData(0x1F);
  EPD_IO_WriteReg(EPD_REG_33);  /* Disable RAM bypass and set GS transition to GSA = GS0 and GSB = GS3 */
  EPD_IO_WriteData(0x03);
  EPD_IO_WriteReg(EPD_REG_44);  /* Write VCOMregister */
  EPD_IO_WriteData(0xA0);
  EPD_IO_WriteReg(EPD_REG_60);  /* Border waveform */
  EPD_IO_WriteData(0x64);
  EPD_IO_WriteReg(EPD_REG_50);  /* Write LUT register */

  for (nb_bytes=0; nb_bytes<90; nb_bytes++)
  {
    EPD_IO_WriteData(WF_LUT[nb_bytes]);
  }
}

/**
  * @brief  Writes to the selected EPD register.
  * @param  EPD_Reg: Address of the selected register.
  * @param  EPD_RegValue: value to write to the selected register.
  * @retval None
  */
void EPD_GDE021A1::gde021a1_WriteReg(uint8_t EPD_Reg, uint8_t EPD_RegValue)
{
  EPD_IO_WriteReg(EPD_Reg);

  EPD_IO_WriteData(EPD_RegValue);
}

/**
  * @brief  Reads the selected EPD Register.
  * @param  EPD_Reg: address of the selected register
  * @retval EPD Register Value
  */
uint8_t EPD_GDE021A1::gde021a1_ReadReg(uint8_t EPD_Reg)
{
  /* Write 8-bit Index (then Read Reg) */
  EPD_IO_WriteReg(EPD_Reg);

  /* Read 8-bit Reg */
  return (EPD_IO_ReadData());
}

/**
  * @brief  Writes 4 dots.
  * @param  HEX_Code: specifies the Data to write.
  * @retval None
  */
void EPD_GDE021A1::gde021a1_WritePixel(uint8_t HEX_Code)
{
  /* Prepare the register to write data on the RAM */
  EPD_IO_WriteReg(EPD_REG_36);

  /* Send the data to write */
  EPD_IO_WriteData(HEX_Code);
}

/**
  * @brief  Displays picture..
  * @param  pdata: picture address.
  * @param  Xpos:  Image X position in the EPD
  * @param  Ypos:  Image Y position in the EPD
  * @param  Xsize: Image X size in the EPD
  * @note   Xsize have to be a multiple of 4
  * @param  Ysize: Image Y size in the EPD
  * @retval None
  */
void EPD_GDE021A1::gde021a1_DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{
  uint32_t i, j = 0;
  uint8_t pixels_4 = 0;
  uint8_t pixels_4_grey[4] = {0};
  uint8_t nb_4_pixels, data_res = 0;

  /* Prepare the register to write data on the RAM */
  EPD_IO_WriteReg(EPD_REG_36);

  /* X size is a multiple of 8 */
  if ((Xsize % 8) == 0)
  {
    for (i= 0; i< ((((Ysize) * (Xsize/4)))/2) ; i++)
    {
      /* Get the current data */
      pixels_4 = pdata[i];
      if (pixels_4 !=0)
      {
        /* One byte read codes 8 pixels in 1-bit bitmap */
        for (nb_4_pixels = 0; nb_4_pixels < 2; nb_4_pixels++)
        {
          /* Processing 8 pixels */
          /* Preparing the 4 pixels coded with 4 grey level per pixel
             from a monochrome xbm file */
          for (j= 0; j<4; j++)
          {
            if (((pixels_4) & 0x01) == 1)
            {
              /* Two LSB is coding black in 4 grey level */
              pixels_4_grey[j] &= 0xFC;
            }
            else
            {
              /* Two LSB is coded white in 4 grey level */
              pixels_4_grey[j] |= 0x03;
            }
            pixels_4 = pixels_4 >> 1;
          }

          /* Processing 4 pixels */
          /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
             EPD topology */
          data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;

          /* Send the data to the EPD's RAM through SPI */
          EPD_IO_WriteData(data_res);
        }
      }
      else
      {
        /* 1 byte read from xbm files is equivalent to 8 pixels in the
           other words 2 bytes to be transferred */
        EPD_IO_WriteData(0xFF);
        EPD_IO_WriteData(0xFF);
      }
    }
  }

  /* X size is a multiple of 4 */
  else
  {
    for (i= 0; i< ((((Ysize) * ((Xsize/4)+1))/2)) ; i++)
    {
      /* Get the current data */
      pixels_4 = pdata[i];
      if (((i+1) % (((Xsize/4)+1)/2)) != 0)
      {
        if (pixels_4 !=0)
        {
          /* One byte read codes 8 pixels in 1-bit bitmap */
          for (nb_4_pixels = 0; nb_4_pixels < 2; nb_4_pixels++)
          {
            /* Processing 8 pixels */
            /* Preparing the 4 pixels coded with 4 grey level per pixel
               from a monochrome xbm file */
            for (j= 0; j<4; j++)
            {
              if (((pixels_4) & 0x01) == 1)
              {
                /* Two LSB is coding black in 4 grey level */
                pixels_4_grey[j] &= 0xFC;
              }
              else
              {
                /* Two LSB is coded white in 4 grey level */
                pixels_4_grey[j] |= 0x03;
              }
              pixels_4 = pixels_4 >> 1;
            }

            /* Processing 4 pixels */
            /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
               EPD topology */
            data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;

            /* Send the data to the EPD's RAM through SPI */
            EPD_IO_WriteData(data_res);
          }
        }
        else if (pixels_4 == 0)
        {
          /* One byte read from xbm files is equivalent to 8 pixels in the
             other words Two bytes to be transferred */
          EPD_IO_WriteData(0xFF);
          EPD_IO_WriteData(0xFF);
        }
      }

      else if (((i+1) % (((Xsize/4)+1)/2)) == 0)
      {
        if (pixels_4 !=0xf0)
        {
          /* Processing 8 pixels */
          /* Preparing the 4 pixels coded with 4 grey level per pixel
             from a monochrome xbm file */
          for (j= 0; j<4; j++)
          {
            if (((pixels_4) & 0x01) == 1)
            {
              /* 2 LSB is coding black in 4 grey level */
              pixels_4_grey[j] &= 0xFC;
            }
            else
            {
              /* 2 LSB is coded white in 4 grey level */
              pixels_4_grey[j] |= 0x03;
            }
            pixels_4 = pixels_4 >> 1;
          }

          /* Processing 4 pixels */
          /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
             EPD topology */
          data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;

          /* Send the data to the EPD's RAM through SPI */
          EPD_IO_WriteData(data_res);
        }
        else if (pixels_4 == 0xf0)
        {
          /* One byte to be transferred */
          EPD_IO_WriteData(0xFF);
        }
      }
    }
  }
}

/**
  * @brief  Activates display update sequence.
  * @param  None
  * @retval None
  */
void EPD_GDE021A1::gde021a1_RefreshDisplay(void)
{
  /* Write on the Display update control register */
  EPD_IO_WriteReg(EPD_REG_34);

  /* Display update data sequence option */
  EPD_IO_WriteData(0xC4);

  /* Launching the update: Nothing should interrupt this sequence in order
     to avoid display corruption */
  EPD_IO_WriteReg(EPD_REG_32);
}

/**
  * @brief  Disables the clock and the charge pump.
  * @param  None
  * @retval None
  */
void EPD_GDE021A1::gde021a1_CloseChargePump(void)
{
  /* Write on the Display update control register */
  EPD_IO_WriteReg(EPD_REG_34);

  /* Disable CP then Disable Clock signal */
  EPD_IO_WriteData(0x03);

  /* Launching the update: Nothing should interrupt this sequence in order
     to avoid display corruption */
  EPD_IO_WriteReg(EPD_REG_32);
}

/**
  * @brief  Sets a display window.
  * @param  Xpos: specifies the X bottom left position.
  * @param  Ypos: specifies the Y bottom left position.
  * @param  Width: display window width.
  * @param  Height: display window height.
  * @retval None
*/
void EPD_GDE021A1::gde021a1_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Set Y position and the height */
  EPD_IO_WriteReg(EPD_REG_68);
  EPD_IO_WriteData(Ypos);
  EPD_IO_WriteData(Height);
  /* Set X position and the width */
  EPD_IO_WriteReg(EPD_REG_69);
  EPD_IO_WriteData(Xpos);
  EPD_IO_WriteData(Width);
  /* Set the height counter */
  EPD_IO_WriteReg(EPD_REG_78);
  EPD_IO_WriteData(Ypos);
  /* Set the width counter */
  EPD_IO_WriteReg(EPD_REG_79);
  EPD_IO_WriteData(Xpos);
}

/**
  * @brief  Gets the EPD pixel Width.
  * @param  None
  * @retval The EPD Pixel Width
  */
uint16_t EPD_GDE021A1::gde021a1_GetEpdPixelWidth(void)
{
  return GDE021A1_EPD_PIXEL_WIDTH;
}

/**
  * @brief  Gets the EPD pixel Height.
  * @param  None
  * @retval The EPD Pixel Height
  */
uint16_t EPD_GDE021A1::gde021a1_GetEpdPixelHeight(void)
{
  return GDE021A1_EPD_PIXEL_HEIGHT;
}

/**
  * @brief  Draws a character on EPD.
  * @param  Xpos: specifies the X position, can be a value from 0 to 171
  * @param  Ypos: specifies the Y position, can be a value from 0 to 17
  * @param  c: pointer to the character data
  * @retval None
  */
void EPD_GDE021A1::DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c)
{
  uint32_t index = 0;
  uint32_t data_length = 0;
  uint16_t height = 0;
  uint16_t width = 0;

  width  = pFont->Width;
  height = pFont->Height;

  /* Set the Character display window */
  gde021a1_SetDisplayWindow(Xpos, Ypos, (Xpos + width - 1), (Ypos + height - 1));

  data_length = (height * width);

  for(index = 0; index < data_length; index++)
  {
    gde021a1_WritePixel(c[index]);
  }
}
