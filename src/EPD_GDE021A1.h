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

#ifndef __EPD_GDE021A1_H
#define __EPD_GDE021A1_H

#include "mbed.h"
#include "fontsepd.h"

/**
  * @brief  EPD color
  */
#define EPD_COLOR_BLACK         0x00
#define EPD_COLOR_DARKGRAY      0x55
#define EPD_COLOR_LIGHTGRAY     0xAA
#define EPD_COLOR_WHITE         0xFF

/**
  * @brief  Line mode structures definition
  */
typedef enum
{
  CENTER_MODE = 0x01,
  RIGHT_MODE  = 0x02,
  LEFT_MODE   = 0x03
} Text_AlignModeTypdef;

/**
 * ePaperDisplay on SPI
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "EPD_GDE021A1.h"
 *
 * #define EPD_CS       PA_15
 * #define EPD_DC       PB_11
 * #define EPD_RESET    PB_2
 * #define EPD_BUSY     PA_8
 * #define EPD_POWER    PB_10
 * #define EPD_SPI_MOSI PB_5
 * #define EPD_SPI_MISO PB_4
 * #define EPD_SPI_SCK  PB_3
 *
 * EPD_GDE021A1 epd(EPD_CS, EPD_DC, EPD_RESET, EPD_BUSY, EPD_POWER, EPD_SPI_MOSI, EPD_SPI_MISO, EPD_SPI_SCK);
 *
 * int main()
 * {
 *   epd.Clear(EPD_COLOR_WHITE);
 *   epd.DisplayStringAtLine(5, (uint8_t*)"MBED", CENTER_MODE);
 *   epd.DisplayStringAtLine(3, (uint8_t*)"Epaper display", LEFT_MODE);
 *   epd.DisplayStringAtLine(2, (uint8_t*)"demo", LEFT_MODE);
 *   epd.RefreshDisplay();
 *   wait(2);
 *
 *   while(1) {
 *     led1 = !led1;
 *     wait(1);
 *   }
 * }
 * @endcode
 */
class EPD_GDE021A1
{
public:
   /**
    * Constructor
    * @param cs   EPD CS pin
    * @param dc   EPD DC pin
    * @param rst  EPD RESET pin
    * @param bsy  EPD BUSY pin
    * @param pwr  EPD POWER pin
    * @param mosi SPI MOSI pin
    * @param miso SPI MISO pin
    * @param scl  SPI SCLK pin
    */
    EPD_GDE021A1(PinName cs, PinName dc, PinName rst, PinName bsy, PinName pwr, PinName spi_mosi, PinName spi_miso, PinName spi_scl);

   /**
    * Destructor
    */
    ~EPD_GDE021A1();

    /**
      * @brief  Gets the EPD X size.
      * @param  None
      * @retval EPD X size
      */
    uint32_t GetXSize(void);

    /**
      * @brief  Gets the EPD Y size.
      * @param  None
      * @retval EPD Y size
      */
    uint32_t GetYSize(void);

   /**
    * @brief  Sets the Text Font.
    * @param  pFonts: specifies the layer font to be used.
    * @retval None
    */
    void SetFont(sFONT *pFonts);

    /**
      * @brief  Gets the Text Font.
      * @param  None.
      * @retval the used layer font.
      */
    sFONT *GetFont(void);

   /**
    * @brief  Clears the EPD.
    * @param  Color: Color of the background
    * @retval None
    */
    void Clear(uint16_t Color);

    /**
      * @brief  Displays one character.
      * @param  Xpos: start column address.
      * @param  Ypos: the Line where to display the character shape.
      * @param  Ascii: character ascii code, must be between 0x20 and 0x7E.
      * @retval None
      */
    void DisplayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii);

    /**
      * @brief  Displays characters on the EPD.
      * @param  Xpos: X position
      * @param  Ypos: Y position
      * @param  Text: Pointer to string to display on EPD
      * @param  Mode: Display mode
      *          This parameter can be one of the following values:
      *            @arg  CENTER_MODE
      *            @arg  RIGHT_MODE
      *            @arg  LEFT_MODE
      * @retval None
      */
    void DisplayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode);

    /**
      * @brief  Displays a character on the EPD.
      * @param  Line: Line where to display the character shape
      *          This parameter can be one of the following values:
      *            @arg  0..8: if the Current fonts is Font8
      *            @arg  0..5: if the Current fonts is Font12
      *            @arg  0..3: if the Current fonts is Font16
      *            @arg  0..2: if the Current fonts is Font20
      * @param  ptr: Pointer to string to display on EPD
      * @retval None
      */
    void DisplayStringAtLine(uint16_t Line, uint8_t *ptr, Text_AlignModeTypdef Mode);

    /**
      * @brief  Draws an horizontal line.
      * @param  Xpos: X position
      * @param  Ypos: Y position
      * @param  Length: line length
      * @retval None
      */
    void DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);

    /**
      * @brief  Draws a vertical line.
      * @param  Xpos: X position
      * @param  Ypos: Y position
      * @param  Length: line length.
      * @retval None
      */
    void DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);

    /**
      * @brief  Draws a rectangle.
      * @param  Xpos: X position
      * @param  Ypos: Y position
      * @param  Height: rectangle height
      * @param  Width: rectangle width
      * @retval None
      */
    void DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);

    /**
      * @brief  Displays a full rectangle.
      * @param  Xpos: X position.
      * @param  Ypos: Y position.
      * @param  Height: display rectangle height.
      * @param  Width: display rectangle width.
      * @retval None
      */
    void FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);

    /**
      * @brief  Draws an Image.
      * @param  Xpos: X position in the EPD
      * @param  Ypos: Y position in the EPD
      * @param  Xsize: X size in the EPD
      * @param  Ysize: Y size in the EPD
      * @param  pdata: Pointer to the Image address
      * @retval None
      */
    void DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata);

    /**
      * @brief  Disables the clock and the charge pump.
      * @param  None
      * @retval None
      */
    void CloseChargePump(void);

    /**
      * @brief  Updates the display from the data located into the RAM.
      * @param  None
      * @retval None
      */
    void RefreshDisplay(void);

private:
    SPI _spi;
    DigitalOut _cs;
    DigitalOut _dc;
    DigitalOut _rst;
    DigitalIn _bsy;
    DigitalOut _pwr;

    sFONT *pFont;

    void EPD_IO_WriteData(uint16_t RegValue);
    void EPD_IO_WriteReg(uint8_t Reg);
    uint16_t EPD_IO_ReadData(void);

    void gde021a1_Init(void);
    void gde021a1_WriteReg(uint8_t EPD_Reg, uint8_t EPD_RegValue);
    uint8_t gde021a1_ReadReg(uint8_t EPD_Reg);
    void gde021a1_WritePixel(uint8_t HEX_Code);
    void gde021a1_DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata);
    void gde021a1_RefreshDisplay(void);
    void gde021a1_CloseChargePump(void);
    void gde021a1_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
    uint16_t gde021a1_GetEpdPixelWidth(void);
    uint16_t gde021a1_GetEpdPixelHeight(void);

    void DrawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);
};

#endif
