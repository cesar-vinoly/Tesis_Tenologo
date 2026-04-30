#ifndef ILI9488_H
#define ILI9488_H

#include "main.h"

/* Resolución */
#define ILI9488_WIDTH   480
#define ILI9488_HEIGHT  320

/* Colores RGB565 */
#define ILI9488_COLOR_BLACK    0x0000
#define ILI9488_COLOR_WHITE    0xFFFF
#define ILI9488_COLOR_RED      0xF800
#define ILI9488_COLOR_GREEN    0x07E0
#define ILI9488_COLOR_BLUE     0x001F
#define ILI9488_COLOR_YELLOW   0xFFE0
#define ILI9488_COLOR_CYAN     0x07FF
#define ILI9488_COLOR_MAGENTA  0xF81F

/* Pines de control TFT
   Cámbialos aquí si luego decides usar otros */
#define ILI9488_CS_GPIO_Port    GPIOE
#define ILI9488_CS_Pin          GPIO_PIN_13

#define ILI9488_DC_GPIO_Port    GPIOE
#define ILI9488_DC_Pin          GPIO_PIN_12

#define ILI9488_RST_GPIO_Port   GPIOE
#define ILI9488_RST_Pin         GPIO_PIN_11

void ILI9488_Init(SPI_HandleTypeDef *hspi);
void ILI9488_FillScreen(uint16_t color);
void ILI9488_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9488_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale);
void ILI9488_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale);
void ILI9488_DrawTextWrapped(uint16_t x, uint16_t y, uint16_t max_width, const char *str, uint16_t color, uint16_t bg, uint8_t scale);

#endif
