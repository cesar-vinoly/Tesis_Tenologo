#include "ili9488.h"
#include <string.h>

/* Comandos ILI9488 */
#define ILI9488_SWRESET     0x01
#define ILI9488_SLPOUT      0x11
#define ILI9488_DISPON      0x29
#define ILI9488_CASET       0x2A
#define ILI9488_PASET       0x2B
#define ILI9488_RAMWR       0x2C
#define ILI9488_MADCTL      0x36
#define ILI9488_PIXFMT      0x3A

static SPI_HandleTypeDef *s_hspi = NULL;

/* -------------------- Fuente 5x7 -------------------- */
/* Cada fila usa 5 bits. Bit 4 = pixel izquierdo */

static const uint8_t g_space[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static const uint8_t g_qmark[7] = {0x0E,0x11,0x01,0x02,0x04,0x00,0x04};
static const uint8_t g_exclam[7]= {0x04,0x04,0x04,0x04,0x04,0x00,0x04};
static const uint8_t g_dot[7]   = {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C};
static const uint8_t g_comma[7] = {0x00,0x00,0x00,0x00,0x00,0x0C,0x08};
static const uint8_t g_colon[7] = {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00};
static const uint8_t g_dash[7]  = {0x00,0x00,0x00,0x1F,0x00,0x00,0x00};
static const uint8_t g_under[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x1F};
static const uint8_t g_slash[7] = {0x01,0x02,0x04,0x08,0x10,0x00,0x00};
static const uint8_t g_plus[7]  = {0x00,0x04,0x04,0x1F,0x04,0x04,0x00};
static const uint8_t g_equal[7] = {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00};
static const uint8_t g_lpar[7]  = {0x02,0x04,0x08,0x08,0x08,0x04,0x02};
static const uint8_t g_rpar[7]  = {0x08,0x04,0x02,0x02,0x02,0x04,0x08};

static const uint8_t g_0[7] = {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E};
static const uint8_t g_1[7] = {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E};
static const uint8_t g_2[7] = {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F};
static const uint8_t g_3[7] = {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E};
static const uint8_t g_4[7] = {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02};
static const uint8_t g_5[7] = {0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E};
static const uint8_t g_6[7] = {0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E};
static const uint8_t g_7[7] = {0x1F,0x01,0x02,0x04,0x08,0x08,0x08};
static const uint8_t g_8[7] = {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E};
static const uint8_t g_9[7] = {0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E};

static const uint8_t g_A[7] = {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11};
static const uint8_t g_B[7] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E};
static const uint8_t g_C[7] = {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E};
static const uint8_t g_D[7] = {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E};
static const uint8_t g_E[7] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F};
static const uint8_t g_F[7] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10};
static const uint8_t g_G[7] = {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E};
static const uint8_t g_H[7] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11};
static const uint8_t g_I[7] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x1F};
static const uint8_t g_J[7] = {0x01,0x01,0x01,0x01,0x11,0x11,0x0E};
static const uint8_t g_K[7] = {0x11,0x12,0x14,0x18,0x14,0x12,0x11};
static const uint8_t g_L[7] = {0x10,0x10,0x10,0x10,0x10,0x10,0x1F};
static const uint8_t g_M[7] = {0x11,0x1B,0x15,0x15,0x11,0x11,0x11};
static const uint8_t g_N[7] = {0x11,0x11,0x19,0x15,0x13,0x11,0x11};
static const uint8_t g_O[7] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E};
static const uint8_t g_P[7] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10};
static const uint8_t g_Q[7] = {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D};
static const uint8_t g_R[7] = {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11};
static const uint8_t g_S[7] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E};
static const uint8_t g_T[7] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04};
static const uint8_t g_U[7] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E};
static const uint8_t g_V[7] = {0x11,0x11,0x11,0x11,0x11,0x0A,0x04};
static const uint8_t g_W[7] = {0x11,0x11,0x11,0x15,0x15,0x15,0x0A};
static const uint8_t g_X[7] = {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11};
static const uint8_t g_Y[7] = {0x11,0x11,0x0A,0x04,0x04,0x04,0x04};
static const uint8_t g_Z[7] = {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F};

static const uint8_t g_a[7] = {0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F};
static const uint8_t g_b[7] = {0x10,0x10,0x16,0x19,0x11,0x11,0x1E};
static const uint8_t g_c[7] = {0x00,0x00,0x0E,0x10,0x10,0x10,0x0E};
static const uint8_t g_d[7] = {0x01,0x01,0x0D,0x13,0x11,0x11,0x0F};
static const uint8_t g_e[7] = {0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E};
static const uint8_t g_f[7] = {0x06,0x09,0x08,0x1C,0x08,0x08,0x08};
static const uint8_t g_g[7] = {0x00,0x0F,0x11,0x11,0x0F,0x01,0x0E};
static const uint8_t g_h[7] = {0x10,0x10,0x16,0x19,0x11,0x11,0x11};
static const uint8_t g_i[7] = {0x04,0x00,0x0C,0x04,0x04,0x04,0x0E};
static const uint8_t g_j[7] = {0x02,0x00,0x06,0x02,0x02,0x12,0x0C};
static const uint8_t g_k[7] = {0x10,0x10,0x12,0x14,0x18,0x14,0x12};
static const uint8_t g_l[7] = {0x0C,0x04,0x04,0x04,0x04,0x04,0x0E};
static const uint8_t g_m[7] = {0x00,0x00,0x1A,0x15,0x15,0x15,0x15};
static const uint8_t g_n[7] = {0x00,0x00,0x16,0x19,0x11,0x11,0x11};
static const uint8_t g_o[7] = {0x00,0x00,0x0E,0x11,0x11,0x11,0x0E};
static const uint8_t g_p[7] = {0x00,0x1E,0x11,0x11,0x1E,0x10,0x10};
static const uint8_t g_q[7] = {0x00,0x0F,0x11,0x11,0x0F,0x01,0x01};
static const uint8_t g_r[7] = {0x00,0x00,0x16,0x19,0x10,0x10,0x10};
static const uint8_t g_s[7] = {0x00,0x00,0x0F,0x10,0x0E,0x01,0x1E};
static const uint8_t g_t[7] = {0x08,0x08,0x1C,0x08,0x08,0x09,0x06};
static const uint8_t g_u[7] = {0x00,0x00,0x11,0x11,0x11,0x13,0x0D};
static const uint8_t g_v[7] = {0x00,0x00,0x11,0x11,0x11,0x0A,0x04};
static const uint8_t g_w[7] = {0x00,0x00,0x11,0x11,0x15,0x15,0x0A};
static const uint8_t g_x[7] = {0x00,0x00,0x11,0x0A,0x04,0x0A,0x11};
static const uint8_t g_y[7] = {0x00,0x00,0x11,0x11,0x0F,0x01,0x0E};
static const uint8_t g_z[7] = {0x00,0x00,0x1F,0x02,0x04,0x08,0x1F};

static const uint8_t* ILI9488_GetGlyph(char c)
{
    switch (c)
    {
        case ' ': return g_space;
        case '?': return g_qmark;
        case '!': return g_exclam;
        case '.': return g_dot;
        case ',': return g_comma;
        case ':': return g_colon;
        case '-': return g_dash;
        case '_': return g_under;
        case '/': return g_slash;
        case '+': return g_plus;
        case '=': return g_equal;
        case '(': return g_lpar;
        case ')': return g_rpar;

        case '0': return g_0;
        case '1': return g_1;
        case '2': return g_2;
        case '3': return g_3;
        case '4': return g_4;
        case '5': return g_5;
        case '6': return g_6;
        case '7': return g_7;
        case '8': return g_8;
        case '9': return g_9;

        case 'A': return g_A;
        case 'B': return g_B;
        case 'C': return g_C;
        case 'D': return g_D;
        case 'E': return g_E;
        case 'F': return g_F;
        case 'G': return g_G;
        case 'H': return g_H;
        case 'I': return g_I;
        case 'J': return g_J;
        case 'K': return g_K;
        case 'L': return g_L;
        case 'M': return g_M;
        case 'N': return g_N;
        case 'O': return g_O;
        case 'P': return g_P;
        case 'Q': return g_Q;
        case 'R': return g_R;
        case 'S': return g_S;
        case 'T': return g_T;
        case 'U': return g_U;
        case 'V': return g_V;
        case 'W': return g_W;
        case 'X': return g_X;
        case 'Y': return g_Y;
        case 'Z': return g_Z;

        case 'a': return g_a;
        case 'b': return g_b;
        case 'c': return g_c;
        case 'd': return g_d;
        case 'e': return g_e;
        case 'f': return g_f;
        case 'g': return g_g;
        case 'h': return g_h;
        case 'i': return g_i;
        case 'j': return g_j;
        case 'k': return g_k;
        case 'l': return g_l;
        case 'm': return g_m;
        case 'n': return g_n;
        case 'o': return g_o;
        case 'p': return g_p;
        case 'q': return g_q;
        case 'r': return g_r;
        case 's': return g_s;
        case 't': return g_t;
        case 'u': return g_u;
        case 'v': return g_v;
        case 'w': return g_w;
        case 'x': return g_x;
        case 'y': return g_y;
        case 'z': return g_z;

        default:  return g_qmark;
    }
}

/* -------------------- Privadas -------------------- */

static void ILI9488_Select(void)
{
    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_RESET);
}

static void ILI9488_Unselect(void)
{
    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_SET);
}

static void ILI9488_DC_Command(void)
{
    HAL_GPIO_WritePin(ILI9488_DC_GPIO_Port, ILI9488_DC_Pin, GPIO_PIN_RESET);
}

static void ILI9488_DC_Data(void)
{
    HAL_GPIO_WritePin(ILI9488_DC_GPIO_Port, ILI9488_DC_Pin, GPIO_PIN_SET);
}

static void ILI9488_ResetHW(void)
{
    HAL_GPIO_WritePin(ILI9488_RST_GPIO_Port, ILI9488_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(ILI9488_RST_GPIO_Port, ILI9488_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
}

static void ILI9488_WriteCommand(uint8_t cmd)
{
    ILI9488_Select();
    ILI9488_DC_Command();
    HAL_SPI_Transmit(s_hspi, &cmd, 1, HAL_MAX_DELAY);
    ILI9488_Unselect();
}

static void ILI9488_WriteData(const uint8_t *data, uint16_t size)
{
    ILI9488_Select();
    ILI9488_DC_Data();
    HAL_SPI_Transmit(s_hspi, (uint8_t *)data, size, HAL_MAX_DELAY);
    ILI9488_Unselect();
}

static void ILI9488_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    ILI9488_WriteCommand(ILI9488_CASET);
    data[0] = (uint8_t)(x0 >> 8);
    data[1] = (uint8_t)(x0 & 0xFF);
    data[2] = (uint8_t)(x1 >> 8);
    data[3] = (uint8_t)(x1 & 0xFF);
    ILI9488_WriteData(data, 4);

    ILI9488_WriteCommand(ILI9488_PASET);
    data[0] = (uint8_t)(y0 >> 8);
    data[1] = (uint8_t)(y0 & 0xFF);
    data[2] = (uint8_t)(y1 >> 8);
    data[3] = (uint8_t)(y1 & 0xFF);
    ILI9488_WriteData(data, 4);

    ILI9488_WriteCommand(ILI9488_RAMWR);
}

static void ILI9488_WriteColor565(uint16_t color, uint32_t pixels)
{
    uint8_t r5 = (uint8_t)((color >> 11) & 0x1F);
    uint8_t g6 = (uint8_t)((color >> 5)  & 0x3F);
    uint8_t b5 = (uint8_t)( color        & 0x1F);

    uint8_t pixel[3];
    uint8_t buffer[192];
    uint32_t i;

    pixel[0] = (uint8_t)(((uint16_t)r5 * 255U / 31U) & 0xFCU);
    pixel[1] = (uint8_t)(((uint16_t)g6 * 255U / 63U) & 0xFCU);
    pixel[2] = (uint8_t)(((uint16_t)b5 * 255U / 31U) & 0xFCU);

    for (i = 0; i < sizeof(buffer); i += 3U)
    {
        buffer[i]     = pixel[0];
        buffer[i + 1] = pixel[1];
        buffer[i + 2] = pixel[2];
    }

    ILI9488_Select();
    ILI9488_DC_Data();

    while (pixels > 0U)
    {
        uint32_t chunk = (pixels > 64U) ? 64U : pixels;
        HAL_SPI_Transmit(s_hspi, buffer, (uint16_t)(chunk * 3U), HAL_MAX_DELAY);
        pixels -= chunk;
    }

    ILI9488_Unselect();
}

/* -------------------- Públicas -------------------- */

void ILI9488_Init(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    static const uint8_t gamma_pos[] = {
        0x00, 0x04, 0x0E, 0x08, 0x17, 0x0A, 0x40, 0x79,
        0x4D, 0x07, 0x0E, 0x0A, 0x1A, 0x1D, 0x0F
    };

    static const uint8_t gamma_neg[] = {
        0x00, 0x1B, 0x1F, 0x02, 0x10, 0x05, 0x32, 0x34,
        0x43, 0x02, 0x0A, 0x09, 0x33, 0x37, 0x0F
    };

    static const uint8_t power1[] = {0x17, 0x15};
    static const uint8_t power2[] = {0x41};
    static const uint8_t vcom[]   = {0x00, 0x12, 0x80};

    /* Orientación horizontal: valosres {0xE8}, {0x28}*/
    static const uint8_t madctl[] = {0x28};

    /* ILI9488 en modo RGB666, 18 bits por pixel */
    static const uint8_t pixfmt[] = {0x66};

    static const uint8_t ifmode[] = {0x00};
    static const uint8_t frame[]  = {0xA0};
    static const uint8_t inv[]    = {0x02};
    static const uint8_t dispf[]  = {0x02, 0x02};
    static const uint8_t adjust[] = {0xA9, 0x51, 0x2C, 0x82};

    s_hspi = hspi;

    /*
     * Como ahora CS, DC y RST están en GPIOE:
     * CS  = PE13
     * DC  = PE12
     * RST = PE11
     */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(ILI9488_CS_GPIO_Port, ILI9488_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ILI9488_DC_GPIO_Port, ILI9488_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ILI9488_RST_GPIO_Port, ILI9488_RST_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = ILI9488_CS_Pin | ILI9488_DC_Pin | ILI9488_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    ILI9488_Unselect();
    ILI9488_ResetHW();

    ILI9488_WriteCommand(ILI9488_SWRESET);
    HAL_Delay(150);

    ILI9488_WriteCommand(0xE0);
    ILI9488_WriteData(gamma_pos, sizeof(gamma_pos));

    ILI9488_WriteCommand(0xE1);
    ILI9488_WriteData(gamma_neg, sizeof(gamma_neg));

    ILI9488_WriteCommand(0xC0);
    ILI9488_WriteData(power1, sizeof(power1));

    ILI9488_WriteCommand(0xC1);
    ILI9488_WriteData(power2, sizeof(power2));

    ILI9488_WriteCommand(0xC5);
    ILI9488_WriteData(vcom, sizeof(vcom));

    ILI9488_WriteCommand(ILI9488_MADCTL);
    ILI9488_WriteData(madctl, sizeof(madctl));

    ILI9488_WriteCommand(ILI9488_PIXFMT);
    ILI9488_WriteData(pixfmt, sizeof(pixfmt));

    ILI9488_WriteCommand(0xB0);
    ILI9488_WriteData(ifmode, sizeof(ifmode));

    ILI9488_WriteCommand(0xB1);
    ILI9488_WriteData(frame, sizeof(frame));

    ILI9488_WriteCommand(0xB4);
    ILI9488_WriteData(inv, sizeof(inv));

    ILI9488_WriteCommand(0xB6);
    ILI9488_WriteData(dispf, sizeof(dispf));

    ILI9488_WriteCommand(0xF7);
    ILI9488_WriteData(adjust, sizeof(adjust));

    ILI9488_WriteCommand(ILI9488_SLPOUT);
    HAL_Delay(120);

    ILI9488_WriteCommand(ILI9488_DISPON);
    HAL_Delay(120);
}

void ILI9488_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((x >= ILI9488_WIDTH) || (y >= ILI9488_HEIGHT))
    {
        return;
    }

    if ((x + w) > ILI9488_WIDTH)
    {
        w = ILI9488_WIDTH - x;
    }

    if ((y + h) > ILI9488_HEIGHT)
    {
        h = ILI9488_HEIGHT - y;
    }

    ILI9488_SetAddressWindow(x, y, x + w - 1U, y + h - 1U);
    ILI9488_WriteColor565(color, (uint32_t)w * (uint32_t)h);
}

void ILI9488_FillScreen(uint16_t color)
{
    ILI9488_FillRect(0, 0, ILI9488_WIDTH, ILI9488_HEIGHT, color);
}

void ILI9488_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale)
{
    const uint8_t *glyph;
    uint8_t row, col;

    glyph = ILI9488_GetGlyph(c);

    for (row = 0; row < 7U; row++)
    {
        for (col = 0; col < 5U; col++)
        {
            uint16_t px = x + (uint16_t)(col * scale);
            uint16_t py = y + (uint16_t)(row * scale);

            if (glyph[row] & (1U << (4U - col)))
            {
                ILI9488_FillRect(px, py, scale, scale, color);
            }
            else
            {
                ILI9488_FillRect(px, py, scale, scale, bg);
            }
        }
    }

    ILI9488_FillRect(x + (uint16_t)(5U * scale), y, scale, (uint16_t)(7U * scale), bg);
}

void ILI9488_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale)
{
    while ((*str) != '\0')
    {
        ILI9488_DrawChar(x, y, *str, color, bg, scale);
        x += (uint16_t)(6U * scale);
        str++;
    }
}

void ILI9488_DrawTextWrapped(uint16_t x, uint16_t y, uint16_t max_width, const char *str, uint16_t color, uint16_t bg, uint8_t scale)
{
    uint16_t cursor_x = x;
    uint16_t cursor_y = y;
    uint16_t char_w = (uint16_t)(6U * scale);
    uint16_t char_h = (uint16_t)(8U * scale);

    while (*str != '\0')
    {
        if (*str == '\n')
        {
            cursor_x = x;
            cursor_y += char_h;
            str++;
            continue;
        }

        if ((cursor_x + char_w) > (x + max_width))
        {
            cursor_x = x;
            cursor_y += char_h;
        }

        if ((cursor_y + char_h) > ILI9488_HEIGHT)
        {
            break;
        }

        ILI9488_DrawChar(cursor_x, cursor_y, *str, color, bg, scale);
        cursor_x += char_w;
        str++;
    }
}
