#ifndef ILI9341_H
#define ILI9341_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include "font.h"

// Estrutura para armazenar os dados e pinos do display
typedef struct {
    uint8_t sck;
    uint8_t mosi;
    uint8_t cs;
    uint8_t rst;
    uint8_t dc;
    uint8_t bl;
    uint8_t orientation;
    // Valor	Descrição
    // 0x00	Modo de orientação normal (paisagem)
    // 0x60	Rotação de 90 graus (retrato)
    // 0xC0	Rotação de 180 graus (paisagem invertida)
    // 0xA0	Rotação de 270 graus (retrato invertido)
} ILI9341_t;

// Funções públicas
void ILI9341_init(ILI9341_t* display);
void ILI9341_drawRect(ILI9341_t* display_, int x, int y, int w, int h, uint16_t color);
void ILI9341_drawText(ILI9341_t* display_, int x, int y, const char* text, uint16_t color, int scale, int fill);
uint16_t RGB_to_RGB565(uint8_t r, uint8_t g, uint8_t b);
void ILI9341_clear(ILI9341_t* display_);

#endif
