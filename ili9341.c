#include "ili9341.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include "font.h"

// Definindo comandos do ILI9341
#define ILI9341_CMD_SWRESET 0x01
#define ILI9341_CMD_SLPOUT 0x11
#define ILI9341_CMD_DISPON 0x29
#define ILI9341_CMD_DISPOFF 0x28
#define ILI9341_CMD_CASET 0x2A
#define ILI9341_CMD_PASET 0x2B
#define ILI9341_CMD_RAMWR 0x2C


// Função para enviar comando ao display
void ILI9341_sendCommand(ILI9341_t* display_, uint8_t cmd) {
    gpio_put(display_->dc, 0); // Define DC como 0 para comando
    gpio_put(display_->cs, 0); // Ativa chip select (CS)
    spi_write_blocking(spi0, &cmd, 1);
    gpio_put(display_->cs, 1); // Desativa chip select (CS)
}

// Função para enviar dados ao display
void ILI9341_sendData(ILI9341_t* display_, uint8_t data) {
    gpio_put(display_->dc, 1); // Define DC como 1 para dados
    gpio_put(display_->cs, 0); // Ativa chip select (CS)
    spi_write_blocking(spi0, &data, 1);
    gpio_put(display_->cs, 1); // Desativa chip select (CS)
}

// Função de reset do display
void ILI9341_reset(ILI9341_t* display_) {
    gpio_put(display_->rst, 0); // Ativa o reset
    sleep_ms(100);           // Aguarda 100ms
    gpio_put(display_->rst, 1);  // Desativa o reset
    sleep_ms(100);           // Aguarda 100ms
}

// Função para configurar a orientação do display
void ILI9341_setOrientation(ILI9341_t* display_) {
    ILI9341_sendCommand(display_, 0x36);  // Comando para definir a orientação
    ILI9341_sendData(display_, display_->orientation); // Dados de orientação
}

// Função para inicializar o display
void ILI9341_init(ILI9341_t* display_) {
    // Inicializa SPI
    spi_init(spi0, 500000); // 500 kHz para inicialização
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    // Configura pinos SPI
    gpio_set_function(display_->sck, GPIO_FUNC_SPI);
    gpio_set_function(display_->mosi, GPIO_FUNC_SPI);
    gpio_init(display_->cs);
    gpio_set_dir(display_->cs, GPIO_OUT);
    gpio_put(display_->cs, 1);

    // Configura pinos de controle
    gpio_init(display_->rst);
    gpio_set_dir(display_->rst, GPIO_OUT);
    gpio_init(display_->dc);
    gpio_set_dir(display_->dc, GPIO_OUT);
    gpio_init(display_->bl);
    gpio_set_dir(display_->bl, GPIO_OUT);
    gpio_put(display_->bl, 1); // Ativa o backlight

    // Reseta e inicializa o display
    ILI9341_reset(display_);
    ILI9341_sendCommand(display_, ILI9341_CMD_SWRESET);
    sleep_ms(150);  // Espera para o reset ser completo
    ILI9341_sendCommand(display_, ILI9341_CMD_SLPOUT);
    sleep_ms(150);  // Espera para sair do modo sleep
    ILI9341_sendCommand(display_, ILI9341_CMD_DISPON);
    sleep_ms(100);  // Espera o d ligar

    // Define a orientação do d
    ILI9341_setOrientation(display_);
}

// Função para converter a cor RGB para o formato RGB565
uint16_t RGB_to_RGB565(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t color = 0;
    color |= ((r >> 3) << 11);  // 5 bits para vermelho
    color |= ((b >> 2) << 5);   // 6 bits para verde
    color |= (g >> 3);          // 5 bits para azul
    return color;
}

// Função para desenhar um retângulo no d
void ILI9341_drawRect(ILI9341_t* display_, int x, int y, int w, int h, uint16_t color) {
    ILI9341_sendCommand(display_, ILI9341_CMD_CASET);  // Set Column Address
    uint8_t data[4] = {x >> 8, x & 0xFF, (x + w - 1) >> 8, (x + w - 1) & 0xFF};
    for (int i = 0; i < 4; i++) ILI9341_sendData(display_, data[i]);

    ILI9341_sendCommand(display_, ILI9341_CMD_PASET);  // Set Row Address
    data[0] = y >> 8;
    data[1] = y & 0xFF;
    data[2] = (y + h - 1) >> 8;
    data[3] = (y + h - 1) & 0xFF;
    for (int i = 0; i < 4; i++) ILI9341_sendData(display_, data[i]);

    ILI9341_sendCommand(display_, ILI9341_CMD_RAMWR);  // Write to RAM

    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;

    ILI9341_sendData(display_, color_low);   // Envie o byte azul primeiro
    ILI9341_sendData(display_, color_high);  // Envie o byte vermelho depois

    for (int i = 0; i < w * h; i++) {
        ILI9341_sendData(display_, color_low);  // Envia a parte azul
        ILI9341_sendData(display_, color_high); // Envia a parte vermelha
    }
}

// Função para desenhar texto no d
void ILI9341_drawText(ILI9341_t* display_, int x, int y, const char* text, uint16_t color, int scale, int fill) {
  while (*text) {
      uint8_t charIndex = 0;
      
      if (*text == ' ') {
          x += 6;  // Adiciona espaço entre as palavras
          text++;
          continue;
      } else if (*text >= '!' && *text <= '~') {
          charIndex = *text - '!' + 1;  // Maiúsculas 'A' a 'Z' começam no índice 33
      } else {
          text++;
          continue;
      }

      const uint8_t* charBitmap = &font6x8[charIndex * 6];

      for (int i = 0; i < 6; i++) {
          uint8_t line = charBitmap[i];
          for (int j = 0; j < 8; j++) {
              if (line & (1 << (7 - j))) {
                  ILI9341_drawRect(display_, (x + i)*scale, (y + j)*scale, fill, fill, color);
              }
          }
      }

      x += 6;
      text++;
  }
}

// Função para limpar toda a tela
void ILI9341_clear(ILI9341_t* display_) {
    ILI9341_drawRect(display_, 0, 0, 320, 240, RGB_to_RGB565(0, 0, 0));
}
