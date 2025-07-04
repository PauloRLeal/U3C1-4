#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ili9341.h"

// Configurações de GPIOs
#define PINO_MPU6050 0x68      // Endereço do MPU6050
#define PINO_SDA 0             // Pino SDA (I2C)
#define PINO_SCL 1             // Pino SCL (I2C)

#define PINO_SERVO 2          // Pino do Servo Motor
#define PINO_LED_VERDE 11      // Pino do LED Verde
#define PINO_LED_AZUL 12       // Pino do LED Azul
#define PINO_LED_VERMELHO 13   // Pino do LED Vermelho

// Definindo os pinos do display para comunicação SPI
ILI9341_t display = {
    .sck = 18,
    .mosi = 19,
    .cs = 17,
    .rst = 16,
    .dc = 8,
    .bl = 9,
    .orientation = 0x60
};


// Configurações do Servo
#define DUTY_MIN 1000          // Pulso de 1 ms (0°)
#define DUTY_MID 1500          // Pulso de 1.5 ms (90°)
#define DUTY_MAX 2000          // Pulso de 2 ms (180°)
#define PERIODO_SERVO 20       // Período do PWM do servo em ms

char conv_inclinacao[26];
char conv_alerta[26];


// Inicializa LEDs RGB
void inicializar_leds() {
    gpio_init(PINO_LED_VERDE);
    gpio_set_dir(PINO_LED_VERDE, GPIO_OUT);
    gpio_put(PINO_LED_VERDE, 0);

    gpio_init(PINO_LED_AZUL);
    gpio_set_dir(PINO_LED_AZUL, GPIO_OUT);
    gpio_put(PINO_LED_AZUL, 0);

    gpio_init(PINO_LED_VERMELHO);
    gpio_set_dir(PINO_LED_VERMELHO, GPIO_OUT);
    gpio_put(PINO_LED_VERMELHO, 0);
}

// Ajusta LEDs conforme a inclinação
void ajustar_led(float inclinacao) {
    if (inclinacao > 30.0) {
        gpio_put(PINO_LED_VERMELHO, 1);
        gpio_put(PINO_LED_AZUL, 0);
        gpio_put(PINO_LED_VERDE, 0);
        printf("LED VERMELHO: Alerta estrutural\n");
    } else if (inclinacao >= 15.0) {
        gpio_put(PINO_LED_VERMELHO, 0);
        gpio_put(PINO_LED_AZUL, 1);
        gpio_put(PINO_LED_VERDE, 0);
        printf("LED AZUL: Atenção\n");
    } else {
        gpio_put(PINO_LED_VERMELHO, 0);
        gpio_put(PINO_LED_AZUL, 0);
        gpio_put(PINO_LED_VERDE, 1);
        printf("LED VERDE: Estrutura estável\n");
    }
}

// Inicializa o MPU6050
void inicializar_mpu6050() {
    uint8_t reset[2] = {0x6B, 0x00};  // Power Management 1
    i2c_write_blocking(i2c0, PINO_MPU6050, reset, 2, false);
}

// Lê valores do acelerômetro
void ler_acelerometro(int16_t *acel_x, int16_t *acel_y, int16_t *acel_z) {
    uint8_t buffer[6];
    i2c_write_blocking(i2c0, PINO_MPU6050, (uint8_t[]){0x3B}, 1, true);
    i2c_read_blocking(i2c0, PINO_MPU6050, buffer, 6, false);
    *acel_x = (buffer[0] << 8) | buffer[1];
    *acel_y = (buffer[2] << 8) | buffer[3];
    *acel_z = (buffer[4] << 8) | buffer[5];
}

// Calcula inclinação no eixo X
float calcular_inclinacao(int16_t acel_x, int16_t acel_y, int16_t acel_z) {
    float acel_x_g = acel_x / 16384.0;
    float acel_y_g = acel_y / 16384.0;
    float acel_z_g = acel_z / 16384.0;
    return atan2(acel_x_g, sqrt(acel_y_g * acel_y_g + acel_z_g * acel_z_g)) * (180.0 / M_PI);
}

// Função para enviar pulsos PWM manualmente ao servo motor
void enviar_pulso_servo(uint duty_us) {
    gpio_put(PINO_SERVO, 1);                  // Sinal alto
    sleep_us(duty_us);                        // Mantém o sinal alto por 'duty_us'
    gpio_put(PINO_SERVO, 0);                  // Sinal baixo
    sleep_ms(PERIODO_SERVO - (duty_us / 1000));  // Aguarda até completar o período (20 ms)
}

// Ajusta o servo motor com base na inclinação
void ajustar_servo(float inclinacao) {
    uint duty_us;

    if (inclinacao > 30.0) {
        duty_us = DUTY_MAX;  // Janela totalmente aberta
        printf("Servo: Janela totalmente aberta\n");
        sprintf(conv_alerta, "Inclinacao > 30.0", inclinacao);
    } else if (inclinacao < 15.0) {
        duty_us = DUTY_MIN;  // Janela fechada
        printf("Servo: Janela fechada\n");
        sprintf(conv_alerta, "Inclinacao < 15.0", inclinacao);
    } else {
        duty_us = DUTY_MID;  // Janela semiaberta
        printf("Servo: Janela semiaberta\n");
        sprintf(conv_alerta, "15.0 <= Inclinacao <= 30.0", inclinacao);
    }

    // Envia pulsos para estabilizar o servo
    for (int i = 0; i < 50; i++) {
        enviar_pulso_servo(duty_us);
    }
}

int main() {
    // Inicializa I2C, LEDs e servo
    stdio_init_all();
    // Inicializa I2C no canal 0
    i2c_init(i2c0, 400 * 1000); // 400 kHz
    gpio_set_function(PINO_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PINO_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PINO_SDA);
    gpio_pull_up(PINO_SCL);

    gpio_init(PINO_SERVO);
    gpio_set_dir(PINO_SERVO, GPIO_OUT);

    inicializar_leds();
    inicializar_mpu6050();

    int16_t acel_x, acel_y, acel_z;
    float inclinacao;

    // Inicializa o display ILI9341
    ILI9341_init(&display);

    // Define a cor da legenda no display (branco neste caso)
    uint16_t color = RGB_to_RGB565(255, 255, 255);  // Converte de RGB para RGB565

    while (true) {
        printf("Iniciando leitura do MPU6050...\n");

        // Lê dados do acelerômetro e calcula a inclinação
        ler_acelerometro(&acel_x, &acel_y, &acel_z);
        inclinacao = calcular_inclinacao(acel_x, acel_y, acel_z);

        printf("Inclinação detectada: %.1f°\n", inclinacao);

        // Ajusta LEDs e servo motor conforme a inclinação
        ajustar_led(inclinacao);
        ajustar_servo(inclinacao);

        sprintf(conv_inclinacao, "Inclinacao: %.3f graus", inclinacao);

        ILI9341_clear(&display);

        ILI9341_drawText(&display, 0, 232, "------- Leituras --------", color, 2, 2);
        ILI9341_drawText(&display, 0, 216, conv_inclinacao, color, 2, 2);

        ILI9341_drawText(&display, 0, 192, "-------- Alertas! --------", color, 2, 2);
        ILI9341_drawText(&display, 0, 176, conv_alerta, color, 2, 2);


        sleep_ms(100);  // Aguarda 1 segundo antes da interação
    }
}
