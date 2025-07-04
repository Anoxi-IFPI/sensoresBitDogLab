#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Canal I2C usado
#define I2C_PORT i2c0

// Pinos I2C do Pico
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

// Endereço do sensor BH1750
#define BH1750_ADDR 0x23
#define BH1750_CONTINUOUS_HIGH_RES_MODE 0x10

// GPIOs dos LEDs
#define LED_BAIXO 11
#define LED_MEDIO 12
#define LED_ALTO  13

void bh1750_init() {
    uint8_t buf[1] = {BH1750_CONTINUOUS_HIGH_RES_MODE};
    i2c_write_blocking(I2C_PORT, BH1750_ADDR, buf, 1, false);
}

float bh1750_read_lux() {
    uint8_t data[2];
    int result = i2c_read_blocking(I2C_PORT, BH1750_ADDR, data, 2, false);
    if (result != 2) {
        printf("Erro ao ler o sensor BH1750\n");
        return -1;
    }
    uint16_t raw = (data[0] << 8) | data[1];
    return raw / 1.2; // Conversão para lux
}

void configurar_leds() {
    gpio_init(LED_BAIXO);
    gpio_set_dir(LED_BAIXO, GPIO_OUT);
    gpio_init(LED_MEDIO);
    gpio_set_dir(LED_MEDIO, GPIO_OUT);
    gpio_init(LED_ALTO);
    gpio_set_dir(LED_ALTO, GPIO_OUT);
}

void acender_led_por_lux(float lux) {
    if (lux <= 100) {
        gpio_put(LED_BAIXO, 1);
        gpio_put(LED_MEDIO, 0);
        gpio_put(LED_ALTO, 0);
    } else if (lux <= 300) {
        gpio_put(LED_BAIXO, 1);
        gpio_put(LED_MEDIO, 1);
        gpio_put(LED_ALTO, 0);
    } else if (lux <= 700) {
        gpio_put(LED_BAIXO, 1);
        gpio_put(LED_MEDIO, 1);
        gpio_put(LED_ALTO, 1);
    } else {
        // Muita luz — pisca LED alto
        gpio_put(LED_BAIXO, 1);
        gpio_put(LED_MEDIO, 1);
        for (int i = 0; i < 3; i++) {
            gpio_put(LED_ALTO, 1);
            sleep_ms(150);
            gpio_put(LED_ALTO, 0);
            sleep_ms(150);
        }
    }
}

int main() {
    stdio_init_all();

    // Inicializa o canal I2C
    i2c_init(I2C_PORT, 100 * 1000); // 100kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    sleep_ms(100);
    bh1750_init();
    configurar_leds();

    while (true) {
        float lux = bh1750_read_lux();
        if (lux >= 0) {
            printf("Luminosidade: %.2f lux\n", lux);
            acender_led_por_lux(lux);
        }
        sleep_ms(1000);
    }

    return 0;
}
