#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define AHT10_ADDR 0x38
#define AHT10_CMD_INITIALIZE 0xE1
#define AHT10_CMD_MEASURE 0xAC
#define AHT10_CMD_SOFT_RESET 0xBA
#define AHT10_STATUS_BUSY_MASK 0x80
#define AHT10_STATUS_CAL_MASK 0x08

#define AHT10_I2C_PORT i2c0
#define AHT10_SDA_PIN 0
#define AHT10_SCL_PIN 1
#define AHT10_I2C_BAUDRATE 100000

#include "inc/ssd1306.h"

#define OLED_I2C_PORT i2c1
#define OLED_SDA_PIN 14
#define OLED_SCL_PIN 15
#define OLED_I2C_BAUDRATE 400000

struct render_area frame_area;
uint8_t ssd_buffer[ssd1306_buffer_length];

void aht10_init();
void aht10_reset();
bool aht10_read_data(float *humidity, float *temperature);

void init_oled();
void display_message_oled(const char *message, int line);
void clear_oled_display();

void aht10_init() {
    i2c_init(AHT10_I2C_PORT, AHT10_I2C_BAUDRATE);
    gpio_set_function(AHT10_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(AHT10_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(AHT10_SDA_PIN);
    gpio_pull_up(AHT10_SCL_PIN);

    printf("I2C0 para AHT10 configurado. Tentando resetar AHT10...\n");
    aht10_reset();

    uint8_t init_cmd[3] = {AHT10_CMD_INITIALIZE, 0x08, 0x00};
    int ret = i2c_write_blocking(AHT10_I2C_PORT, AHT10_ADDR, init_cmd, 3, false);
    
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao escrever comando de inicializacao para AHT10.\n");
        display_message_oled("AHT10 Init Err", 0);
        return;
    }

    sleep_ms(300);

    uint8_t status;
    i2c_read_blocking(AHT10_I2C_PORT, AHT10_ADDR, &status, 1, false);
    if (!(status & AHT10_STATUS_CAL_MASK)) {
        printf("AHT10 NAO CALIBRADO! Tente reiniciar o sistema.\n");
        display_message_oled("AHT10 Nao Calib.", 0);
    } else {
        printf("AHT10 inicializado e calibrado com sucesso.\n");
        display_message_oled("AHT10 OK!", 0);
    }
}

void aht10_reset() {
    uint8_t reset_cmd = AHT10_CMD_SOFT_RESET;
    int ret = i2c_write_blocking(AHT10_I2C_PORT, AHT10_ADDR, &reset_cmd, 1, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao enviar comando de reset para AHT10.\n");
    }
    sleep_ms(20);
}

bool aht10_read_data(float *humidity, float *temperature) {
    uint8_t measure_cmd[3] = {AHT10_CMD_MEASURE, 0x33, 0x00};
    int ret = i2c_write_blocking(AHT10_I2C_PORT, AHT10_ADDR, measure_cmd, 3, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao enviar comando de medicao para AHT10.\n");
        return false;
    }

    sleep_ms(80);

    uint8_t status_byte;
    i2c_read_blocking(AHT10_I2C_PORT, AHT10_ADDR, &status_byte, 1, false);

    if (status_byte & AHT10_STATUS_BUSY_MASK) {
        printf("AHT10 Ocupado, nao foi possivel ler os dados.\n");
        return false;
    }

    uint8_t data[6];
    ret = i2c_read_blocking(AHT10_I2C_PORT, AHT10_ADDR, data, 6, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao ler dados do AHT10.\n");
        return false;
    }

    uint32_t raw_humidity = ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
    raw_humidity = raw_humidity >> 4;

    uint32_t raw_temperature = ((uint32_t)data[3] & 0x0F) << 16 | ((uint32_t)data[4] << 8) | data[5];

    *humidity = (float)raw_humidity * 100.0f / 1048576.0f;
    *temperature = (float)raw_temperature * 200.0f / 1048576.0f - 50.0f;

    return true;
}

void init_oled() {
    i2c_init(OLED_I2C_PORT, OLED_I2C_BAUDRATE);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    ssd1306_init();
    frame_area.start_column = 0;
    frame_area.end_column = ssd1306_width - 1;
    frame_area.start_page = 0;
    frame_area.end_page = ssd1306_n_pages - 1;
    calculate_render_area_buffer_length(&frame_area);
    memset(ssd_buffer, 0, ssd1306_buffer_length);

    sleep_ms(100);
    printf("Display OLED inicializado no I2C1.\n");
}

void clear_oled_display() {
    memset(ssd_buffer, 0, ssd1306_buffer_length);
    render_on_display(ssd_buffer, &frame_area);
}

void display_message_oled(const char *message, int line) {
    ssd1306_draw_string(ssd_buffer, 5, line * 8, message);
}

int main() {
    stdio_init_all();

    printf("Iniciando sistema com AHT10 e OLED...\n");

    init_oled();
    clear_oled_display();
    display_message_oled("Iniciando...", 0);
    display_message_oled("AHT10 & OLED", 1);
    render_on_display(ssd_buffer, &frame_area);
    sleep_ms(2000);
    clear_oled_display();

    aht10_init();

    float humidity, temperature;
    char temp_str[32];
    char hum_str[32];

    while (true) {
        if (aht10_read_data(&humidity, &temperature)) {
            printf("Umidade: %.2f %%RH, Temperatura: %.2f C\n", humidity, temperature);
            
            snprintf(temp_str, sizeof(temp_str), "Temp: %.1f C", temperature);
            snprintf(hum_str, sizeof(hum_str), "Umid: %.1f %%RH", humidity);
            
            clear_oled_display();
            display_message_oled(temp_str, 0);
            display_message_oled(hum_str, 2);
            render_on_display(ssd_buffer, &frame_area);
            
        } else {
            printf("Falha na leitura do AHT10. Tentando resetar...\n");
            clear_oled_display();
            display_message_oled("Erro AHT10!", 0);
            render_on_display(ssd_buffer, &frame_area);
            aht10_reset();
            sleep_ms(500);
        }
        sleep_ms(3000);
    }

    return 0;
}