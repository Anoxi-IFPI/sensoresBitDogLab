#include <stdio.h> //entrada e saida padrão
#include <string.h> //Biblioteca para desenhar strings no dispaly
#include <stdlib.h>  //
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "inc/ssd1306.h"

// Definições dos pinos (conforme seu código original)
#define VERDE 11
#define AZUL 12
#define VERMELHO 13
// #define BTA 6
// #define BTB 7

// OLED - usando i2c1
const uint I2C_SDA = 14;  // GP14
const uint I2C_SCL = 15;  // GP15

// BH1750 - usando i2c0
#define I2C_BH1750_PORT i2c0
#define I2C_BH1750_SDA 0  // GP0
#define I2C_BH1750_SCL 1  // GP1

// servo motor
#define PINO_SERVO 8
#define PERIODO_SERVO 20

void enviar_pulso(uint duty_us) {
    gpio_put(PINO_SERVO, 1);
    sleep_us(duty_us);
    gpio_put(PINO_SERVO, 0);
    sleep_ms(PERIODO_SERVO - (duty_us / 1000));
}

// Configuração PWM
const uint16_t periodo = 200;
const float divisor_pwm = 16.0;
const uint16_t stemp_pwm = 40;
uint8_t nivelLuminosidade = 0;
uint8_t contadorA = 0;

// BH1750
#define BH1750_ADDR 0x23
#define BH1750_CONTINUOUS_HIGH_RES_MODE 0x10

// Variáveis globais para o OLED
struct render_area frame_area;
uint8_t ssd[ssd1306_buffer_length];

void config_pwm(int led) {
    uint slice;
    gpio_set_function(led, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(led);
    pwm_set_clkdiv(slice, divisor_pwm);
    pwm_set_wrap(slice, periodo);
    pwm_set_gpio_level(led, stemp_pwm);
    pwm_set_enabled(slice, true);
}

void bh1750_init() {
    uint8_t buf[1] = {BH1750_CONTINUOUS_HIGH_RES_MODE};
    i2c_write_blocking(I2C_BH1750_PORT, BH1750_ADDR, buf, 1, false);
}

float bh1750_read_lux() {
    uint8_t data[2];
    int result = i2c_read_blocking(I2C_BH1750_PORT, BH1750_ADDR, data, 2, false);
    if (result != 2) {
        printf("Erro ao ler o sensor BH1750\n");
        return -1;
    }
    uint16_t raw = (data[0] << 8) | data[1];
    return raw / 1.2;
}

void exibir_mensagem(const char *mensagem) {
    memset(ssd, 0, ssd1306_buffer_length);
    ssd1306_draw_string(ssd, 5, 0, mensagem);
    render_on_display(ssd, &frame_area);
}

void atualizar_display_por_lux(float lux) {
    char mensagem[50];
    
    if (lux < 10) {
        snprintf(mensagem, sizeof(mensagem), "Escuro: %.1f lx", lux);
        pwm_set_gpio_level(VERDE, 0);
        pwm_set_gpio_level(AZUL, 0);
        pwm_set_gpio_level(VERMELHO, 0);
    } 
    else if (lux >= 10 && lux < 50) {
        snprintf(mensagem, sizeof(mensagem), "lux baixa: %.1f lx", lux);
        pwm_set_gpio_level(VERDE, periodo * 0.2);
        pwm_set_gpio_level(AZUL, 0);
        pwm_set_gpio_level(VERMELHO, 0);
        enviar_pulso(500);
    } 
    else if (lux >= 50 && lux < 200) {
        snprintf(mensagem, sizeof(mensagem), "Iluminado: %.1f lx", lux);
        pwm_set_gpio_level(VERDE, periodo * 0.5);
        pwm_set_gpio_level(AZUL, periodo * 0.2);
        pwm_set_gpio_level(VERMELHO, 0);
        enviar_pulso(1000);
    } 
    else if (lux >= 200 && lux < 1000) {
        snprintf(mensagem, sizeof(mensagem), "Claro: %.1f lx", lux);
        pwm_set_gpio_level(VERDE, periodo);
        pwm_set_gpio_level(AZUL, periodo * 0.5);
        pwm_set_gpio_level(VERMELHO, periodo * 0.2);
        enviar_pulso(1500);
    } 
    else {
        snprintf(mensagem, sizeof(mensagem), "lux alta: %.1f lx", lux);
        pwm_set_gpio_level(VERDE, periodo);
        pwm_set_gpio_level(AZUL, periodo);
        pwm_set_gpio_level(VERMELHO, periodo);
        enviar_pulso(2000);

    }
    
    exibir_mensagem(mensagem);
}

// void Incrementar(int led, int botao) {
//     if (gpio_get(botao)) return;
    
//     sleep_ms(50); // Debounce
//     if (gpio_get(botao)) return;
    
//     if (botao == BTA && nivelLuminosidade < 5) {
//         contadorA++;
//         nivelLuminosidade++;
//     } 
//     else if (botao == BTB && nivelLuminosidade > 0) {
//         nivelLuminosidade--;
//     }
    
//     printf("Nivel: %d\n", nivelLuminosidade);
//     pwm_set_gpio_level(led, periodo / 5 * nivelLuminosidade);
    
//     while (!gpio_get(botao)); // Espera soltar o botão
//     sleep_ms(200);
// }

int main() {
    stdio_init_all();

    gpio_init(PINO_SERVO);
    gpio_set_dir(PINO_SERVO, GPIO_OUT);
    
    // Inicializa I2C0 para BH1750 (GP0 e GP1)
    i2c_init(I2C_BH1750_PORT, 100 * 1000);
    gpio_set_function(I2C_BH1750_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_BH1750_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_BH1750_SDA);
    gpio_pull_up(I2C_BH1750_SCL);
    
    // Inicializa I2C1 para OLED (GP14 e GP15)
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Inicializa OLED
    ssd1306_init();
    frame_area.start_column = 0;
    frame_area.end_column = ssd1306_width - 1;
    frame_area.start_page = 0;
    frame_area.end_page = ssd1306_n_pages - 1;
    calculate_render_area_buffer_length(&frame_area);
    memset(ssd, 0, ssd1306_buffer_length);
    
    // Inicializa BH1750
    bh1750_init();
    
    // Configura LEDs PWM
    config_pwm(VERDE);
    config_pwm(AZUL);
    config_pwm(VERMELHO);
    
    // Configura botões
    // gpio_init(BTA);
    // gpio_set_dir(BTA, GPIO_IN);
    // gpio_pull_up(BTA);
    // gpio_init(BTB);
    // gpio_set_dir(BTB, GPIO_IN);
    // gpio_pull_up(BTB);
    
    exibir_mensagem("Sistema Iniciado");
    sleep_ms(2000);
    
    bool modo_automatico = true;
    
    while (true) {
        float lux = bh1750_read_lux();
        
        if (modo_automatico && lux >= 0) {
            atualizar_display_por_lux(lux);
        } else {
            char msg[30];
            snprintf(msg, sizeof(msg), "Manual: Nivel %d", nivelLuminosidade);
            exibir_mensagem(msg);
        }
        
        // Verifica botões para alternar modo
        // if (!gpio_get(BTA) && !gpio_get(BTB)) {
        //     sleep_ms(50);
        //     if (!gpio_get(BTA) && !gpio_get(BTB)) {
        //         modo_automatico = !modo_automatico;
        //         exibir_mensagem(modo_automatico ? "Modo Auto" : "Modo Manual");
        //         sleep_ms(1000);
        //         while (!gpio_get(BTA) || !gpio_get(BTB));
        //     }
        // }
        
        // if (!modo_automatico) {
        //     Incrementar(VERDE, BTA);
        //     Incrementar(VERDE, BTB);
        // }
        
        sleep_ms(100);
    }
    
    return 0;
}