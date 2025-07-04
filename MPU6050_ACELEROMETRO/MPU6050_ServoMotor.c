#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h> // Para funções trigonométricas como atan2f e sqrtf
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "inc/ssd1306.h" // Biblioteca para o display OLED SSD1306

// --- Definições para o Sensor MPU6050 ---
#define MPU6050_ADDR         0x68 // Endereço I2C do MPU6050 (AD0 para GND)
#define MPU6050_PWR_MGMT_1   0x6B // Registrador Power Management 1
#define MPU6050_ACCEL_XOUT_H 0x3B // Registrador MSB do X do Acelerômetro

// Fator de escala do acelerômetro para ±2g (padrão do MPU6050 ao iniciar)
// 1g (aceleração da gravidade) = 16384 unidades brutas
#define ACCEL_SCALE_FACTOR   16384.0f 

// --- Definições de Pinos e I2C para o MPU6050 (usando I2C0) ---
#define MPU6050_I2C_PORT    i2c0    // Instância I2C0
#define MPU6050_SDA_PIN     0       // Pino GP0 para SDA
#define MPU6050_SCL_PIN     1       // Pino GP1 para SCL
#define MPU6050_I2C_BAUDRATE 100000 // Frequência do I2C para o MPU6050

// --- Definições de Pinos e I2C para o Display OLED (usando I2C1) ---
#define OLED_I2C_PORT        i2c1
#define OLED_SDA_PIN         14
#define OLED_SCL_PIN         15
#define OLED_I2C_BAUDRATE    400000 // Frequência do I2C para o OLED (mais rápida é comum)

// --- Definições para o Servo Motor SG90 ---
#define PINO_SERVO 8 // Pino GPIO conectado ao sinal do servo motor
#define PWM_FREQ   50 // Frequência do PWM em Hz (50Hz para a maioria dos servos = 20ms período)

// Cálculo do PWM_WRAP para uma frequência de 50Hz (20ms de período)
// clk_sys (125MHz) / (freq_pwm * clk_div) - 1
// Se clk_div for 125, então o contador do PWM roda a 1MHz (1us por tick)
// Para 20ms (20000us), o wrap deve ser 19999 (0 a 19999 = 20000 ticks)
#define PWM_WRAP   19999 

// Largura do pulso em microsegundos para as posições do servo SG90
// Estes são valores típicos; pode ser necessário ajustar ligeiramente para seu servo específico
#define SERVO_PULSE_MIN_US 500  // 0 graus (pulso de 0.5ms)
#define SERVO_PULSE_CENTER_US 1500 // 90 graus (pulso de 1.5ms)
#define SERVO_PULSE_MAX_US 2500 // 180 graus (pulso de 2.5ms)

// --- Variáveis Globais para o OLED ---
struct render_area frame_area;
uint8_t ssd_buffer[ssd1306_buffer_length];

// --- Limite de Inclinação para Alerta ---
// Se o Pitch ou Roll exceder este valor (em graus), um alerta será exibido
#define ANGULO_ALERTA_GRAUS 30.0f 

// --- Protótipos de Funções ---
// Funções do MPU6050
void mpu6050_init();
void mpu6050_read_raw_data(int16_t accel[3], int16_t gyro[3]);
void calculate_angles(int16_t accel[3], float *pitch, float *roll);

// Funções do Servo Motor
void init_servo_pwm();
uint32_t angle_to_duty_cycles(float angle_degrees);
void set_servo_angle(float angle_degrees);

// Funções do Display OLED
void init_oled();
void clear_oled_display();
void display_message_oled(const char *message, int line);
void display_angles_oled(float pitch, float roll, const char *alerta_str);

// --- Implementação das Funções do MPU6050 ---

void mpu6050_init() {
    i2c_init(MPU6050_I2C_PORT, MPU6050_I2C_BAUDRATE);
    gpio_set_function(MPU6050_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA_PIN);
    gpio_pull_up(MPU6050_SCL_PIN);

    printf("I2C0 para MPU6050 configurado.\n");
    sleep_ms(100);

    // Acorda o MPU6050 e seleciona o clock interno
    uint8_t buf[2];
    buf[0] = MPU6050_PWR_MGMT_1; 
    buf[1] = 0x00; 
    
    int ret = i2c_write_blocking(MPU6050_I2C_PORT, MPU6050_ADDR, buf, 2, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao acordar MPU6050! Verifique conexoes e endereco I2C.\n");
    } else {
        printf("MPU6050 acordado e inicializado com sucesso.\n");
    }
    sleep_ms(100);
}

void mpu6050_read_raw_data(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[14]; // Buffer para armazenar os 14 bytes de dados (Accel X,Y,Z, Temp, Gyro X,Y,Z)

    uint8_t reg_addr = MPU6050_ACCEL_XOUT_H; // Começa a ler do registrador do acelerômetro X
    // Envia o endereço do registrador e mantém a conexão I2C ativa (repeated start = true)
    int ret = i2c_write_blocking(MPU6050_I2C_PORT, MPU6050_ADDR, &reg_addr, 1, true); 
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao solicitar leitura de dados do MPU6050.\n");
        // Em caso de erro, zera os arrays para evitar dados inválidos
        memset(accel, 0, sizeof(int16_t)*3);
        memset(gyro, 0, sizeof(int16_t)*3);
        return;
    }
    
    // Lê os 14 bytes de dados do sensor
    ret = i2c_read_blocking(MPU6050_I2C_PORT, MPU6050_ADDR, buffer, 14, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao ler dados do MPU6050.\n");
        memset(accel, 0, sizeof(int16_t)*3);
        memset(gyro, 0, sizeof(int16_t)*3);
        return;
    }

    // Combina os bytes alto e baixo para formar os valores de 16 bits
    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]);  // AccelX
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]);  // AccelY
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]);  // AccelZ

    gyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]);   // GyroX
    gyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]); // GyroY
    gyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]); // GyroZ
}

// Calcula os ângulos Pitch e Roll a partir dos dados do acelerômetro
void calculate_angles(int16_t accel[3], float *pitch, float *roll) {
    // Converte os valores brutos do acelerômetro para 'g' (aceleração da gravidade)
    float accel_x = (float)accel[0] / ACCEL_SCALE_FACTOR;
    float accel_y = (float)accel[1] / ACCEL_SCALE_FACTOR;
    float accel_z = (float)accel[2] / ACCEL_SCALE_FACTOR;

    // Fórmula para Pitch (inclinação para frente/trás) em graus
    // atan2f(Ay, sqrt(Ax^2 + Az^2)) para um MPU montado de uma forma específica
    // Aqui usamos (Ax, sqrt(Ay^2 + Az^2)) para Pitch e (Ay, sqrt(Ax^2 + Az^2)) para Roll
    // O M_PI é definido em math.h
    *pitch = atan2f(accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;

    // Fórmula para Roll (inclinação para os lados) em graus
    *roll = atan2f(accel_y, sqrtf(accel_x * accel_x + accel_z * accel_z)) * 180.0f / M_PI;
}


// --- Implementação das Funções do Servo Motor ---

void init_servo_pwm() {
    gpio_set_function(PINO_SERVO, GPIO_FUNC_PWM); // Define o pino do servo como uma saída PWM

    uint slice_num = pwm_gpio_to_slice_num(PINO_SERVO); // Obtém o número do slice PWM associado ao pino
    uint chan_num = pwm_gpio_to_channel(PINO_SERVO);   // Obtém o número do canal PWM (A ou B) associado ao pino

    // Configura o período do PWM (WRAP)
    // Para 50Hz (20ms de período) com um clock divisor de 125, o contador vai de 0 a 19999.
    pwm_set_wrap(slice_num, PWM_WRAP); 
    
    // Configura o divisor do clock do PWM
    // clk_sys (125MHz) / 125 = 1MHz (o contador PWM incrementa a cada 1us)
    pwm_set_clkdiv(slice_num, 125.0f); 

    pwm_set_chan_level(slice_num, chan_num, 0); // Define o duty cycle inicial como 0 (servo parado)
    pwm_set_enabled(slice_num, true); // Habilita o hardware PWM para começar a gerar pulsos
    printf("Servo PWM configurado no pino GP%d.\n", PINO_SERVO);
}

// Converte um ângulo em graus (0-180) para a largura do pulso em microsegundos
uint32_t angle_to_duty_cycles(float angle_degrees) {
    // Mapeia o ângulo para a largura do pulso usando interpolação linear
    uint32_t duty_us = (uint32_t)(SERVO_PULSE_MIN_US + 
                                  (angle_degrees / 180.0f) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US));
    
    // Garante que o valor do pulso esteja dentro dos limites definidos
    if (duty_us < SERVO_PULSE_MIN_US) duty_us = SERVO_PULSE_MIN_US;
    if (duty_us > SERVO_PULSE_MAX_US) duty_us = SERVO_PULSE_MAX_US;
    
    return duty_us; // Retorna a largura do pulso em microsegundos
}

// Define a posição do servo em graus (0-180)
void set_servo_angle(float angle_degrees) {
    uint slice_num = pwm_gpio_to_slice_num(PINO_SERVO);
    uint chan_num = pwm_gpio_to_channel(PINO_SERVO); // Correção: pwm_gpio_to_channel

    uint32_t duty_us = angle_to_duty_cycles(angle_degrees);
    
    // Define o duty cycle do canal PWM. Como cada tick do PWM é 1us,
    // o duty_us é diretamente o número de ticks.
    pwm_set_chan_level(slice_num, chan_num, duty_us);
}

// --- Implementação das Funções do Display OLED ---

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

// Limpa o buffer do display, mas não o renderiza na tela para otimização
void clear_oled_display() {
    memset(ssd_buffer, 0, ssd1306_buffer_length);
}

// Desenha uma mensagem no buffer do OLED em uma linha específica
void display_message_oled(const char *message, int line) {
    // A função ssd1306_draw_string espera um char *, então fazemos um cast para remover o 'const'
    // (idealmente, o protótipo da biblioteca ssd1306.h seria atualizado para const char*)
    ssd1306_draw_string(ssd_buffer, 5, line * 8, (char *)message); 
}

// Exibe os ângulos Pitch e Roll e uma mensagem de alerta no OLED
void display_angles_oled(float pitch, float roll, const char *alerta_str) {
    char pitch_str[32];
    char roll_str[32];

    snprintf(pitch_str, sizeof(pitch_str), "Pitch: %.1f deg", pitch);
    snprintf(roll_str, sizeof(roll_str), "Roll:  %.1f deg", roll);

    clear_oled_display(); // Limpa o buffer antes de desenhar os novos dados

    display_message_oled(pitch_str, 0); // Linha 0 para Pitch
    display_message_oled(roll_str, 2);  // Linha 2 para Roll (com espaço de uma linha)

    if (alerta_str[0] != '\0') { // Se a string de alerta não estiver vazia
        display_message_oled(alerta_str, 4); // Exibe o alerta na linha 4
    } 
    
    render_on_display(ssd_buffer, &frame_area); // Envia o conteúdo do buffer para o display OLED
}


// --- Função Principal ---
int main() {
    stdio_init_all(); // Inicializa a comunicação serial via USB

    printf("Iniciando sistema de monitoramento de inclinacao e controle de servo...\n");

    // --- Inicialização de Periféricos ---
    // É uma boa prática inicializar o display primeiro para que ele possa mostrar mensagens de status/erro
    init_oled();      
    init_servo_pwm(); // Inicializa o Servo Motor
    mpu6050_init();   // Inicializa o MPU6050 (acelerômetro/giroscópio)

    // --- Mensagem de Início no OLED ---
    clear_oled_display();
    display_message_oled("MPU6050 + Servo", 0);
    display_message_oled("Pronto!", 2);
    render_on_display(ssd_buffer, &frame_area); // Exibe a mensagem na tela
    sleep_ms(2000); // Exibe a mensagem por 2 segundos
    clear_oled_display(); // Limpa para a primeira leitura

    float pitch = 0.0f;
    float roll = 0.0f;
    int16_t accel_data[3]; // Armazena os dados brutos do acelerômetro (X, Y, Z)
    int16_t gyro_data[3];  // Armazena os dados brutos do giroscópio (X, Y, Z)
    char alerta_str[32];   // Buffer para a mensagem de alerta no OLED

    // --- Posição Inicial do Servo Motor ---
    // Centraliza o servo em 90 graus no início
    set_servo_angle(90.0f); 
    sleep_ms(500); // Dá um tempo para o servo se mover para a posição inicial

    // --- Loop Principal do Programa ---
    while (true) {
        // 1. Leitura e Cálculo dos Ângulos
        mpu6050_read_raw_data(accel_data, gyro_data); // Lê os valores brutos do MPU6050
        calculate_angles(accel_data, &pitch, &roll); // Calcula Pitch e Roll a partir dos dados do acelerômetro

        printf("Pitch: %.2f deg, Roll: %.2f deg\n", pitch, roll); // Imprime no console serial

        // 2. Controle Dinâmico do Servo Motor
        // Mapeia o ângulo Pitch (-90 a 90 graus) para a faixa do servo (0 a 180 graus).
        // Um Pitch de -90deg corresponde a 0deg do servo, 0deg Pitch a 90deg servo, 90deg Pitch a 180deg servo.
        float servo_angle = pitch + 90.0f; 
        
        // Limita o ângulo do servo para garantir que ele não tente ir além de sua faixa física
        if (servo_angle < 0.0f) {
            servo_angle = 0.0f;
        }
        if (servo_angle > 180.0f) {
            servo_angle = 180.0f;
        }
        set_servo_angle(servo_angle); // Move o servo para o ângulo calculado

        // 3. Lógica de Alerta Visual para o OLED
        // Verifica se o valor absoluto (sem sinal) de Pitch ou Roll excede o limite de alerta
        if (fabs(pitch) > ANGULO_ALERTA_GRAUS || fabs(roll) > ANGULO_ALERTA_GRAUS) {
            snprintf(alerta_str, sizeof(alerta_str), "ALERTA: Inclinado!");
        } else {
            alerta_str[0] = '\0'; // Nenhuma mensagem de alerta (string vazia)
        }

        // 4. Atualiza o Display OLED
        // Exibe os ângulos e, se houver, a mensagem de alerta no OLED
        display_angles_oled(pitch, roll, alerta_str);

        sleep_ms(100); // Pequeno atraso para a próxima leitura e atualização (100ms = 10x por segundo)
    }

    return 0; // O programa embutido normalmente nunca sai do loop principal
}