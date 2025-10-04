#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h" // Garanta que FreqS aqui seja 50

// =========================================================================
// DEFINIÇÕES DE HARDWARE E SENSOR
// =========================================================================
#define I2C_PORT       i2c0     // Controlador I2C a ser usado
#define I2C_SDA_PIN    0        // Pino SDA
#define I2C_SCL_PIN    1        // Pino SCL
#define I2C_FREQ       400000   // Frequência do I2C (400 kHz)

// A taxa de amostragem do sensor (50 Hz) deve corresponder ao FreqS em spo2_algorithm.h
// que agora definimos como 50 Hz. O MAX30105_SAMPLERATE_50 é usado na configuração.

// Limites mínimos para a detecção de um sinal válido.
// Se os valores de IR e RED caírem abaixo disso, provavelmente não há dedo ou o sinal é muito fraco.
// Estes valores são empíricos e podem precisar de ajuste dependendo do seu sensor e aplicação.
#define MIN_VALID_RAW_VALUE 1000 // Se o IR e RED forem menores que isso, o sinal é suspeito.

// =========================================================================
// INSTÂNCIAS DE OBJETOS E VARIÁVEIS
// =========================================================================
MAX30105 particleSensor; // Instância do objeto sensor

// Buffers para os dados brutos de IR e Red para o algoritmo
// BUFFER_SIZE é calculado com base em FreqS (50 Hz * 4 segundos = 200)
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

// Variáveis para os resultados do algoritmo
int32_t spo2 = 0;
int8_t spo2_valid = 0;
int32_t heartRate = 0;
int8_t heartRate_valid = 0;

int bufferIdx = 0; // Índice atual para preencher os buffers circularmente

// =========================================================================
// FUNÇÃO PRINCIPAL
// =========================================================================

int main() {
    stdio_init_all(); // Inicializa USB serial
    printf("Iniciando sistema oxímetro/HR com RP2040...\n");

    // Inicializa I2C
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN); // Habilita pull-ups internos (verifique se seu módulo tem pull-ups externos)
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C inicializado: SDA=GP%d, SCL=GP%d @ %d Hz\n", I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);

    // Inicializa o sensor
    if (!particleSensor.begin(I2C_PORT, MAX30105_ADDRESS)) {
        printf("ERRO: MAX30105 não encontrado. Verifique conexões.\n");
        while (1) { sleep_ms(1000); }
    }
    printf("MAX30105 encontrado e pronto.\n");

    // Configurações do sensor para otimização do sinal
    // Parâmetros: powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange
    // ledMode = 2 (Red+IR para oximetria)
    // sampleRate = 50 Hz (MAX30105_SAMPLERATE_50) - **IMPORTANTE: FreqS em spo2_algorithm.h DEVE SER 50**
    
    // AJUSTES AQUI:
    // powerLevel: Aumentado para 0x7F (25.4mA) ou 0xFF (50mA) para tentar um sinal mais forte.
    // sampleAverage: Aumentado para 8 para mais suavização do ruído.
    particleSensor.setup(0x7F, 8, 2, 50, 411, 4096); 
    printf("Sensor configurado para leituras de oximetria/HR.\n");
    printf("Posicione o dedo sobre o sensor (cubra bem) para iniciar...\n");

    while (1) {
        // Verifica quantas novas amostras estão disponíveis no FIFO do sensor
        uint16_t num_new_samples = particleSensor.check();

        // Se houver novas amostras, processe-as
        if (num_new_samples > 0) {
            bool current_sample_valid = true; // Flag para verificar a validade da amostra atual

            for (int i = 0; i < num_new_samples; i++) {
                uint32_t currentIR = particleSensor.getFIFOIR();
                uint32_t currentRed = particleSensor.getFIFORed();
                
                // Verifica se a amostra bruta tem um valor mínimo, indicando um sinal real
                if (currentIR < MIN_VALID_RAW_VALUE || currentRed < MIN_VALID_RAW_VALUE) {
                    current_sample_valid = false; // Sinal muito baixo, provavelmente sem dedo ou com luz
                    // Opcional: imprimir aviso aqui para depuração inicial
                    // printf("AVISO: Sinal bruto muito baixo. IR: %lu, Red: %lu\n", currentIR, currentRed);
                }

                irBuffer[bufferIdx] = currentIR;
                redBuffer[bufferIdx] = currentRed;
                
                particleSensor.nextSample(); 

                bufferIdx = (bufferIdx + 1) % BUFFER_SIZE;
            }

            // O algoritmo precisa de BUFFER_SIZE (200) amostras para um cálculo inicial.
            // Apenas execute o algoritmo depois que o buffer estiver preenchido o suficiente
            // E se a última amostra lida foi considerada válida.
            if (current_sample_valid && bufferIdx >= BUFFER_SIZE - 1) { 
                maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &spo2_valid, &heartRate, &heartRate_valid);

                // Exibe os resultados apenas se os dados forem considerados válidos pelo algoritmo
                printf("-------------------------------------------\n");
                printf("Amostras processadas: %d\n", num_new_samples);
                
                if (heartRate_valid) {
                    printf("Frequência Cardíaca: %ld BPM\n", heartRate);
                } else {
                    printf("Frequência Cardíaca: Aguardando sinal válido...\n");
                }

                if (spo2_valid) {
                    printf("Saturação de Oxigênio (SpO2): %ld%%\n", spo2);
                } else {
                    printf("Saturação de Oxigênio (SpO2): Aguardando sinal válido...\n");
                }
            } else if (!current_sample_valid) {
                 // Se o sinal não foi válido, não faz sentido calcular e exibir resultados
                 // Pode-se imprimir uma mensagem para o usuário reposicionar o dedo
                 printf("AVISO: Sinal fraco/Ausente. Posicione o dedo corretamente e evite luz ambiente.\n");
            }
        }
        
        sleep_ms(20); // Atraso de 20ms para 50 Hz de amostragem
    }

    return 0; 
}