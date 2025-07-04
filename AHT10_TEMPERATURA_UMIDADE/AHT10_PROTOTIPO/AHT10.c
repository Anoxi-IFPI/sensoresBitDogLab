#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h" // Para configurar pinos GPIO

// --- Definições para o AHT10 ---
#define AHT10_ADDR           0x38  // Endereço I2C do AHT10
#define AHT10_CMD_INITIALIZE 0xE1  // Comando para inicialização/calibração
#define AHT10_CMD_MEASURE    0xAC  // Comando para iniciar a medição
#define AHT10_CMD_SOFT_RESET 0xBA  // Comando para resetar o sensor
#define AHT10_STATUS_BUSY_MASK 0x80 // Máscara para verificar se o sensor está ocupado
#define AHT10_STATUS_CAL_MASK  0x08 // Máscara para verificar se o sensor está calibrado

// --- Definições I2C para o Pico ---
#define I2C_PORT    i2c0    // Usaremos a instância I2C0
#define I2C_SDA_PIN 0       // Pino GP0 para SDA
#define I2C_SCL_PIN 1       // Pino GP1 para SCL
#define I2C_BAUDRATE 100000 // Frequência do I2C: 100 kHz (padrão)

// --- Protótipos de Funções ---
void aht10_init();
void aht10_reset();
bool aht10_read_data(float *humidity, float *temperature);

// --- Função: aht10_init ---
// Inicializa a comunicação I2C e o sensor AHT10
void aht10_init() {
    // Inicializa a interface I2C0
    i2c_init(I2C_PORT, I2C_BAUDRATE);

    // Configura os pinos GPIO para I2C
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);

    // Habilita pull-ups internos (importante para I2C)
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("I2C e GPIOs configurados. Tentando resetar AHT10...\n");
    aht10_reset(); // Sempre bom resetar no início

    // Envia o comando de inicialização/calibração
    uint8_t init_cmd[3] = {AHT10_CMD_INITIALIZE, 0x08, 0x00};
    int ret = i2c_write_blocking(I2C_PORT, AHT10_ADDR, init_cmd, 3, false);
    
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao escrever comando de inicializacao para AHT10.\n");
        return;
    }

    sleep_ms(300); // Espera o sensor inicializar e calibrar

    // Verifica o estado de calibração
    uint8_t status;
    i2c_read_blocking(I2C_PORT, AHT10_ADDR, &status, 1, false);
    if (!(status & AHT10_STATUS_CAL_MASK)) {
        printf("AHT10 NAO CALIBRADO! Tente reiniciar o sistema.\n");
    } else {
        printf("AHT10 inicializado e calibrado com sucesso.\n");
    }
}

// --- Função: aht10_reset ---
// Reseta o sensor AHT10
void aht10_reset() {
    uint8_t reset_cmd = AHT10_CMD_SOFT_RESET;
    int ret = i2c_write_blocking(I2C_PORT, AHT10_ADDR, &reset_cmd, 1, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao enviar comando de reset para AHT10.\n");
    }
    sleep_ms(20); // O datasheet recomenda esperar 20ms após o reset
}


// --- Função: aht10_read_data ---
// Realiza a medição e leitura dos dados de umidade e temperatura
// Retorna true em caso de sucesso, false em caso de falha
bool aht10_read_data(float *humidity, float *temperature) {
    // 1. Envia o comando de medição
    uint8_t measure_cmd[3] = {AHT10_CMD_MEASURE, 0x33, 0x00};
    int ret = i2c_write_blocking(I2C_PORT, AHT10_ADDR, measure_cmd, 3, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao enviar comando de medicao para AHT10.\n");
        return false;
    }

    // 2. Espera a medição ser concluída (máximo 75ms)
    sleep_ms(80); // Um pouco mais do que o máximo para garantir

    // 3. Lê o byte de status para verificar se o sensor está ocupado
    uint8_t status_byte;
    i2c_read_blocking(I2C_PORT, AHT10_ADDR, &status_byte, 1, false);

    if (status_byte & AHT10_STATUS_BUSY_MASK) {
        printf("AHT10 Ocupado, nao foi possivel ler os dados.\n");
        return false;
    }

    // 4. Lê os 6 bytes de dados (1 byte de status + 2x 2.5 bytes de dados)
    uint8_t data[6];
    ret = i2c_read_blocking(I2C_PORT, AHT10_ADDR, data, 6, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Erro ao ler dados do AHT10.\n");
        return false;
    }

    // O primeiro byte é o status_byte (data[0]), já lido acima, mas relido aqui
    // data[1] e data[2] são os 8 bits mais significativos da umidade
    // data[3] contém os 4 bits menos significativos da umidade e os 4 bits mais significativos da temperatura
    // data[4] e data[5] são os 8 bits menos significativos da temperatura

    // Converte os dados brutos
    uint32_t raw_humidity = ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
    raw_humidity = raw_humidity >> 4; // Desloca 4 bits para a direita, como os 4 LSBs estão no data[3]

    uint32_t raw_temperature = ((uint32_t)data[3] & 0x0F) << 16 | ((uint32_t)data[4] << 8) | data[5];

    // Calcula a Umidade Relativa (RH%)
    *humidity = (float)raw_humidity * 100.0f / 1048576.0f; // 1048576 = 2^20

    // Calcula a Temperatura (°C)
    *temperature = (float)raw_temperature * 200.0f / 1048576.0f - 50.0f; // 1048576 = 2^20

    return true;
}

// --- Função Principal ---
int main() {
    stdio_init_all(); // Inicializa comunicação serial (UART) para printf

    printf("Iniciando AHT10 no Pico...\n");

    aht10_init(); // Inicializa o sensor AHT10

    float humidity, temperature;

    while (true) {
        if (aht10_read_data(&humidity, &temperature)) {
            printf("Umidade: %.2f %%RH, Temperatura: %.2f C\n", humidity, temperature);
        } else {
            printf("Falha na leitura do AHT10. Tentando resetar...\n");
            aht10_reset(); // Tenta resetar em caso de falha
            sleep_ms(500); // Pequeno atraso antes de tentar novamente
        }
        sleep_ms(2000); // Lê a cada 2 segundos
    }

    return 0;
}