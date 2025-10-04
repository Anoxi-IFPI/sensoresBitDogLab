#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "tof.h"

#define I2C_PORT i2c0 
#define PICO_DEFAULT_I2C_SDA_PIN 0
#define PICO_DEFAULT_I2C_SCL_PIN 1

int main() {
    int iDistance;
    int model, revision;

    stdio_init_all();

    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    sleep_ms(3000);

    printf("Iniciando o sensor VL53L0X no Kit BitDogLab...\n");

    int status_model_rev = tofGetModel(&model, &revision);
    if (status_model_rev) {
        printf("Sensor VL53L0X encontrado!\n");
        printf("Modelo ID: %d\n", model);
        printf("Revisao ID: %d\n", revision);
    } else {
        printf("Erro: Nao foi possivel obter o modelo/revisao do sensor. Verifique as conexoes I2C.\n");
        return 1;
    }

    int init_status = tofInit(0, 0x29, 1); 
    if (init_status != 1) {
        printf("Falha ao inicializar o VL53L0X. Verifique as conexoes ou o endereco I2C.\n");
        return 1;
    }
    printf("VL53L0X inicializado com sucesso.\n");

    printf("Comecando a leitura de distancia continua...\n");
    
    while (true) {
        iDistance = tofReadDistance();

        if ((iDistance < 4096) && (iDistance > 0)) {
            printf("Distancia: %d mm\n", iDistance);
        } 
        sleep_ms(50); 
    }

    return 0;
}