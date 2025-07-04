#include <stdio.h>
#include "pico/stdlib.h"

// Definições dos pinos
#define TRIG_PIN 9  // Pino de Trigger do HC-SR04
#define ECHO_PIN 8  // Pino de Echo do HC-SR04
#define RED_LED 13  // LED para distância curta
#define BLUE_LED 12 // LED para distância média
#define GREEN_LED 11 // LED para distância longa
#define BUZZER 1    // Pino para o buzzer

void alternar_led(int red, int blue, int green) {
    gpio_put(RED_LED, red);
    gpio_put(BLUE_LED, blue);
    gpio_put(GREEN_LED, green);
}

void bit_buzzer(int estado) {
    gpio_put(BUZZER, estado);
}

void setup() {
    // Inicializa os pinos
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    gpio_init(RED_LED);
    gpio_set_dir(RED_LED, GPIO_OUT);

    gpio_init(BLUE_LED);
    gpio_set_dir(BLUE_LED, GPIO_OUT);

    gpio_init(GREEN_LED);
    gpio_set_dir(GREEN_LED, GPIO_OUT);

    gpio_init(BUZZER);
    gpio_set_dir(BUZZER, GPIO_OUT);
}

float medir_distancia() {
    // Envia um pulso de 10us para o sensor
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
    
    // Aguarda o sinal de retorno no pino Echo
    while (gpio_get(ECHO_PIN) == 0);
    uint32_t inicio = time_us_32();
    
    while (gpio_get(ECHO_PIN) == 1);
    uint32_t fim = time_us_32();

    // Calcula a distância em cm
    float duracao = fim - inicio;
    float distancia = (duracao * 0.034) / 2; // Velocidade do som: 0.034 cm/us
    return distancia;
}

void loop() {
    float distancia = medir_distancia(); // Mede a distância
    printf("Distância: %.2f cm\n", distancia);

    // Controla os LEDs e o buzzer com base na distância
    if (distancia < 10 && distancia < 27) {
        alternar_led(1, 0, 0); // Acende LED vermelho
        bit_buzzer(1); // Liga o buzzer
    } else if (distancia < 27) {
        alternar_led(0, 1, 0); // Acende LED azul
        bit_buzzer(0); // Desliga o buzzer
    } else {
        alternar_led(0, 0, 1); // Acende LED verde
        bit_buzzer(0); // Desliga o buzzer
    }

    sleep_ms(500); // Aguarda meio segundo antes da próxima medição
}

int main() {
    stdio_init_all(); // Inicializa a comunicação serial
    setup();          // Configura os pinos
    while (true) {
        loop();       // Executa o loop principal
    }
    return 0;
}
