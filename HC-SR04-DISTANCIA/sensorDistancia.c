#include <stdio.h>
#include "pico/stdlib.h"

#define TRIG_PIN 9
#define ECHO_PIN 8

#define LED_RED 13
#define LED_GREEN 11
#define LED_YELLOW 12 // Simula com ambos ligados (verde + vermelho)

// Inicializa GPIOs do LED
void setup_leds() {
    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);
    
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    
    gpio_init(LED_YELLOW);
    gpio_set_dir(LED_YELLOW, GPIO_OUT);
}

// Liga apenas um LED por vez (verde, amarelo ou vermelho)
void set_led_color(const char* color) {
    if (color == "green") {
        gpio_put(LED_GREEN, 1);
        gpio_put(LED_RED, 0);
        gpio_put(LED_YELLOW, 0);
        printf("verde\n");
    } else if (color == "yellow") {
        gpio_put(LED_GREEN, 0);
        gpio_put(LED_RED, 0);
        gpio_put(LED_YELLOW, 1);
        printf("amerelo\n");
    } else if (color == "red") {
        gpio_put(LED_GREEN, 0);
        gpio_put(LED_RED, 1);
        gpio_put(LED_YELLOW, 0);
        printf("vermelho\n");
    } else {
        // Apaga todos
        gpio_put(LED_GREEN, 0);
        gpio_put(LED_RED, 0);
        gpio_put(LED_YELLOW, 0);
    }
}

float medir_distancia_cm() {
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    while (gpio_get(ECHO_PIN) == 0);
    absolute_time_t start_time = get_absolute_time();

    while (gpio_get(ECHO_PIN) == 1);
    absolute_time_t end_time = get_absolute_time();

    int64_t duration_us = absolute_time_diff_us(start_time, end_time);
    float distance_cm = (duration_us / 2.0f) / 29.1f;
    return distance_cm;
}

int main() {
    stdio_init_all();

    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    setup_leds();

    while (true) {
        float distancia = medir_distancia_cm();
        printf("Distância: %.2f cm\n", distancia);

        if (distancia > 100.0f) {
            set_led_color("green");
        } else if (distancia > 50.0f) {
            set_led_color("azul");
        } else {
            set_led_color("red");
        }

        sleep_ms(1000);
    }
}
