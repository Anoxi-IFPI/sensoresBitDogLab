#include <stdio.h>    // Para printf, snprintf
#include <string.h>   // Para memset
#include <stdlib.h>   // Para malloc, free (usado por algumas libs, como a OLED)
#include "pico/stdlib.h"  // Funções padrão do Pico SDK (sleep_ms, stdio_init_all)
#include "hardware/i2c.h" // Funções de hardware I2C
#include "hardware/gpio.h" // Funções de hardware GPIO (para pinos I2C)
#include "pico/time.h"    // Funções de tempo (get_absolute_time, to_ms_since_boot)

// Inclua cabeçalhos da sua biblioteca portada do sensor MAX30105
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h" // Garanta que FreqS aqui seja 50 para o algoritmo

// Inclua cabeçalhos do display OLED
// Estes cabeçalhos devem estar na pasta 'inc'
#include "inc/ssd1306.h"       // Contém as definições para o driver SSD1306
#include "inc/ssd1306_font.h"  // Contém as definições de fontes (se usadas diretamente)

// =========================================================================
// DEFINIÇÕES DE HARDWARE E SENSORES/DISPLAY
// =========================================================================
// Configuração I2C para o Sensor MAX30105 (usando I2C0)
#define SENSOR_I2C_PORT    i2c0       
#define SENSOR_SDA_PIN     0       
#define SENSOR_SCL_PIN     1       
#define SENSOR_I2C_FREQ    400000   // 400 kHz para comunicação I2C rápida

// Configuração I2C para o Display OLED (usando I2C1, como no seu exemplo AHT10)
#define OLED_I2C_PORT      i2c1
#define OLED_SDA_PIN       14
#define OLED_SCL_PIN       15
#define OLED_I2C_FREQ      400000

// Endereço I2C do display OLED (VERIFIQUE O ENDEREÇO CORRETO DO SEU MÓDULO OLED!)
// O mais comum é 0x3C ou 0x3D.
#define OLED_I2C_ADDRESS   0x3C 

// Taxa de amostragem para o sensor MAX30105
// É crucial que esta taxa (SENSOR_SAMPLE_RATE) corresponda ao FreqS definido em spo2_algorithm.h.
// Se FreqS em spo2_algorithm.h é 50, então aqui também.
#define SENSOR_SAMPLE_RATE 50 // Hz

// Limite mínimo para os valores brutos de IR e Red.
// Se as leituras estiverem abaixo deste valor, indica que o dedo não está bem posicionado
// ou há muita luz ambiente, e o sinal não é válido para cálculo.
#define MIN_VALID_RAW_VALUE 1000 // Valor empírico, pode precisar de ajuste

// =========================================================================
// VARIÁVEIS GLOBAIS DA BIBLIOTECA OLED (DECLARAÇÕES EXTERNAS)
// Estas variáveis são definidas e gerenciadas pela sua biblioteca SSD1306 (ssd1306.c, ssd1306_i2c.c).
// Precisamos declará-las como 'extern' para usá-las aqui.
// =========================================================================
extern struct render_area frame_area; // Estrutura para definir a área de atualização do OLED
extern uint8_t ssd_buffer[ssd1306_buffer_length]; // Buffer de pixels da tela do OLED

// Variáveis de dimensão do OLED (definidas como macros ou consts em ssd1306.h/ssd1306_i2c.h)
// Não precisam ser declaradas 'extern' se são macros. Se forem variáveis, precisariam.
// Como o erro anterior indicou que eram macros, vamos confiar nisso.
// extern uint16_t ssd1306_width; // Removida conforme análise anterior, se são macros.
// extern uint16_t ssd1306_height;
// extern uint16_t ssd1306_n_pages;
// extern uint16_t ssd1306_buffer_length;

// Funções da biblioteca OLED (declaradas em ssd1306.h, implementadas em ssd1306.c/ssd1306_i2c.c)
extern void ssd1306_init(); // Nota: Sua biblioteca usa esta função sem parâmetros I2C
extern void render_on_display(uint8_t *buffer, struct render_area *render_area);
extern void calculate_render_area_buffer_length(struct render_area *area);
// Corrigida a assinatura para aceitar 'const char*'
extern void ssd1306_draw_string(uint8_t *ssd, int16_t x, int16_t y, const char *string);


// =========================================================================
// INSTÂNCIAS DE OBJETOS E VARIÁVEIS DO SENSOR MAX30105
// =========================================================================
MAX30105 particleSensor; // Instância do objeto sensor MAX30105

// Buffers para armazenar as amostras brutas de IR e Red do sensor
// BUFFER_SIZE é definido em spo2_algorithm.h (geralmente 100 ou 200).
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];

// Variáveis para armazenar os resultados calculados pelo algoritmo
int32_t spo2 = 0;            // Saturação de Oxigênio (0-100%)
int8_t spo2_valid = 0;       // Flag: 1 se SpO2 é válido, 0 caso contrário
int32_t heartRate = 0;       // Frequência Cardíaca (Batimentos Por Minuto)
int8_t heartRate_valid = 0;  // Flag: 1 se HR é válido, 0 caso contrário

int bufferIdx = 0; // Índice para gerenciar o preenchimento circular dos buffers irBuffer/redBuffer

// =========================================================================
// FUNÇÕES AUXILIARES PARA O OLED (WRAPPERS)
// Estas funções simplificam as chamadas para as funções da biblioteca SSD1306
// usando o buffer global 'ssd_buffer' e a área de renderização 'frame_area'.
// =========================================================================

/**
 * @brief Inicializa o display OLED e seus parâmetros I2C.
 * Esta função configura o barramento I2C que o OLED utilizará
 * e chama a função de inicialização do driver SSD1306.
 */
void init_oled() {
    // Inicializa a interface I2C para o OLED.
    i2c_init(OLED_I2C_PORT, OLED_I2C_FREQ);
    // Define as funções GPIO para SDA e SCL como I2C.
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    // Habilita os pull-ups internos. Importante para I2C.
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);

    // Chama a função de inicialização da biblioteca SSD1306.
    // Baseado no seu exemplo, ssd1306_init() não recebe parâmetros I2C.
    // A configuração de endereço I2C e porta deve ser feita via macros
    // em inc/ssd1306.h ou inc/ssd1306_i2c.h.
    ssd1306_init(); 
    
    // Configura a área de renderização para cobrir a tela inteira.
    frame_area.start_column = 0;
    frame_area.end_column = ssd1306_width - 1; // ssd1306_width é uma macro/global da lib
    frame_area.start_page = 0;
    frame_area.end_page = ssd1306_n_pages - 1; // ssd1306_n_pages é uma macro/global da lib
    
    // Calcula o comprimento do buffer necessário para a área de renderização.
    calculate_render_area_buffer_length(&frame_area);
    
    // Limpa o buffer de pixels na RAM antes de enviar para o display.
    memset(ssd_buffer, 0, ssd1306_buffer_length);

    // Pequeno atraso para estabilização.
    sleep_ms(100);
    printf("Display OLED inicializado no I2C%d (SDA:GP%d, SCL:GP%d).\n", 
           (OLED_I2C_PORT == i2c0 ? 0 : 1), OLED_SDA_PIN, OLED_SCL_PIN);
}

/**
 * @brief Limpa o conteúdo do buffer de pixels do OLED e atualiza o display.
 */
void clear_oled_display() {
    memset(ssd_buffer, 0, ssd1306_buffer_length); // Preenche o buffer com zeros (pixels desligados)
    render_on_display(ssd_buffer, &frame_area);   // Envia o buffer vazio para a tela
}

/**
 * @brief Desenha uma mensagem de texto no buffer do OLED em uma linha específica.
 * @param message A string de texto a ser exibida.
 * @param line A linha (baseada em 0) onde o texto será exibido (cada linha tem 8 pixels de altura).
 */
void display_message_oled(const char *message, int line) {
    // Desenha a string no buffer de pixels 'ssd_buffer'.
    // A posição X inicial é 5 pixels. A posição Y é calculada pela linha * 8 pixels/linha.
    ssd1306_draw_string(ssd_buffer, 5, line * 8, message); 
}

// =========================================================================
// FUNÇÃO PRINCIPAL DO PROGRAMA
// =========================================================================

int main() {
    // Inicializa todos os periféricos padrão do Pico SDK (incluindo USB para stdio).
    stdio_init_all();
    printf("Iniciando sistema oxímetro/HR com RP2040 e OLED...\n");

    // 1. Inicializa o barramento I2C para o sensor MAX30105.
    i2c_init(SENSOR_I2C_PORT, SENSOR_I2C_FREQ);
    gpio_set_function(SENSOR_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SENSOR_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SENSOR_SDA_PIN);
    gpio_pull_up(SENSOR_SCL_PIN);
    printf("I2C0 para sensor (SDA=GP%d, SCL=GP%d) @ %d Hz\n", SENSOR_SDA_PIN, SENSOR_SCL_PIN, SENSOR_I2C_FREQ);

    // 2. Inicializa o display OLED.
    init_oled();
    clear_oled_display(); // Limpa a tela
    display_message_oled("Iniciando...", 0); // Exibe mensagem na linha 0
    display_message_oled("Oximetro & HR", 1); // Exibe mensagem na linha 1
    render_on_display(ssd_buffer, &frame_area); // Envia o buffer atualizado para o OLED
    sleep_ms(2000); // Exibe a mensagem por 2 segundos
    clear_oled_display(); // Limpa a tela novamente

    // 3. Inicializa o sensor MAX30105.
    // O método begin() da sua biblioteca MAX30105 espera a instância I2C do Pico SDK.
    if (!particleSensor.begin(SENSOR_I2C_PORT, MAX30105_ADDRESS)) {
        printf("ERRO: MAX30105 não encontrado. Verifique conexões (VCC, GND, SDA, SCL).\n");
        clear_oled_display();
        display_message_oled("ERRO: Sensor", 0);
        display_message_oled("MAX30105", 1);
        display_message_oled("nao encontrado!", 2);
        render_on_display(ssd_buffer, &frame_area);
        while (1) { sleep_ms(1000); } // Loop infinito para erro crítico
    }
    printf("MAX30105 encontrado e pronto.\n");

    // 4. Configurações do sensor MAX30105.
    // powerLevel: Intensidade do LED (0x7F = 25.4mA, 0xFF = 50mA para sinais mais fortes).
    // sampleAverage: Média de 8 amostras para reduzir ruído.
    // ledMode: 2 para Red+IR (ideal para oximetria).
    // sampleRate: 50 Hz (deve coincidir com FreqS em spo2_algorithm.h).
    // pulseWidth: 411us para maior resolução de ADC.
    // adcRange: 4096.
    particleSensor.setup(0x7F, 8, 2, SENSOR_SAMPLE_RATE, 411, 4096); 
    printf("Sensor configurado.\n");
    
    // Mensagem inicial de instrução no OLED.
    clear_oled_display();
    display_message_oled("Posicione o dedo", 0);
    display_message_oled("e cubra o sensor!", 1);
    render_on_display(ssd_buffer, &frame_area);
    sleep_ms(2000);
    clear_oled_display();

    // =========================================================================
    // LOOP PRINCIPAL DE LEITURA E PROCESSAMENTO
    // =========================================================================
    while (1) {
        // Verifica quantas novas amostras estão disponíveis no FIFO do sensor.
        // `particleSensor.check()` lê os dados do FIFO do sensor para uma estrutura interna.
        uint16_t num_new_samples = particleSensor.check();

        if (num_new_samples > 0) {
            bool current_sample_valid = true; // Flag para verificar a validade das amostras lidas

            // Processa todas as novas amostras disponíveis.
            for (int i = 0; i < num_new_samples; i++) {
                // Obtém as leituras brutas de IR e Red.
                uint32_t currentIR = particleSensor.getFIFOIR();
                uint32_t currentRed = particleSensor.getFIFORed();
                
                // Verifica se as amostras brutas estão abaixo de um limite mínimo.
                // Isso ajuda a filtrar casos onde não há dedo ou há muita luz ambiente.
                if (currentIR < MIN_VALID_RAW_VALUE || currentRed < MIN_VALID_RAW_VALUE) {
                    current_sample_valid = false; // Marca o sinal como inválido para este ciclo
                }

                // Armazena as amostras nos buffers circulares locais.
                irBuffer[bufferIdx] = currentIR;
                redBuffer[bufferIdx] = currentRed;
                
                // Avança o ponteiro de leitura do FIFO do sensor.
                particleSensor.nextSample(); 
                
                // Avança o índice do nosso buffer local (circular).
                bufferIdx = (bufferIdx + 1) % BUFFER_SIZE;
            }

            // Apenas executa o algoritmo se as amostras foram válidas no ciclo atual
            // e se o buffer estiver preenchido o suficiente para o algoritmo (pelo menos uma vez).
            if (current_sample_valid && bufferIdx >= BUFFER_SIZE - 1) { 
                // Chama a função do algoritmo para calcular SpO2 e Frequência Cardíaca.
                // Os buffers 'irBuffer' e 'redBuffer' contêm as últimas 'BUFFER_SIZE' amostras.
                maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer, &spo2, &spo2_valid, &heartRate, &heartRate_valid);

                // Imprime os resultados no monitor serial para depuração.
                printf("-------------------------------------------\n");
                printf("Amostras processadas neste ciclo: %d\n", num_new_samples);
                printf("Posicao atual no buffer: %d\n", bufferIdx);
                
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

                // =========================================================================
                // EXIBIÇÃO DOS RESULTADOS NO DISPLAY OLED
                // =========================================================================
                clear_oled_display(); // Limpa a tela antes de desenhar o novo frame

                char text_buffer[32]; // Buffer temporário para formatar strings

                // Exibe a Frequência Cardíaca no OLED
                if (heartRate_valid) {
                    snprintf(text_buffer, sizeof(text_buffer), "BPM: %ld", heartRate);
                    display_message_oled(text_buffer, 0); // Linha 0
                } else {
                    display_message_oled("BPM: ---", 0);
                    display_message_oled("Aguardando...", 1);
                }

                // Exibe a Saturação de Oxigênio no OLED
                if (spo2_valid) {
                    snprintf(text_buffer, sizeof(text_buffer), "SpO2: %ld%%", spo2);
                    display_message_oled(text_buffer, 3); // Linha 3
                } else {
                    display_message_oled("SpO2: ---", 3);
                    display_message_oled("Aguardando...", 4);
                }

                render_on_display(ssd_buffer, &frame_area); // Atualiza o display com os novos dados
                // =========================================================================

            } else { // Se o sinal não foi válido (current_sample_valid é false), exibe aviso no OLED
                 printf("AVISO: Sinal fraco/ausente. Posicione o dedo corretamente e evite luz ambiente.\n");
                 clear_oled_display();
                 display_message_oled("SINAL FRACO!", 0);
                 display_message_oled("Reposicione dedo", 1);
                 display_message_oled("E EVITE LUZ!", 2);
                 render_on_display(ssd_buffer, &frame_area);
            }
        }
        
        sleep_ms(20); // Pequeno atraso para controlar a taxa de loop (50 Hz do sensor = 20ms por amostra)
    }

    return 0; // O loop infinito garante que nunca chegaremos aqui.
}