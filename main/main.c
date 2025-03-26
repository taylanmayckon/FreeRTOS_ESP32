#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include "driver/ledc.h"
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"

#define LED_RED GPIO_NUM_5 // GPIO 
#define LED_GREEN GPIO_NUM_17 // GPIO
#define LED_BLUE GPIO_NUM_16 // PWM

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0 
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Resolução do PWM a ser gerado (VALOR DO TOP)
#define TOP_PWM 8191 // Definindo o TOP do PWM 
#define LEDC_FREQUENCY (4000) // Freq do PWM em Hz

void GPIO_init(){
    gpio_config_t leds = {
        .pin_bit_mask = (1<<LED_RED) | (1<<LED_GREEN), // BITMASK para os pinos de output
        .mode = GPIO_MODE_DEF_OUTPUT, // Definindo como output
        .pull_up_en = GPIO_PULLUP_DISABLE, // Sem pullup
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Sem pulldown
        .intr_type = GPIO_INTR_DISABLE, // Sem interrupções
    };

    ESP_ERROR_CHECK(gpio_config(&leds)); // Iniciando a configuração e verificando erros

    // Configurando os pinos como output
    gpio_set_direction(LED_RED, GPIO_MODE_DEF_OUTPUT); 
    gpio_set_direction(LED_GREEN, GPIO_MODE_DEF_OUTPUT);
}

void PWM_init(int gpio_number){
    // Configurando o timer do LEDC PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE, 
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configurando o canal PWM a ser usado
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio_number,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}



void TaskBlink(){
    GPIO_init();
    bool ledState = true;

    while(true){
        gpio_set_level(LED_RED, ledState);
        gpio_set_level(LED_GREEN, !ledState);

        ledState = !ledState;
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void TaskPWM(){
    PWM_init(LED_BLUE);
    float duty_percent = 0.0;
    bool rising = true;

    while(true){
        // Ajustando se vai incrementar ou decrementar o Duty Cycle
        if(rising){ // Aumenta de 0% até 100%
            duty_percent+=0.01;
            if(duty_percent >= 1.0f){
                rising=false;
            }
        }
        else{ // Reduz de 100% para 0%
            duty_percent-=0.01;
            if(duty_percent<=0.0f){
                rising=true;
            }
        }

        // Atualizando o valor do PWM
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, TOP_PWM*duty_percent));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}


void app_main(void){
    xTaskCreate(&TaskBlink, // Endereço da Task da GPIO
                "blink", // Identificador da task
                4096, // Bytes disponibilizados para a Task
                NULL, // Se for enviar parâmetro para as tasks, é aqui que coloca
                1, // Prioridade (1 a 5), min -> max
                NULL); // Handler

    xTaskCreate(&TaskPWM, // Endereço da Task do PWN
                "pwm", // Identificador da task
                4096, // Bytes disponibilizados para a Task
                NULL, // Se for enviar parâmetro para as tasks, é aqui que coloca
                5, // Prioridade (1 a 5), min -> max
                NULL); // Handler
}