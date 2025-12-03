#include <stdio.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_timer.h>

#define DIR_PIN GPIO_NUM_8  
#define STEP_PIN GPIO_NUM_22
#define ENC_A_PIN GPIO_NUM_1   
#define ENC_B_PIN GPIO_NUM_2  
#define BUTTON_PIN GPIO_NUM_3 

static unsigned int debounce_time_a = 5000;
static esp_timer_handle_t timer_a = NULL;
static unsigned int debounce_time_b = 5000;
static esp_timer_handle_t timer_b = NULL;
static unsigned int debounce_time_c = 5000;
static esp_timer_handle_t timer_c = NULL;
static bool  last_state = false;
static int current_position = 0;


static int saved_positions[3] = {72, 144, 250};
static int target_position = 0;                  
static bool is_moving_to_position = false;       
static int position_index = 0;                   

void step_motor(bool direction)
{
    gpio_set_level(DIR_PIN, direction);
    gpio_set_level(STEP_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(STEP_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    switch (direction)
    {
        case 1:
            current_position++;
            break;
        case 0:
            current_position--;
            break;
    }
}

void move_to_saved_position(int index) {
    is_moving_to_position = true;
    target_position = saved_positions[index];

    
    int steps_to_move = target_position - current_position;
    bool direction = (steps_to_move > 0);
    
    gpio_set_level(DIR_PIN, direction);
    
    int steps = (steps_to_move > 0) ? steps_to_move : -steps_to_move;
    
    for (int i = 0; i < steps; i++) {
        gpio_set_level(STEP_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(STEP_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        if (direction) {
            current_position++;
        } else {
            current_position--;
        }
    }
    
    is_moving_to_position = false;
}

void IRAM_ATTR gpioISRHandle_a(void*args){
    gpio_intr_disable(ENC_A_PIN);
    esp_timer_start_once(timer_a, debounce_time_a);

    if (is_moving_to_position) {
        return;
    }
    
    bool current_a = gpio_get_level(ENC_A_PIN);
    bool current_b = gpio_get_level(ENC_B_PIN);
    
    if (current_a != last_state){
        if (current_a)
        {
            if (current_b == 0)
            {
                for(int i = 0; i < 10; i++)
                    step_motor(1);
            }
        }
    }
    last_state = current_a;
}

void IRAM_ATTR gpioISRHandle_b(void*args){
    gpio_intr_disable(ENC_B_PIN);
    esp_timer_start_once(timer_b, debounce_time_b);

    if (is_moving_to_position) {
        return;
    }
    
    bool current_a = gpio_get_level(ENC_A_PIN);
    bool current_b = gpio_get_level(ENC_B_PIN);
    
    if (current_a == 0)
    {
        if (current_b == 1)
        {
            for(int i = 0; i < 10; i++)
                step_motor(0);
        } 
    }
}

void IRAM_ATTR gpioISRHandle_c(void*args)
{
    gpio_intr_disable(BUTTON_PIN);
    position_index = (position_index + 1) % 3;  
    esp_timer_start_once(timer_c, debounce_time_c);
}

void debounceTimer_a(void*args){
    gpio_intr_enable(ENC_A_PIN);
}

void debounceTimer_b(void*args){
    gpio_intr_enable(ENC_B_PIN);
}

void debounceTimer_c(void*args){
    move_to_saved_position(position_index);
    gpio_intr_enable(BUTTON_PIN);
}

void init_proc()
{
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(STEP_PIN, 0);

    gpio_set_direction(ENC_A_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(ENC_B_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(ENC_A_PIN);
    gpio_pullup_en(ENC_B_PIN);
    gpio_pullup_en(BUTTON_PIN);

    last_state = gpio_get_level(ENC_A_PIN);

    if((debounce_time_a > 0) && !timer_a){
        esp_timer_create_args_t tmr_cfg;
        tmr_cfg.arg = NULL;
        tmr_cfg.callback = debounceTimer_a;
        tmr_cfg.dispatch_method = ESP_TIMER_TASK;
        tmr_cfg.name = "debounce";
        tmr_cfg.skip_unhandled_events = false;

        esp_timer_create(&tmr_cfg, &timer_a);
    }

    if((debounce_time_b > 0) && !timer_b){
        esp_timer_create_args_t tmr_cfg;
        tmr_cfg.arg = NULL;
        tmr_cfg.callback = debounceTimer_b;
        tmr_cfg.dispatch_method = ESP_TIMER_TASK;
        tmr_cfg.name = "debounce";
        tmr_cfg.skip_unhandled_events = false;

        esp_timer_create(&tmr_cfg, &timer_b);
    }

    if((debounce_time_c > 0) && !timer_c){
        esp_timer_create_args_t tmr_cfg;
        tmr_cfg.arg = NULL;
        tmr_cfg.callback = debounceTimer_c;
        tmr_cfg.dispatch_method = ESP_TIMER_TASK;
        tmr_cfg.name = "debounce";
        tmr_cfg.skip_unhandled_events = false;

        esp_timer_create(&tmr_cfg, &timer_c);
    }
    
    saved_positions[0] = 72; 
    saved_positions[1] = 144; 
    saved_positions[2] = 250; 
}

void app_main(void) {

    init_proc();

    gpio_install_isr_service(0);
    gpio_set_intr_type(ENC_A_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(ENC_A_PIN, gpioISRHandle_a, NULL);

    gpio_set_intr_type(ENC_B_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(ENC_B_PIN, gpioISRHandle_b, NULL);

    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(BUTTON_PIN, gpioISRHandle_c, NULL);

    while(1){
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}