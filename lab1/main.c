#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

static esp_timer_handle_t timer = NULL;
static uint32_t debounce_time = 10000;
static uint32_t last_state = 0;
static uint32_t counter = 0;
static uint32_t last_count = -1;


static void debounceTimer(void* args){
    gpio_intr_disable(GPIO_NUM_5);
    uint32_t new_state = gpio_get_level(GPIO_NUM_5);

    if(new_state == 0 && last_state == 1){
        counter++;
        if(last_count != counter){
            ESP_LOGW("MAIN", "Counter = %d", counter);
        }
        else{
            last_count = counter;
        }
    }
    else if(new_state == 1){
        last_state = 1;
    }

    gpio_intr_enable(GPIO_NUM_5);

}

static void gpioISRHandle(void* args){
    gpio_intr_disable(GPIO_NUM_5);
    esp_timer_start_once(timer, debounce_time);
}

void app_main(void)
{
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_NUM_5);
    gpio_pulldown_dis(GPIO_NUM_5);

    if((debounce_time > 0) && !timer){
        esp_timer_create_args_t tmr_cfg;
        tmr_cfg.arg = NULL;
        tmr_cfg.callback = debounceTimer;
        tmr_cfg.dispatch_method = ESP_TIMER_TASK;
        tmr_cfg.name = "debounce";
        tmr_cfg.skip_unhandled_events = false;

        esp_timer_create(&tmr_cfg, &timer);
    }

    gpio_set_intr_type(GPIO_NUM_5, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_5, gpioISRHandle, NULL);
    gpio_intr_enable(GPIO_NUM_5);

    last_state = gpio_get_level(GPIO_NUM_5);

}