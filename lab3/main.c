#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "esp_timer.h"

#define I2C_MASTER_SDA_IO 9
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_FREQ_HZ 40000
#define I2C_MASTER_NUM I2C_NUM_0  

#define LED_GPIO 3
#define LED_NUM 64
#define LED_MATRIX_SIZE 8

typedef struct {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} gyro_type_t;

typedef struct {
    float x;
    float y;
} ball_position_t;

static ball_position_t ball = {3.5f, 3.5f};
static gyro_type_t gyro_data;
static uint8_t led_matrix[LED_NUM * 3];
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

static void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_write_reg(uint8_t addr, uint8_t data) {
    uint8_t buf[2] = {addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, 0x68, buf, sizeof(buf), pdMS_TO_TICKS(1000));
}

static esp_err_t mpu6050_read_reg(uint8_t addr, uint8_t *data, uint8_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, 0x68, &addr, 1, data, len, pdMS_TO_TICKS(1000));
}

static void mpu6050_init() {
    mpu6050_write_reg(0x6B, 0x00);
}

static esp_err_t mpu6050_read_gyro(gyro_type_t *data) {
    uint8_t buf[6];
    esp_err_t err = mpu6050_read_reg(0x43, buf, sizeof(buf));

    data->gyro_x = (buf[0] << 8) | buf[1];
    data->gyro_y = (buf[2] << 8) | buf[3];
    data->gyro_z = (buf[4] << 8) | buf[5];

    return err;
}

static void ws2812_init() {
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = 10000000,
        .trans_queue_depth = 4,
        .flags.with_dma = false,
    };
    rmt_new_tx_channel(&tx_chan_config, &led_chan);

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 3,
            .level1 = 0,
            .duration1 = 9,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 9,
            .level1 = 0,
            .duration1 = 3
        },
    };
    rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder);
    rmt_enable(led_chan);
}

static void set_pixel_color(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
    if (x >= LED_MATRIX_SIZE || y >= LED_MATRIX_SIZE) return;
    
    uint16_t index;
    if (y % 2 == 0) {
        index = y * LED_MATRIX_SIZE + x;
    } else {
        index = y * LED_MATRIX_SIZE + (LED_MATRIX_SIZE - 1 - x);
    }
    
    led_matrix[index * 3 + 0] = g;
    led_matrix[index * 3 + 1] = r;
    led_matrix[index * 3 + 2] = b;
}

static void clear_matrix() {
    memset(led_matrix, 0, sizeof(led_matrix));
}

static void update_matrix() {
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    rmt_transmit(led_chan, led_encoder, led_matrix, sizeof(led_matrix), &tx_config);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void draw_ball() {
    int ball_x = (int)ball.x;
    int ball_y = (int)ball.y;

    set_pixel_color(ball.x, ball.y, 0, 0, 255);
    int neighbor[3][2] = {{0, 1}, {1, 0}, {1, 1}};

    for (int i = 0; i < 3; i++) {
        int px = ball_x + neighbor[i][0];
        int py = ball_y + neighbor[i][1];
        
        if (px >= 0 && px < LED_MATRIX_SIZE && py >= 0 && py < LED_MATRIX_SIZE) {
            set_pixel_color(px, py, 0, 0, 255);
        }
    }
}


static void update_ball_position() {
    float sensitivity = 0.0001f;
    
    ball.x += gyro_data.gyro_y * sensitivity;
    ball.y += gyro_data.gyro_x * sensitivity;
    
    if (ball.x < 0.5f) ball.x = 0;
    if (ball.x > LED_MATRIX_SIZE - 1.5f) ball.x = LED_MATRIX_SIZE - 1.5f;
    if (ball.y < 0.5f) ball.y = 0;
    if (ball.y > LED_MATRIX_SIZE - 1.5f) ball.y = LED_MATRIX_SIZE - 1.5f;
}

static void mpu6050_task() {
    while (true) {
        if (mpu6050_read_gyro(&gyro_data) == ESP_OK) {
            printf("X=%d, Y=%d, Z=%d\n", gyro_data.gyro_x, gyro_data.gyro_y, gyro_data.gyro_z);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void ball_animation_task() {
    while (true) {
        update_ball_position();  
        clear_matrix();
        draw_ball();
        update_matrix();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void) {

    i2c_init();
    mpu6050_init();
    ws2812_init();
        
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 2, NULL);
    xTaskCreate(ball_animation_task, "ball_animation_task", 4096, NULL, 2, NULL);
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}