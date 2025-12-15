#include <driver/adc.h>
#include <driver/gptimer.h>
#include <math.h>
#include <atomic>

#define MIC_PIN ADC1_CHANNEL_6
#define LED_PIN 2
#define SAMPLE_RATE_HZ 2000
#define LOW_CUTOFF_FREQ 200.0f
#define HIGH_CUTOFF_FREQ 400.0f
#define THRESHOLD_AMPLITUDE 10
#define SAMPLE_PERIOD_MS (1000.0f / SAMPLE_RATE_HZ)

typedef struct {
    float a0, a1, a2;
    float b1, b2;
    float x1, x2;
    float y1, y2;
} BiquadFilter;

typedef struct {
    BiquadFilter filter;
    std::atomic<float> amplitude;
    std::atomic<bool> led_state;
    std::atomic<uint32_t> sample_count;
} AudioProcessor;

AudioProcessor processor;
gptimer_handle_t gptimer = NULL;

void calculate_bandpass_coeffs(BiquadFilter* filter, float freq, float Q, float fs) {
    float w0 = 2.0f * M_PI * freq / fs;
    float sin_w0 = sin(w0);
    float cos_w0 = cos(w0);
    float alpha = sin_w0 / (2.0f * Q);
    
    float a0_inv = 1.0f / (1.0f + alpha);
    
    filter->a0 = (alpha) * a0_inv;
    filter->a1 = 0;
    filter->a2 = (-alpha) * a0_inv;
    filter->b1 = (-2.0f * cos_w0) * a0_inv;
    filter->b2 = (1.0f - alpha) * a0_inv;
    
    filter->x1 = 0;
    filter->x2 = 0;
    filter->y1 = 0;
    filter->y2 = 0;
}

float apply_biquad_filter(BiquadFilter* filter, float input) {
    float output = filter->a0 * input + 
                   filter->a1 * filter->x1 + 
                   filter->a2 * filter->x2 - 
                   filter->b1 * filter->y1 - 
                   filter->b2 * filter->y2;
    
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    
    return output;
}

static bool IRAM_ATTR onTimerCallback(gptimer_handle_t timer, 
                                      const gptimer_alarm_event_data_t *edata, 
                                      void *user_data) {
    int adc_value = adc1_get_raw(MIC_PIN);
    
    float input_signal = ((float)adc_value - 2048.0f) / 2048.0f;
    float filtered = apply_biquad_filter(&processor.filter, input_signal);
    float amplitude_scaled = fabs(filtered) * 1000.0f;
    
    processor.amplitude.store(amplitude_scaled);
    
    bool should_led_be_on = amplitude_scaled > THRESHOLD_AMPLITUDE;
    processor.led_state.store(should_led_be_on);
    
    processor.sample_count.store(processor.sample_count.load() + 1);
    
    return true;
}

void setup() {
    Serial.begin(115200);
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    float center_freq = sqrt(LOW_CUTOFF_FREQ * HIGH_CUTOFF_FREQ);
    float bandwidth = HIGH_CUTOFF_FREQ - LOW_CUTOFF_FREQ;
    float Q = center_freq / bandwidth;
    
    calculate_bandpass_coeffs(&processor.filter, center_freq, Q, SAMPLE_RATE_HZ);
    
    processor.amplitude = 0;
    processor.led_state = false;
    processor.sample_count = 0;
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MIC_PIN, ADC_ATTEN_DB_11);
    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    
    gptimer_new_timer(&timer_config, &gptimer);
    
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000 / SAMPLE_RATE_HZ,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);
    
    gptimer_event_callbacks_t cbs = {
        .on_alarm = onTimerCallback,
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);
    
    gptimer_enable(gptimer);
    gptimer_start(gptimer);
}

void loop() {
    static uint32_t last_print_time = 0;
    static uint32_t last_sample_count = 0;
    static bool last_led_state = false;
    
    uint32_t current_time = millis();
    uint32_t current_sample_count = processor.sample_count.load();
    
    bool current_led_state = processor.led_state.load();
    if (current_led_state != last_led_state) {
        digitalWrite(LED_PIN, current_led_state ? HIGH : LOW);
        last_led_state = current_led_state;
    }
    
    if (current_time - last_print_time >= 500 || current_sample_count != last_sample_count) {
        float amp = processor.amplitude.load();
        bool led_on = processor.led_state.load();
        
        Serial.printf("Амплитуда (200-400 Гц): %6.1f | Светодиод: %s\n", 
                     amp, 
                     led_on ? "ВКЛ" : "ВЫКЛ");
        
        last_print_time = current_time;
        last_sample_count = current_sample_count;
    }
    
    delay(10);
}