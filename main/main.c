#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// https://esp32tutorials.com/esp32-gpio-interrupts-esp-idf/
#define LED_PIN 12
#define BUTTON_PIN 15

// the debounce time of a button in ticks, increase this time if it still chatters
#define DEBOUNCE_TIME 30 

// https://esp32tutorials.com/esp32-pwm-esp-idf-led-brightness-control/
#define LED_PWM_PIN 13
// PWM LED control channel
static ledc_channel_config_t ledc_channel;
#define POTENTIOMETER_PIN 32
// GPIO32 is ADC1_CH4
static const adc1_channel_t adc_channel = ADC_CHANNEL_4;
// Count of ADC value, from which the average is calculated
#define SAMPLE_CNT 32

// interrupt queue
QueueHandle_t button_interrupt_queue = NULL;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int gpio_num = (int)args;
    if (gpio_num == BUTTON_PIN)
    {
        xQueueSendFromISR(button_interrupt_queue, &gpio_num, NULL);
    }
}

static void init_led_pwm(void)
{
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
    
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&ledc_timer);
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = LED_PWM_PIN;
    ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&ledc_channel);
}

static void toogle_led()
{
    printf("Toogle led to %d.\n", !gpio_get_level(LED_PIN));
    gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
}

static void button_press(void *params)
{
    int gpio_num, count = 0;

    // variables to keep track of the timing of recent interrupts
    unsigned long button_time = 0;  
    unsigned long last_button_time = 0;

    while (true)
    {
        if (xQueueReceive(button_interrupt_queue, &gpio_num, portMAX_DELAY))
        {
            button_time = xTaskGetTickCount();
            if (button_time - last_button_time > DEBOUNCE_TIME)
            {
                last_button_time = button_time;
                printf("GPIO %d was pressed %d times. The state is %d.\n", gpio_num, count++, gpio_get_level(gpio_num));

                toogle_led();
            }
        }
    }
}

static void sample_adc1()
{
    uint32_t adc = 0;
    for (int i = 0; i < SAMPLE_CNT; ++i)
    {
        adc += adc1_get_raw(adc_channel);
    }
    adc /= SAMPLE_CNT;

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, adc);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

void app_main(void)
{    
    // * LED 
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    // set led pin as input, output (input because we need to read the state of the pin)
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT);

    // * LED PWM
    init_led_pwm();

    // * BUTTON 
    esp_rom_gpio_pad_select_gpio(BUTTON_PIN);
    // set button pin as input
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    // set button to pull-up mode
    // gpio_set_level(BUTTON_PIN, 1);
    // disable pulldown resistor
    gpio_pulldown_dis(BUTTON_PIN);
    // enable pullup resistor
    gpio_pullup_en(BUTTON_PIN);
    // set gpio interrupt for button on falling edge
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
    // enable interrupt on the pin
    gpio_intr_enable(BUTTON_PIN);

    // * INTERRUPTS
    // create interrupt queue of 10 elements
    button_interrupt_queue = xQueueCreate(10, sizeof(uint32_t));
    // create a button press task
    xTaskCreate(button_press, "Button press", 2048, NULL, 10, NULL);
    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_PIN, gpio_interrupt_handler, (void *)BUTTON_PIN);

    while(true)
    {
        // Continuously sample ADC1
        sample_adc1();

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
