#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <driver/i2c.h>
#include "sdkconfig.h"
#include "HD44780.h"

// https://esp32tutorials.com/esp32-gpio-interrupts-esp-idf/
#define LED_PIN 18
#define BUTTON_PIN 15

// the debounce time of a button in ticks, increase this time if it still chatters
#define DEBOUNCE_TIME 30 

// https://esp32tutorials.com/esp32-pwm-esp-idf-led-brightness-control/
#define LED_PWM_PIN 19

// https://esp32tutorials.com/i2c-lcd-esp32-esp-idf/
#define LCD_ADDR 0x3F
#define SDA_PIN  21
#define SCL_PIN  22
#define LCD_COLS 20
#define LCD_ROWS 4

// interrupt queue
QueueHandle_t interruptQueue = NULL;

// LED control channel
static ledc_channel_config_t ledc_channel;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int gpioNum = (int)args;
    xQueueSendFromISR(interruptQueue, &gpioNum, NULL);
}

static void init_led_pwm(void)
{
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
    int gpioNum, count = 0;

    // variables to keep track of the timing of recent interrupts
    unsigned long buttonTime = 0;  
    unsigned long lastButtonTime = 0;

    while (true)
    {
        if (xQueueReceive(interruptQueue, &gpioNum, portMAX_DELAY))
        {
            buttonTime = xTaskGetTickCount();
            if (buttonTime - lastButtonTime > DEBOUNCE_TIME)
            {
                lastButtonTime = buttonTime;
                printf("GPIO %d was pressed %d times. The state is %d.\n", gpioNum, count++, gpio_get_level(gpioNum));

                toogle_led();
            }
        }
    }
}

static void lcd_task(void* param)
{
    char num[20];
    while (true) {
        LCD_home();
        LCD_clearScreen();
        LCD_writeStr("20x4 I2C LCD");
        vTaskDelay(3000 / portTICK_RATE_MS);
        LCD_clearScreen();
        LCD_writeStr("Lets Count 0-10!");
        vTaskDelay(3000 / portTICK_RATE_MS);
        LCD_clearScreen();
        for (int i = 0; i <= 10; i++) {
            LCD_setCursor(8, 2);
            sprintf(num, "%d", i);
            LCD_writeStr(num);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
  
    }
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

    // * LCD
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    xTaskCreate(lcd_task, "Demo Task", 2048, NULL, 5, NULL);

    // * INTERRUPTS
    // create interrupt queue of 10 elements
    interruptQueue = xQueueCreate(10, sizeof(uint32_t));
    // create a led toogle task
    xTaskCreate(button_press, "Button press", 2048, NULL, 10, NULL);
    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_PIN, gpio_interrupt_handler, (void *)BUTTON_PIN);

    volatile int adc = 0;
    while(true)
    {
        ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, adc);
        ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);

        if(adc>1000) adc = 0;
        adc += 10;

        // vTaskDelay(1000/ portTICK_PERIOD_MS);
        vTaskDelay(100/ portTICK_PERIOD_MS);
    }
}
