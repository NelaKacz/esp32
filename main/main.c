#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define LED_PIN 18
#define BUTTON_PIN 15

#define DEBOUNCE_TIME 20 // the debounce time in ticks, increase this time if it still chatters

// interrupt queue
QueueHandle_t interruptQueue = NULL;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int gpio_num = (int)args;
    xQueueSendFromISR(interruptQueue, &gpio_num, NULL);
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
        if (xQueueReceive(interruptQueue, &gpio_num, portMAX_DELAY))
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

void app_main(void)
{    
    // * LED 
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    // set led pin as input, output (input because we need to read the state of the pin)
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT);

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
    interruptQueue = xQueueCreate(10, sizeof(uint32_t));
    // create a led toogle task
    xTaskCreate(button_press, "Button press", 2048, NULL, 10, NULL);
    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_PIN, gpio_interrupt_handler, (void *)BUTTON_PIN);

    while(true)
    {
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}
