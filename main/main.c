#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"

// https://esp32tutorials.com/esp32-gpio-interrupts-esp-idf/
#define LED_PIN 18
#define BUTTON_PIN 15

// the debounce time of a button in ticks, increase this time if it still chatters
#define DEBOUNCE_TIME 30 

// https://esp32tutorials.com/esp32-pwm-esp-idf-led-brightness-control/
#define LED_PWM_PIN 19
// PWM LED control channel
static ledc_channel_config_t ledc_channel;

#define POTENTIOMETER_PIN 32
// GPIO32 is ADC1_CH4
static const adc1_channel_t adc_channel = ADC_CHANNEL_4;

// Count of ADC value, from which the average is calculated
#define SAMPLE_CNT 32

// https://github.com/DiegoPaezA/ESP32-freeRTOS/blob/master/i2cPCA9548a/main/i2cPCA9548a.c
#define PCA9548ADDR   0x70  // W   ADDRESS MUX
// PCA9548 CHANNELS
#define  PCA9548_OFF  0x00  // W   CHANNELS OFF
#define  PCA9548_CH1  0x01  // W   ACTIVATION OF CHANNEL 1
#define  PCA9548_CH2  0x02  // W   ACTIVATION OF CHANNEL 2
#define  PCA9548_CH3  0x04  // W   ACTIVATION OF CHANNEL 3
#define  PCA9548_CH4  0x08  // W   ACTIVATION OF CHANNEL 4
#define  PCA9548_CH5  0x10  // W   ACTIVATION OF CHANNEL 5
#define  PCA9548_CH6  0x20  // W   ACTIVATION OF CHANNEL 6
#define  PCA9548_CH7  0x40  // W   ACTIVATION OF CHANNEL 7
#define  PCA9548_CH8  0x80  // W   ACTIVATION OF CHANNEL 8

#define ACK_VAL    0x0
#define NACK_VAL   0x1
#define I2C_MASTER_FREQ_HZ 100000

// // https://esp32tutorials.com/i2c-lcd-esp32-esp-idf/
// #define LCD_ADDR 0x3F
// #define SDA_PIN  21
// #define SCL_PIN  22
// #define LCD_COLS 20
// #define LCD_ROWS 4

// interrupt queues
QueueHandle_t buttonInterruptQueue = NULL;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int gpioNum = (int)args;
    if (gpioNum == BUTTON_PIN)
    {
        xQueueSendFromISR(buttonInterruptQueue, &gpioNum, NULL);
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
    int gpioNum, count = 0;

    // variables to keep track of the timing of recent interrupts
    unsigned long buttonTime = 0;  
    unsigned long lastButtonTime = 0;

    while (true)
    {
        if (xQueueReceive(buttonInterruptQueue, &gpioNum, portMAX_DELAY))
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

void init_i2c()
{
    // configure the i2c controller 0 in master mode, normal speed
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = 21;
	conf.scl_io_num = 22;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ; //100000
	conf.clk_flags = 0; //(V4.4)  is 0, the clock allocator will select only according to the desired frequency.
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	printf("I2C controller configured\r\n");

	// install the driver
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	printf("I2C driver installed\r\n\r\n");

}

void selectI2CChannel(int muxADD, int channelADD) 
{
	// create and execute the command link
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,(muxADD << 1) | I2C_MASTER_WRITE,true);
	i2c_master_write_byte(cmd,channelADD,true);
	i2c_master_stop(cmd);
	if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS) == ESP_OK) 
    {
		printf("Selecting channel 0x%02x of Mux I2C \r\n", channelADD);
	}
	i2c_cmd_link_delete(cmd);

}

void i2c_scan(void){
	int devices_found = 0;
		for(int address = 1; address < 127; address++) {

			// create and execute the command link
			i2c_cmd_handle_t cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
			i2c_master_stop(cmd);
			if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS) == ESP_OK) {
				printf("-> found device with address 0x%02x\r\n", address);
				devices_found++;
			}
			i2c_cmd_link_delete(cmd);
		}
		if(devices_found == 0) printf("\r\n-> no devices found\r\n");
		printf("\r...scan completed!\r\n");
}

// static void lcd_task(void* param)
// {
//     char num[20];
//     while (true) {
//         LCD_home();
//         LCD_clearScreen();
//         LCD_writeStr("20x4 I2C LCD");
//         vTaskDelay(3000 / portTICK_RATE_MS);
//         LCD_clearScreen();
//         LCD_writeStr("Lets Count 0-10!");
//         vTaskDelay(3000 / portTICK_RATE_MS);
//         LCD_clearScreen();
//         for (int i = 0; i <= 10; i++) {
//             LCD_setCursor(8, 2);
//             sprintf(num, "%d", i);
//             LCD_writeStr(num);
//             vTaskDelay(1000 / portTICK_RATE_MS);
//         }
//     }
// }

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
    // LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    // xTaskCreate(lcd_task, "Demo Task", 2048, NULL, 5, NULL);

    // * INTERRUPTS
    // create interrupt queue of 10 elements
    buttonInterruptQueue = xQueueCreate(10, sizeof(uint32_t));
    // create a button press task
    xTaskCreate(button_press, "Button press", 2048, NULL, 10, NULL);
    // install gpio isr service
    gpio_install_isr_service(0);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_PIN, gpio_interrupt_handler, (void *)BUTTON_PIN);

    init_i2c();
    esp_log_level_set("I2C", ESP_LOG_INFO);
	i2c_scan();
 	selectI2CChannel(PCA9548ADDR,PCA9548_CH7);
    while(true)
    {
        sample_adc1();

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
