#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"

#define SAMPLE_RATE     (44100)
#define I2S_NUM         (0)
#define I2S_BCK_IO      (GPIO_NUM_26)
#define I2S_WS_IO       (GPIO_NUM_25)
#define I2S_DO_IO       (GPIO_NUM_22)
#define I2S_DI_IO       (-1)

static xQueueHandle i2s_evt_queue = NULL;

void init_i2s() {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16, /* the DAC module will only take the 8bits from MSB */
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 6,
        .dma_buf_len = 60,
        .use_apll = true
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, NULL);
}

static void IRAM_ATTR toggle_button(void *args) {
    int32_t num = 1;
    xQueueSendFromISR(i2s_evt_queue, &num, NULL);
}

void init_gpio() {
    gpio_set_intr_type(GPIO_NUM_32, GPIO_INTR_NEGEDGE);
    gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_NUM_32);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_NUM_32, toggle_button, NULL);
}

void app_main(void)
{
    int32_t num;
    bool play = true;
    i2s_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    init_i2s();
    init_gpio();
    int i = 0;
    while (1) {
        if (xQueueReceive(i2s_evt_queue, &num, portMAX_DELAY)) {
            play = !play;
            if (play) {
                i2s_start(I2S_NUM);
            } else {
                i2s_stop(I2S_NUM);
            }
        }
    }
}
