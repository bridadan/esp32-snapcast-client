#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"

#define SAMPLE_RATE     (44100)
#define I2S_NUM         (0)
#define I2S_BCK_IO      (GPIO_NUM_14)
#define I2S_WS_IO       (GPIO_NUM_25)
#define I2S_DO_IO       (GPIO_NUM_26)
#define I2S_DI_IO       (-1)

#define NUM_CHANNELS 2
#define BITS_PER_SAMPLE 16
#define SIZE_OF_SAMPLE (BITS_PER_SAMPLE / 8) // 16 bits / 8 bits == 2 bytes per sample
#define DMA_BUF_SIZE_MS 5
#define DMA_BUF_SIZE_BYTES (SAMPLE_RATE * SIZE_OF_SAMPLE * NUM_CHANNELS * DMA_BUF_SIZE_MS) / 1000
#define DMA_BUF_NUMBER 5
#define MAX_FRAMES_TO_READ ((DMA_BUF_SIZE_BYTES / NUM_CHANNELS) / SIZE_OF_SAMPLE)

#define DR_FLAC_NO_STDIO
#define DR_FLAC_NO_OGG
#define DR_FLAC_IMPLEMENTATION
#define DR_FLAC_BUFFER_SIZE DMA_BUF_SIZE_BYTES
#include "dr_flac.h"



static xQueueHandle i2s_event_queue = NULL;
static xQueueHandle control_event_queue = NULL;

extern const uint8_t flac_file_start[] asm("_binary_flac_sample_small_flac_start");
extern const uint8_t flac_file_end[] asm("_binary_flac_sample_small_flac_end");

void init_i2s() {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = BITS_PER_SAMPLE, /* the DAC module will only take the 8bits from MSB */
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = DMA_BUF_NUMBER,
        .dma_buf_len = DMA_BUF_SIZE_BYTES,
        .use_apll = true
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 10, &i2s_event_queue);
    i2s_set_pin(I2S_NUM, &pin_config);
}

static unsigned int flac_file_index;
static unsigned int flac_file_size;
static drflac *pFlac;
static drflac_int16 i2s_buf[DMA_BUF_SIZE_BYTES];
static uint32_t i2s_buf_start, i2s_buf_end;

size_t onRead(void *pUserData, void *pBufferOut, size_t bytesToRead) {
    size_t bytes_written = 0;
    if (pBufferOut != NULL) {
        uint8_t *buffer_out = (uint8_t*)pBufferOut;
        while (bytes_written < bytesToRead && flac_file_index < flac_file_size) {
            buffer_out[bytes_written] = flac_file_start[flac_file_index];
            bytes_written++;
            flac_file_index++;
        }
    }

    return bytes_written;
}

drflac_bool32 onSeek(void *pUserData, int offset, drflac_seek_origin origin) {
    drflac_bool32 result = DRFLAC_TRUE;
    switch (origin) {
    case drflac_seek_origin_start:
        if (offset < flac_file_size) {
            flac_file_index = offset;
        } else {
            result = DRFLAC_FALSE;
        }
    break;

    case drflac_seek_origin_current:
        if (flac_file_index + offset < flac_file_size) {
            flac_file_index += offset;
        } else {
            result = DRFLAC_FALSE;
        }
    break;

    default:
        assert(false);
    break;
    }
    return result;
}

uint32_t min(uint32_t a, uint32_t b) {
    return a <= b ? a : b;
}

void fill_i2s_buffer() {
    drflac_uint64 frames_read;
    size_t bytes_written = 1;

    if (flac_file_index == flac_file_size) {
        i2s_stop(I2S_NUM);
        return;
    }

    while (flac_file_index != flac_file_size && bytes_written > 0) {
        if (!(i2s_buf_end - i2s_buf_start)) {
            i2s_buf_start = 0;
            frames_read = drflac_read_pcm_frames_s16(pFlac, MAX_FRAMES_TO_READ, i2s_buf);
            if (frames_read != MAX_FRAMES_TO_READ) {
                // TODO handle end of stream
            }
            i2s_buf_end = frames_read * NUM_CHANNELS;
        }

        esp_err_t result = i2s_write(
            I2S_NUM,
            &(i2s_buf[i2s_buf_start]),
            (i2s_buf_end - i2s_buf_start) * SIZE_OF_SAMPLE,
            &bytes_written,
            0
        );
        assert(result == ESP_OK);
        i2s_buf_start += bytes_written / SIZE_OF_SAMPLE;

        if (!bytes_written) {
            break;
        }
    }
}

static void IRAM_ATTR toggle_button(void *args) {
    int32_t num = 1;
    xQueueSendFromISR(control_event_queue, &num, NULL);
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

void stream() {
    i2s_event_t i2s_event;
    while (1) {
        if (xQueueReceive(i2s_event_queue, &i2s_event, portMAX_DELAY)) {
            switch (i2s_event.type) {
            case I2S_EVENT_DMA_ERROR:
                printf("I2S_EVENT_DMA_ERROR\n");
            break;
            case I2S_EVENT_TX_DONE:
                fill_i2s_buffer();
            break;
            case I2S_EVENT_RX_DONE:
                printf("I2S_EVENT_RX_DONE\n");
            break;
            case I2S_EVENT_MAX:
                printf("I2S_EVENT_MAX\n");
            break;
            }
        }
    }
}

void app_main(void)
{
    int32_t num;
    bool play = true;
    control_event_queue = xQueueCreate(10, sizeof(uint32_t));
    flac_file_index = 0;
    flac_file_size = flac_file_end - flac_file_start;

    init_i2s();
    init_gpio();

    pFlac = drflac_open(&onRead, &onSeek, NULL, NULL);
    i2s_stop(I2S_NUM);
    fill_i2s_buffer();
    i2s_start(I2S_NUM);

    xTaskCreatePinnedToCore(stream, "stream", 4096, NULL, 3, NULL, tskNO_AFFINITY);

    while (1) {
        if (xQueueReceive(control_event_queue, &num, portMAX_DELAY)) {
            play = !play;
            if (play) {
                printf("start\n");
                i2s_start(I2S_NUM);
            } else {
                printf("stop\n");
                i2s_stop(I2S_NUM);
            }
        }
    }
}
