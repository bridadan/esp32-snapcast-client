#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"

#define SAMPLE_RATE     44100
#define I2S_NUM         0
#define I2S_BCK_IO      GPIO_NUM_26
#define I2S_WS_IO       GPIO_NUM_25
#define I2S_DO_IO       GPIO_NUM_22
#define I2S_DI_IO       -1

#define DMA_BUF_SIZE_MS 20
#define DMA_BUF_SIZE_BYTES (SAMPLE_RATE  * DMA_BUF_SIZE_MS) / 1000
#define DMA_BUF_NUMBER 5

#define DR_FLAC_NO_STDIO
#define DR_FLAC_NO_OGG
#define DR_FLAC_IMPLEMENTATION
#define DR_FLAC_BUFFER_SIZE DMA_BUF_SIZE_BYTES
#include "dr_flac.h"



static xQueueHandle i2s_evt_queue = NULL;

extern const uint8_t flac_file_start[] asm("_binary_flac_sample_small_flac_start");
extern const uint8_t flac_file_end[] asm("_binary_flac_sample_small_flac_end");

void init_i2s() {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16, /* the DAC module will only take the 8bits from MSB */
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = DMA_BUF_NUMBER,
        .dma_buf_len = DMA_BUF_SIZE_BYTES,
        .use_apll = true
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, NULL);
}

static unsigned int flac_file_index;
static unsigned int flac_file_size;
static drflac *pFlac;
static drflac_int16 i2s_buf[DMA_BUF_SIZE_BYTES];

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

void write_all_to_i2s(drflac_uint64 frames_read) {
    uint32_t bytes_read = frames_read * 4;
    size_t total_bytes_written = 0;
    size_t bytes_written = 0;
    while (total_bytes_written < bytes_read) {
        esp_err_t result = i2s_write(
            I2S_NUM,
            &(i2s_buf[total_bytes_written]),
            bytes_read - total_bytes_written,
            &bytes_written,
            100
        );
        assert(result == ESP_OK);
        total_bytes_written += bytes_written;
        printf("total_bytes_written: %u/%u\r\n", total_bytes_written, bytes_read);
    }
}

void start_flac_stream() {
    flac_file_index = 0;
    flac_file_size = flac_file_end - flac_file_start;
    //printf("flac_file_size %u\r\n", flac_file_size);
    pFlac = drflac_open(&onRead, &onSeek, NULL, NULL);
    //printf("opened\r\n");
    uint32_t frames_to_read = DMA_BUF_SIZE_BYTES / 4;
    drflac_uint64 frames_read = drflac_read_pcm_frames_s16(pFlac, frames_to_read, i2s_buf);
    while (frames_read == frames_to_read) {
        write_all_to_i2s(frames_read);
        frames_read = drflac_read_pcm_frames_s16(pFlac, frames_to_read, i2s_buf);
    }
    write_all_to_i2s(frames_read);
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

    start_flac_stream();

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
