#include <Arduino.h>
#include <driver/i2s.h>

// I2S pins - XMOS to XIAO ESP32S3 on ReSpeaker Lite
#define I2S_WS    7
#define I2S_BCK   8
#define I2S_DIN   9

#define SAMPLE_RATE     16000
#define SAMPLE_BITS     16
#define CHANNEL_FORMAT  I2S_CHANNEL_FMT_ONLY_LEFT
#define DMA_BUF_COUNT   8
#define DMA_BUF_LEN     512

void i2s_init() {
    i2s_config_t config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = CHANNEL_FORMAT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };

    i2s_pin_config_t pins = {
        .bck_io_num   = I2S_BCK,
        .ws_io_num    = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_DIN
    };

    i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pins);
    i2s_zero_dma_buffer(I2S_NUM_0);
}

void setup() {
    Serial.begin(921600);
    while (!Serial);
    i2s_init();
    // Signal to Python that we are ready
    Serial.println("READY");
}

void loop() {
    // Wait for Python to send a start command
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "START") {
            int16_t buffer[DMA_BUF_LEN];
            size_t bytes_read = 0;

            // Stream for 10 seconds
            int total_samples = SAMPLE_RATE * 10;
            int sent = 0;

            while (sent < total_samples) {
                i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
                Serial.write((uint8_t*)buffer, bytes_read);
                sent += bytes_read / 2;
            }
            Serial.println("\nDONE");
        }
    }
}