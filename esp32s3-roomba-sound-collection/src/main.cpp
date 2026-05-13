#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_WS   7
#define I2S_BCK  8
#define I2S_DIN  44

void setup() {
    Serial.begin(921600);
    delay(3000);
    Serial.println("Configuring I2S...");

    i2s_config_t config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = 16000,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = 512,
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

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("I2S driver install failed: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_NUM_0, &pins);
    if (err != ESP_OK) {
        Serial.printf("I2S set pin failed: %d\n", err);
        return;
    }

    i2s_zero_dma_buffer(I2S_NUM_0);
    Serial.println("I2S OK. Reading mic...");
}

void loop() {
    int16_t buffer[512];
    size_t bytes_read = 0;

    i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);

    int16_t max_val = 0;
    for (int i = 0; i < bytes_read / 2; i++) {
        if (abs(buffer[i]) > max_val) max_val = abs(buffer[i]);
    }
    Serial.printf("Bytes read: %d | Max amplitude: %d\n", bytes_read, max_val);
}