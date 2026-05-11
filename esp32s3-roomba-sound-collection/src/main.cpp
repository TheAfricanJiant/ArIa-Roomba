#include <Arduino.h>
#include <driver/i2s.h>

// Suppress deprecated I2S API warning (ESP-IDF legacy API, still functional)
#pragma GCC diagnostic ignored "-Wcpp"

#define I2S_WS    7
#define I2S_BCK   8
#define I2S_MCLK  9
#define I2S_DIN   44

#define SAMPLE_RATE     48000
#define RECORD_RATE     16000
#define DURATION_SEC    10
#define DOWNSAMPLE      3
#define OUT_SAMPLES     (RECORD_RATE * DURATION_SEC)

// ── Forward declaration ──────────────────────────────────────────────────────
void record_and_send();

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(921600);
    delay(2000);

    i2s_config_t config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = 512,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };

    i2s_pin_config_t pins = {
        .mck_io_num   = I2S_MCLK,
        .bck_io_num   = I2S_BCK,
        .ws_io_num    = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_DIN
    };

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("I2S install failed: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_NUM_0, &pins);
    if (err != ESP_OK) {
        Serial.printf("I2S set pin failed: %d\n", err);
        return;
    }

    i2s_zero_dma_buffer(I2S_NUM_0);
    Serial.println("READY");
}

// ── Record & send ─────────────────────────────────────────────────────────────
void record_and_send() {
    const int CHUNK_FRAMES = 64;
    const int CHUNK_BYTES  = CHUNK_FRAMES * 2 * 4;

    int32_t raw[CHUNK_FRAMES * 2];
    int16_t out_buf[512];
    int     out_idx          = 0;
    int     frame_count      = 0;
    int     skip             = 0;
    int     total_input_frames = OUT_SAMPLES * DOWNSAMPLE;

    while (frame_count < total_input_frames) {
        int    frames_to_read = min(CHUNK_FRAMES, total_input_frames - frame_count);
        size_t bytes_read     = 0;

        i2s_read(I2S_NUM_0, raw, frames_to_read * 8, &bytes_read, portMAX_DELAY);

        int got_frames = bytes_read / 8;

        for (int i = 0; i < got_frames; i++) {
            int32_t sample32 = raw[i * 2] >> 8;   // left channel, 24-bit value

            if (skip == 0) {
                int32_t clamped  = max((int32_t)-32768, min((int32_t)32767, sample32));
                out_buf[out_idx++] = (int16_t)clamped;

                if (out_idx == 512) {
                    Serial.write((uint8_t*)out_buf, 512 * 2);
                    out_idx = 0;
                }
            }
            skip = (skip + 1) % DOWNSAMPLE;
        }

        frame_count += got_frames;
    }

    // Flush remainder
    if (out_idx > 0) {
        Serial.write((uint8_t*)out_buf, out_idx * 2);
    }

    Serial.println("\nDONE");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "START") {
            record_and_send();
        }
    }
}