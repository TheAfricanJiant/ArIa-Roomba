#include <Arduino.h>
#include <driver/i2s.h>
#pragma GCC diagnostic ignored "-Wcpp"

#define I2S_WS    7
#define I2S_BCK   8
#define I2S_MCLK  9
#define I2S_DIN   44

#define SAMPLE_RATE          48000
#define RECORD_RATE          16000
#define DURATION_SEC         10
#define DOWNSAMPLE           3
#define OUT_SAMPLES          (RECORD_RATE * DURATION_SEC)

// ── FIR low-pass filter ───────────────────────────────────────────────────────
// 15-tap, Hamming window, cutoff 7kHz @ 48kHz input
// Keeps full vacuum motor range (100Hz–6kHz), cuts aliasing above 8kHz
// Safe for dirt impact transients — nothing useful lives above 7kHz on a vacuum
#define FIR_TAPS 15
static const float fir_coeffs[FIR_TAPS] = {
    -0.00440f, -0.00940f, -0.00929f,  0.01137f,  0.06392f,
     0.13447f,  0.18905f,  0.21008f,  0.18905f,  0.13447f,
     0.06392f,  0.01137f, -0.00929f, -0.00940f, -0.00440f
};
static float fir_buf[FIR_TAPS] = {0};

float fir_filter(float sample) {
    // Shift history
    for (int i = FIR_TAPS - 1; i > 0; i--) {
        fir_buf[i] = fir_buf[i - 1];
    }
    fir_buf[0] = sample;
    // Convolve
    float acc = 0.0f;
    for (int i = 0; i < FIR_TAPS; i++) {
        acc += fir_coeffs[i] * fir_buf[i];
    }
    return acc;
}

// ── Forward declaration ───────────────────────────────────────────────────────
void record_and_send();

// ── Setup ─────────────────────────────────────────────────────────────────────
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

    if (i2s_driver_install(I2S_NUM_0, &config, 0, NULL) != ESP_OK) {
        Serial.println("ERROR: I2S install failed");
        return;
    }
    if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {
        Serial.println("ERROR: I2S set pin failed");
        return;
    }

    i2s_zero_dma_buffer(I2S_NUM_0);
    Serial.println("READY");
}

// ── Record & send ─────────────────────────────────────────────────────────────
void record_and_send() {
    // Clear filter history before each new clip
    memset(fir_buf, 0, sizeof(fir_buf));

    const int CHUNK_FRAMES       = 64;
    int32_t   raw[CHUNK_FRAMES * 2];
    int16_t   out_buf[512];
    int       out_idx            = 0;
    int       frame_count        = 0;
    int       skip               = 0;
    int       total_input_frames = OUT_SAMPLES * DOWNSAMPLE;  // 480000 frames

    while (frame_count < total_input_frames) {
        int    frames_to_read = min(CHUNK_FRAMES, total_input_frames - frame_count);
        size_t bytes_read     = 0;

        i2s_read(I2S_NUM_0, raw, frames_to_read * 8, &bytes_read, portMAX_DELAY);

        int got_frames = bytes_read / 8;

        for (int i = 0; i < got_frames; i++) {
            // Left channel — shift 24-bit audio out of the 32-bit I2S word
            // Adjust >> 8 if clipping: try >> 9 or >> 10
            // Adjust >> 8 if too quiet: try >> 7
            float sample = (float)(raw[i * 2] >> 8);

            // Run every sample through the FIR BEFORE downsampling
            // This is critical — filter first, then drop samples
            float filtered = fir_filter(sample);

            // Keep every 3rd sample (now alias-free)
            if (skip == 0) {
                int32_t clamped    = (int32_t)max(-32768.0f, min(32767.0f, filtered));
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

    // Flush any remaining samples
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