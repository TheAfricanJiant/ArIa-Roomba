#pip install pyserial
#python3 recorder.py

#Script works with esp32s3 realspeaker lite to 
# record data from vacuum motor 

import serial
import wave
import os
import time

PORT       = "COM10"
BAUD       = 921600
SAMPLE_RATE = 16000
CHANNELS   = 1
SAMPWIDTH  = 2          # 16-bit = 2 bytes
DURATION   = 10         # seconds per clip — matches ESP32 code

OUTPUT_DIR = "recordings"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def record_clip(ser, label, clip_number):
    filename = os.path.join(OUTPUT_DIR, f"{label}_{clip_number:03d}.wav")
    print(f"  Recording {filename} ...")

    ser.write(b"START\n")

    expected_bytes = SAMPLE_RATE * CHANNELS * SAMPWIDTH * DURATION
    audio_data = b""

    while len(audio_data) < expected_bytes:
        chunk = ser.read(min(1024, expected_bytes - len(audio_data)))
        audio_data += chunk

    # Read and discard the DONE line
    ser.readline()

    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPWIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(audio_data)

    print(f"  Saved. ({len(audio_data)} bytes)")

def main():
    ser = serial.Serial(PORT, BAUD, timeout=30)
    time.sleep(2)

    # Wait for READY
    print("Waiting for board...")
    while True:
        line = ser.readline().decode().strip()
        if line == "READY":
            print("Board ready.\n")
            break

    classes = ["no_dirt", "moderate_dirt", "heavy_dirt"]
    clips_per_class = 18  # 18 x 10s = 3 minutes per class

    for label in classes:
        print(f"\n--- CLASS: {label} ---")
        input(f"Set up your vacuum for '{label}' then press ENTER to start recording...")

        for i in range(clips_per_class):
            print(f"Clip {i+1}/{clips_per_class}")
            record_clip(ser, label, i+1)
            time.sleep(0.5)

        print(f"Done with {label}!")

    ser.close()
    print("\nAll done. Upload the 'recordings' folder to Edge Impulse.")

if __name__ == "__main__":
    main()