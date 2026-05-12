# pip install pyserial
# python3 recorder.py

# Script works with esp32s3 respeaker lite to
# record data from vacuum motor

import serial
import serial.tools.list_ports
import wave
import os
import time

BAUD        = 921600
SAMPLE_RATE = 16000
CHANNELS    = 1
SAMPWIDTH   = 2    # 16-bit = 2 bytes
DURATION    = 10   # seconds per clip

OUTPUT_DIR  = "recordings"
os.makedirs(OUTPUT_DIR, exist_ok=True)

CLASSES         = ["no_dirt", "moderate_dirt", "heavy_dirt"]
CLIPS_PER_CLASS = 18
PREFERRED_PORT  = "COM10"   # tried first; falls back to auto-scan if not found


# ── Port discovery ────────────────────────────────────────────────────────────

def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def find_board(last_port=None):
    """
    Try ports in this order:
      1. last_port  (the port that just disconnected — often reconnects same name)
      2. PREFERRED_PORT
      3. Every other port on the system
    Keeps looping forever until a board answers READY.
    """
    attempt = 0
    while True:
        ports = list_serial_ports()

        # Build ordered candidate list without duplicates
        candidates = []
        for p in [last_port, PREFERRED_PORT]:
            if p and p not in candidates:
                candidates.append(p)
        for p in ports:
            if p not in candidates:
                candidates.append(p)

        if not candidates:
            if attempt % 10 == 0:
                print("  No serial ports found. Waiting for device...")
            attempt += 1
            time.sleep(2)
            continue

        for port in candidates:
            try:
                print(f"  Trying {port} ...", end="", flush=True)
                ser = serial.Serial(port, BAUD, timeout=3)
                time.sleep(2)
                ser.reset_input_buffer()

                # Send a newline in case board already sent READY before we opened
                ser.write(b"\n")

                deadline = time.time() + 4
                found    = False
                while time.time() < deadline:
                    line = ser.readline().decode(errors="ignore").strip()
                    if line == "READY":
                        found = True
                        break

                if found:
                    print(f" READY!")
                    return ser, port
                else:
                    print(" no response.")
                    ser.close()

            except (serial.SerialException, OSError):
                print(" unavailable.")

        attempt += 1
        time.sleep(2)


# ── Helpers ───────────────────────────────────────────────────────────────────

def next_clip_number(label):
    existing = [
        f for f in os.listdir(OUTPUT_DIR)
        if f.startswith(label + "_") and f.endswith(".wav")
    ]
    if not existing:
        return 1
    numbers = []
    for f in existing:
        try:
            numbers.append(int(f.replace(label + "_", "").replace(".wav", "")))
        except ValueError:
            pass
    return max(numbers) + 1 if numbers else 1


def count_clips(label):
    return len([
        f for f in os.listdir(OUTPUT_DIR)
        if f.startswith(label + "_") and f.endswith(".wav")
    ])


def print_summary():
    print("\n" + "="*45)
    print("  Current clip counts:")
    for label in CLASSES:
        n = count_clips(label)
        bar = "█" * n + "░" * max(0, CLIPS_PER_CLASS - n)
        print(f"  {label:<18} {bar} {n}/{CLIPS_PER_CLASS}")
    print("="*45)


# ── Recording ─────────────────────────────────────────────────────────────────

def record_clip(ser, label, clip_number):
    filename = os.path.join(OUTPUT_DIR, f"{label}_{clip_number:03d}.wav")
    print(f"  Recording {filename} ...", end="", flush=True)

    ser.reset_input_buffer()
    ser.write(b"START\n")

    expected_bytes = SAMPLE_RATE * CHANNELS * SAMPWIDTH * DURATION
    audio_data     = b""

    while len(audio_data) < expected_bytes:
        try:
            chunk = ser.read(min(4096, expected_bytes - len(audio_data)))
        except (serial.SerialException, OSError):
            raise ConnectionError("Lost connection during recording")
        if not chunk:
            raise ConnectionError("Timeout during recording")
        audio_data += chunk

    try:
        ser.readline()  # discard DONE
    except (serial.SerialException, OSError):
        pass  # non-fatal — we already have the audio

    with wave.open(filename, "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPWIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(audio_data)

    print(f" saved ({len(audio_data)} bytes)")
    return filename


def prompt_action(label, clip_index, total):
    print(f"\n  Clip {clip_index}/{total}  |  Class: '{label}'")
    print("  [ENTER]  Record")
    print("  [s]      Skip this class")
    print("  [r]      Redo last clip")
    print("  [q]      Quit")
    return input("  > ").strip().lower()


# ── Class recording  ──────────────────────────────────────────────────────────

def record_class(ser, label, start_clip, start_recorded):
    """
    Returns (ser, port, status) where status is:
      'done'       — class finished normally
      'skipped'    — user skipped
      'quit'       — user wants to exit
      'reconnect'  — board disconnected; caller should reconnect then retry
    """
    clip_num  = start_clip
    recorded  = start_recorded
    last_file = None

    while recorded < CLIPS_PER_CLASS:
        action = prompt_action(label, recorded + 1, CLIPS_PER_CLASS)

        if action == "":
            try:
                last_file = record_clip(ser, label, clip_num)
                clip_num += 1
                recorded += 1
                time.sleep(0.3)
            except ConnectionError as e:
                print(f"\n  !! {e}")
                print("  Clip NOT saved. Will reconnect and resume from this clip.")
                return ser, "reconnect", clip_num, recorded

        elif action == "r":
            if last_file and os.path.exists(last_file):
                os.remove(last_file)
                clip_num -= 1
                recorded  = max(0, recorded - 1)
                print(f"  Deleted {last_file}. Re-recording...")
                try:
                    last_file = record_clip(ser, label, clip_num)
                    clip_num += 1
                    recorded += 1
                    time.sleep(0.3)
                except ConnectionError as e:
                    print(f"\n  !! {e}")
                    return ser, "reconnect", clip_num, recorded
            else:
                print("  Nothing to redo yet.")

        elif action == "s":
            print(f"  Skipping '{label}'.")
            return ser, "skipped", clip_num, recorded

        elif action == "q":
            return ser, "quit", clip_num, recorded

        else:
            print("  Unknown input — press ENTER, s, r, or q.")

    print(f"\n  Done with '{label}'! ({count_clips(label)} clips total)")
    return ser, "done", clip_num, recorded


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("="*45)
    print("  Vacuum Sound Recorder — Edge Impulse")
    print("="*45)
    print_summary()

    print("\nSearching for board...")
    ser, current_port = find_board()

    class_index = 0

    while class_index < len(CLASSES):
        label      = CLASSES[class_index]
        clip_num   = next_clip_number(label)
        recorded   = count_clips(label)

        # Skip classes already fully recorded
        if recorded >= CLIPS_PER_CLASS:
            print(f"\n  '{label}' already has {recorded} clips — skipping.")
            class_index += 1
            continue

        print(f"\n{'='*45}")
        print(f"  CLASS: {label}")
        print(f"  Already recorded: {recorded}/{CLIPS_PER_CLASS} clip(s)")
        print(f"{'='*45}")

        choice = input("  [ENTER] Start  |  [s] Skip class  |  [q] Quit\n  > ").strip().lower()
        if choice == "s":
            print(f"  Skipping '{label}'.")
            class_index += 1
            continue
        if choice == "q":
            break

        # Recording loop with reconnect support
        while True:
            ser, status, clip_num, recorded = record_class(
                ser, label, clip_num, recorded
            )

            if status == "reconnect":
                print("\n  Board disconnected. Reconnecting...")
                try:
                    ser.close()
                except Exception:
                    pass
                ser, current_port = find_board(last_port=current_port)
                print("  Resuming recording...\n")
                # Loop back into record_class at the same clip
                continue

            elif status == "done":
                class_index += 1
                break

            elif status == "skipped":
                class_index += 1
                break

            elif status == "quit":
                print_summary()
                ser.close()
                return

    print_summary()
    try:
        ser.close()
    except Exception:
        pass
    print("\n  Upload 'recordings/' to Edge Impulse.")


if __name__ == "__main__":
    while True:
        try:
            main()
            break  # clean exit via quit command
        except KeyboardInterrupt:
            print("\n\n  Interrupted by user. Exiting.")
            break
        except Exception as e:
            print(f"\n  Unexpected error: {e}")
            print("  Restarting in 3 seconds...")
            time.sleep(3)