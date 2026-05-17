#!/usr/bin/env python3
"""Hardware smoke test for the custom ESP-NOW transmitter USB command input.

This targets sketches that read ASCII commands from USB serial:
    RC A E T R AUX1 AUX2 AUX3 AUX4

Example:
    python test/custom_tx_usb_test.py --port COM8 --rate 50 --duration 10
"""

import argparse
import itertools
import time


def test_pattern_commands():
    base = [1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500]
    phases = [
        ("neutral", {}),
        ("aileron_right", {0: 1800}),
        ("aileron_left", {0: 1200}),
        ("elevator_forward", {1: 1800}),
        ("elevator_back", {1: 1200}),
        ("rudder_right", {3: 1800}),
        ("rudder_left", {3: 1200}),
    ]

    for name, overrides in itertools.cycle(phases):
        channels = base.copy()
        for index, value in overrides.items():
            channels[index] = value
        yield name, channels


def read_available_lines(ser):
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line:
            print(f"device: {line}")


def send_usb_commands(port, baudrate, rate_hz, duration_s, phase_s, boot_delay_s):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python -m pip install pyserial") from exc

    interval_s = 1.0 / rate_hz
    next_send = time.monotonic()
    end_time = next_send + duration_s if duration_s else None
    next_phase = next_send
    phase_iter = test_pattern_commands()
    phase_name, channels = next(phase_iter)
    packets_sent = 0

    print(f"Opening {port} at {baudrate} baud")
    print(f"Waiting {boot_delay_s:.1f}s for USB serial boot/reset")

    with serial.Serial(port, baudrate, timeout=0.05, write_timeout=1) as ser:
        time.sleep(boot_delay_s)
        ser.reset_input_buffer()
        print("Sending ASCII RC commands with throttle low and AUX1 low.")
        print("Stop with Ctrl+C.")

        while end_time is None or time.monotonic() < end_time:
            now = time.monotonic()

            if now >= next_phase:
                phase_name, channels = next(phase_iter)
                next_phase = now + phase_s
                print(f"phase={phase_name} channels={channels}")

            line = "RC " + " ".join(str(value) for value in channels) + "\n"
            ser.write(line.encode("ascii"))
            packets_sent += 1
            read_available_lines(ser)
            next_send += interval_s

            sleep_s = next_send - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_send = time.monotonic()

        read_available_lines(ser)

    print(f"Sent {packets_sent} USB RC commands")


def parse_args():
    parser = argparse.ArgumentParser(description="Send ASCII RC commands to the custom USB TX")
    parser.add_argument("--port", required=True, help="Serial port, for example COM8 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="USB serial baudrate")
    parser.add_argument("--rate", type=int, default=50, help="Command send rate in Hz")
    parser.add_argument("--duration", type=float, default=0, help="Seconds to send commands; 0 means until Ctrl+C")
    parser.add_argument("--phase-seconds", type=float, default=3.0, help="Seconds per stick test phase")
    parser.add_argument("--boot-delay", type=float, default=3.0, help="Delay after opening port for ESP reset")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    send_usb_commands(args.port, args.baudrate, args.rate, args.duration, args.phase_seconds, args.boot_delay)
