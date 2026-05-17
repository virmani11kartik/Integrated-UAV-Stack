#!/usr/bin/env python3
"""Hardware smoke test for off-the-shelf ELRS TX modules using CRSF UART.

This script writes CRSF RC channel frames directly to a serial device, emulating
the handset/module-bay side of an ELRS transmitter.

Example:
    python test/ots_elrs_serial_test.py --port COM7 --rate 100 --duration 10
"""

import argparse
import itertools
import time

from crsf_test import build_rc_channels_frame, neutral_disarmed_channels


def test_pattern_channels():
    base = neutral_disarmed_channels()
    phases = [
        ("neutral", {}),
        ("roll_right", {0: 1400}),
        ("roll_left", {0: 600}),
        ("pitch_forward", {1: 1400}),
        ("pitch_back", {1: 600}),
        ("yaw_right", {3: 1400}),
        ("yaw_left", {3: 600}),
    ]

    for name, overrides in itertools.cycle(phases):
        channels = base.copy()
        for index, value in overrides.items():
            channels[index] = value
        yield name, channels


def send_serial_frames(port, baudrate, rate_hz, duration_s, phase_s):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python -m pip install pyserial") from exc

    interval_s = 1.0 / rate_hz
    next_send = time.monotonic()
    end_time = next_send + duration_s if duration_s else None
    next_phase = next_send
    phase_iter = test_pattern_channels()
    phase_name, channels = next(phase_iter)
    packets_sent = 0

    print(f"Opening {port} at {baudrate} baud")
    print("Sending CRSF RC frames with throttle low and AUX1/arm low.")
    print("Stop with Ctrl+C.")

    with serial.Serial(port, baudrate, timeout=0, write_timeout=1) as ser:
        while end_time is None or time.monotonic() < end_time:
            now = time.monotonic()

            if now >= next_phase:
                phase_name, channels = next(phase_iter)
                next_phase = now + phase_s
                print(f"phase={phase_name} channels[0:5]={channels[:5]}")

            ser.write(build_rc_channels_frame(channels))
            packets_sent += 1
            next_send += interval_s

            sleep_s = next_send - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_send = time.monotonic()

    print(f"Sent {packets_sent} CRSF frames")


def parse_args():
    parser = argparse.ArgumentParser(description="Send CRSF RC frames to an ELRS UART device")
    parser.add_argument("--port", required=True, help="Serial port, for example COM7 or /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=420000, help="CRSF UART baudrate")
    parser.add_argument("--rate", type=int, default=100, help="RC frame send rate in Hz")
    parser.add_argument("--duration", type=float, default=0, help="Seconds to send frames; 0 means until Ctrl+C")
    parser.add_argument("--phase-seconds", type=float, default=3.0, help="Seconds per stick test phase")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    send_serial_frames(args.port, args.baudrate, args.rate, args.duration, args.phase_seconds)
