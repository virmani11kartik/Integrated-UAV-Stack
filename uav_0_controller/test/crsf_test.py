#!/usr/bin/env python3
"""Unit tests for CRSF RC channel frame encoding.

These tests mirror the packing and CRC logic in src/crsf_bridge.cpp so the
wire format can be checked without an ESP32 or a flight controller.

Run unit tests:
    python test/crsf_test.py
"""

import unittest


CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS = 0x16
CRSF_CHANNEL_VALUE_MIN = 172
CRSF_CHANNEL_VALUE_MID = 992
CRSF_CHANNEL_VALUE_MAX = 1811


def crc8_dvb_s2(data):
    """CRSF CRC8 using the DVB-S2 polynomial 0xD5."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def clamp_channel(value):
    return max(CRSF_CHANNEL_VALUE_MIN, min(CRSF_CHANNEL_VALUE_MAX, int(value)))


def pack_rc_channels(channels):
    """Pack 16 CRSF channels into the 22-byte RC payload."""
    if len(channels) != 16:
        raise ValueError("CRSF RC frames require exactly 16 channels")

    ch = [clamp_channel(value) for value in channels]
    payload = bytearray(22)

    bit_index = 0
    for value in ch:
        for bit in range(11):
            if value & (1 << bit):
                payload[bit_index // 8] |= 1 << (bit_index % 8)
            bit_index += 1

    return bytes(payload)


def build_rc_channels_frame(channels):
    payload = bytes([CRSF_FRAMETYPE_RC_CHANNELS]) + pack_rc_channels(channels)
    return bytes(
        [
            CRSF_ADDRESS_FLIGHT_CONTROLLER,
            len(payload) + 1,  # type + payload + crc
        ]
    ) + payload + bytes([crc8_dvb_s2(payload)])


def neutral_disarmed_channels():
    return [
        992,  # CH1 roll
        992,  # CH2 pitch
        172,  # CH3 throttle low
        992,  # CH4 yaw
        172,  # CH5 AUX1 / arm low
        992,
        992,
        992,
        992,
        992,
        992,
        992,
        992,
        992,
        992,
        992,
    ]


def unpack_rc_channels(payload):
    if len(payload) != 22:
        raise ValueError("CRSF RC payload must be 22 bytes")

    channels = []
    bit_index = 0
    for _ in range(16):
        value = 0
        for bit in range(11):
            if payload[bit_index // 8] & (1 << (bit_index % 8)):
                value |= 1 << bit
            bit_index += 1
        channels.append(value)

    return channels


class TestCRSFEncoding(unittest.TestCase):
    def test_crc8_known_vector(self):
        self.assertEqual(crc8_dvb_s2([CRSF_FRAMETYPE_RC_CHANNELS] + [0] * 22), 0xEF)

    def test_build_neutral_disarmed_frame(self):
        channels = neutral_disarmed_channels()

        frame = build_rc_channels_frame(channels)

        self.assertEqual(len(frame), 26)
        self.assertEqual(frame[0], CRSF_ADDRESS_FLIGHT_CONTROLLER)
        self.assertEqual(frame[1], 24)
        self.assertEqual(frame[2], CRSF_FRAMETYPE_RC_CHANNELS)
        self.assertEqual(frame[-1], crc8_dvb_s2(frame[2:-1]))
        self.assertEqual(unpack_rc_channels(frame[3:25]), channels)

    def test_clamps_out_of_range_channels_before_packing(self):
        channels = [
            0,
            171,
            172,
            992,
            1811,
            1812,
            3000,
            -10,
            500,
            1500,
            2000,
            42,
            2047,
            1000,
            1200,
            1300,
        ]
        expected = [clamp_channel(value) for value in channels]

        frame = build_rc_channels_frame(channels)

        self.assertEqual(unpack_rc_channels(frame[3:25]), expected)
        self.assertEqual(frame[-1], crc8_dvb_s2(frame[2:-1]))

    def test_rejects_wrong_channel_count(self):
        with self.assertRaises(ValueError):
            pack_rc_channels([CRSF_CHANNEL_VALUE_MID] * 15)

        with self.assertRaises(ValueError):
            pack_rc_channels([CRSF_CHANNEL_VALUE_MID] * 17)


if __name__ == "__main__":
    unittest.main()
