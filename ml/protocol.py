"""Decode a complete 12-byte A5 5B sample frame from the firmware.

This deliberately accepts exactly one frame, not arbitrary BLE byte streams.
"""


def decode_frame(frame):
    if len(frame) != 12 or frame[:2] != b"\xa5\x5b":
        raise ValueError("expected one 12-byte A5 5B sample frame")
    timestamp = int.from_bytes(frame[2:6], "big")
    p = frame[6:12]
    values = (p[0] | ((p[1] & 15) << 8), (p[1] >> 4) | (p[2] << 4),
              p[3] | ((p[4] & 15) << 8), (p[4] >> 4) | (p[5] << 4))
    # Firmware stores four left-aligned signed 16-bit values as 12-bit words.
    signed = tuple((value - 4096 if value & 2048 else value) * 16 for value in values)
    return {"timestamp_raw": timestamp, "timestamp_us": timestamp * 10,
            "accel_x_lsb": signed[0], "accel_y_lsb": signed[1],
            "accel_z_lsb": signed[2], "vafe_lsb": signed[3]}
