import unittest
import csv
import tempfile
from pathlib import Path

import numpy as np

from ml.baseline import REQUIRED, demo_rows, evaluate, read_csv, windows
from ml.protocol import decode_frame


class ProtocolTests(unittest.TestCase):
    def test_decode_signed_12_bit_and_timestamp(self):
        # x=-2048, y=2047, z=-1, vafe=1; the payload follows main.c.
        frame = bytes.fromhex("a5 5b 01 02 03 04 00 f8 7f ff 1f 00")
        result = decode_frame(frame)
        self.assertEqual(result["timestamp_raw"], 0x01020304)
        self.assertEqual(result["timestamp_us"], 0x01020304 * 10)
        self.assertEqual([result[k] for k in ("accel_x_lsb", "accel_y_lsb",
                                              "accel_z_lsb", "vafe_lsb")],
                         [-32768, 32752, -16, 16])

    def test_rejects_wrong_header_or_length(self):
        for data in (b"", b"\xa5\x5a" + bytes(10), b"\xa5\x5b" + bytes(11)):
            with self.assertRaises(ValueError):
                decode_frame(data)


class DatasetTests(unittest.TestCase):
    def test_subjects_do_not_overlap(self):
        result = evaluate(demo_rows(), seed=42)
        self.assertFalse(set(result["train_subjects"]) & set(result["test_subjects"]))
        self.assertEqual(set(result["models"]), {"sEMG", "IMU", "sEMG+IMU"})

    def test_gap_window_is_dropped(self):
        rows = demo_rows()[:400]
        clean = windows(rows, window_size=200, step=200)
        for row in rows[200:]:
            row["timestamp_us"] += 1250
        gap = windows(rows, window_size=200, step=100)
        self.assertEqual(len(clean[0]), 2)
        self.assertEqual(gap[3], 1)
        self.assertEqual(len(gap[0]), 2)

    def test_duplicate_timestamp_rejected(self):
        rows = demo_rows()[:400]
        rows[1]["timestamp_us"] = rows[0]["timestamp_us"]
        with self.assertRaisesRegex(ValueError, "duplicate"):
            windows(rows)

    def test_nonfinite_values_rejected(self):
        rows = demo_rows()[:400]
        rows[0]["vafe_lsb"] = np.nan
        with self.assertRaisesRegex(ValueError, "finite"):
            windows(rows)

    def test_csv_round_trip(self):
        rows = demo_rows()[:400]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "samples.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=REQUIRED)
                writer.writeheader()
                writer.writerows(rows)
            actual = windows(read_csv(path), window_size=200, step=200)
            expected = windows(rows, window_size=200, step=200)
            np.testing.assert_allclose(actual[0], expected[0])
            np.testing.assert_array_equal(actual[1], expected[1])


if __name__ == "__main__":
    unittest.main()
