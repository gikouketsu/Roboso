"""Host-only representation tests; these do not validate a robot or SPI bus."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "python/tools/inspect_packet.py"
SPEC = importlib.util.spec_from_file_location("inspect_packet", MODULE_PATH)
packet = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(packet)


class PacketRepresentationTests(unittest.TestCase):
    def test_documented_negative_offsets(self):
        self.assertEqual(packet.encode_offsets(-51, -39), bytes([205, 217]))
        self.assertEqual(packet.decode_packet([205, 217]), (-51, -39))

    def test_sender_saturates_large_offsets(self):
        self.assertEqual(packet.encode_offsets(-1000, 1000), bytes([131, 125]))

    def test_all_sender_values_round_trip_on_both_axes(self):
        for value in range(-125, 126):
            with self.subTest(value=value):
                self.assertEqual(
                    packet.decode_packet(packet.encode_offsets(value, -value)),
                    (value, -value),
                )

    def test_decode_full_int8_range_without_claiming_sender_validity(self):
        self.assertEqual(packet.decode_packet([128, 127]), (-128, 127))

    def test_invalid_length_is_rejected(self):
        for values in ([], [0], [0, 1, 2]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                packet.decode_packet(values)

    def test_invalid_byte_values_are_rejected(self):
        for value in (-1, 256, True, 1.5, "1"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                packet.decode_packet([value, 0])

    def test_noninteger_offsets_are_rejected(self):
        for value in (True, 1.5, "1"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                packet.encode_offsets(value, 0)

    def test_cli_outputs_json_and_flags_out_of_sender_range(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            packet.main(["decode", "128", "127"])
        result = json.loads(output.getvalue())
        self.assertEqual(result["offset"], [-128, 127])
        self.assertFalse(result["within_sender_range"])

    def test_cli_rejects_out_of_range_byte(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                packet.main(["decode", "256", "0"])
        self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
