"""Offline inspection of Roboso's legacy two-byte offset representation.

Added during portfolio maintenance. Does not import or operate camera/SPI code.
"""

import argparse
import json


def _integer(value):
    if type(value) is not int:
        raise ValueError("Expected an integer")
    return value


def encode_offsets(x, y):
    """Match the sender's -125..125 clamp followed by value & 0xFF."""
    offsets = [max(-125, min(125, _integer(value))) for value in (x, y)]
    return bytes(value & 0xFF for value in offsets)


def decode_packet(packet):
    """Interpret exactly two unsigned bytes as two signed int8 values.

    No checksum, freshness, framing, or transport validation is performed.
    """
    if len(packet) != 2:
        raise ValueError("Expected exactly two bytes: x, y")
    for value in packet:
        if not 0 <= _integer(value) <= 255:
            raise ValueError("Byte values must be in 0..255")
    return tuple(value if value < 128 else value - 256 for value in packet)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    encode = commands.add_parser("encode", help="Clamp pixel offsets and encode")
    encode.add_argument("x", type=int)
    encode.add_argument("y", type=int)
    decode = commands.add_parser("decode", help="Decode two decimal wire bytes")
    decode.add_argument("x_byte", type=int)
    decode.add_argument("y_byte", type=int)
    args = parser.parse_args(argv)
    try:
        if args.command == "encode":
            packet = encode_offsets(args.x, args.y)
            result = {"input_offset": [args.x, args.y],
                      "clamped_offset": decode_packet(packet),
                      "wire_bytes": list(packet)}
        else:
            packet = [args.x_byte, args.y_byte]
            offset = decode_packet(packet)
            result = {"wire_bytes": packet, "offset": offset,
                      "within_sender_range": all(-125 <= v <= 125 for v in offset)}
    except ValueError as error:
        parser.error(str(error))
    print(json.dumps(result))


if __name__ == "__main__":
    main()
