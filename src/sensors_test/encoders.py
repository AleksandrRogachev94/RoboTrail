#!/usr/bin/env python3
"""Test script for DC motor encoders.

Spin wheels manually to verify encoder counting works.

Run: python3 -m sensors_test.encoders
"""

import time

from sensors.encoder import Encoder, list_encoder_devices


def main():
    print("Searching for encoder devices...")
    encoders = list_encoder_devices()

    if len(encoders) < 2:
        print(f"Found {len(encoders)} device(s), expected 2.")
        print("Check dtoverlay in /boot/firmware/config.txt and reboot.")
        for path, name in encoders:
            print(f"  {path}: {name}")
        return

    # Second device is left (GPIO 17/27), first is right (GPIO 5/6)
    # Order matches dtoverlay order in config.txt
    right_path, left_path = encoders[0][0], encoders[1][0]

    print(f"Left:  {left_path}")
    print(f"Right: {right_path}")
    print("\nSpin wheels to test. Ctrl+C to stop.\n")

    left = Encoder(left_path)
    right = Encoder(right_path)

    try:
        while True:
            print(
                f"\rL: {left.position:6d}  R: {right.position:6d}", end="", flush=True
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n")
    finally:
        left.close()
        right.close()


if __name__ == "__main__":
    main()
