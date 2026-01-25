"""Hardware encoder reader using Linux rotary-encoder overlay.

Reads encoder pulses from /dev/input/eventX via evdev library.
The kernel handles GPIO interrupts and pulse counting.

Prerequisites:
    Add to /boot/firmware/config.txt:
        dtoverlay=rotary-encoder,pin_a=17,pin_b=27,relative_axis=1
        dtoverlay=rotary-encoder,pin_a=5,pin_b=6,relative_axis=1
    Then reboot. Install: pip install evdev
"""

import evdev


class Encoder:
    """Hardware quadrature encoder reader."""

    def __init__(self, device_path: str, reversed: bool = False):
        """
        Args:
            device_path: Path to input device (e.g., '/dev/input/event0')
            reversed: If True, negate position (for motors mounted opposite direction)
        """
        self.device = evdev.InputDevice(device_path)
        self.device.grab()  # Exclusive access
        self._position = 0
        self._sign = -1 if reversed else 1

    @property
    def position(self) -> int:
        """Current position in encoder ticks (cumulative)."""
        self._poll()
        return self._position

    def reset(self) -> None:
        """Reset position counter to zero."""
        self._poll()  # Drain pending events first
        self._position = 0

    def _poll(self) -> None:
        """Read all pending events and update position."""
        try:
            while True:
                event = self.device.read_one()
                if event is None:
                    break
                if event.type == evdev.ecodes.EV_REL:
                    self._position += event.value * self._sign
        except BlockingIOError:
            pass

    def close(self) -> None:
        """Release device."""
        self.device.ungrab()
        self.device.close()


def list_encoder_devices() -> list[tuple[str, str]]:
    """List all rotary encoder input devices.

    Returns:
        List of (device_path, device_name) tuples.
    """
    encoders = []
    for device_path in evdev.list_devices():
        dev = evdev.InputDevice(device_path)
        if "rotary" in dev.name.lower():
            encoders.append((device_path, dev.name))
        dev.close()
    return encoders
