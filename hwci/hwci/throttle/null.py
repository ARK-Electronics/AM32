"""No-op throttle: ESC signal is driven by something else (e.g. ARK FPV BDShot).

Used by setup B (``throttle_backend: none``) when the harness only flashes
firmware and/or reads SWD while PX4 owns the signal wire. Must never be used
with Flight Stand throttle profiles — those require ``flightstand``.
"""
from __future__ import annotations

from .base import ThrottleSource


class NullThrottle(ThrottleSource):
    def arm(self) -> None:
        pass

    def set(self, throttle: float) -> None:
        pass

    def disarm(self) -> None:
        pass

    def quiesce(self) -> None:
        # Cannot drop the line; PX4 still owns the pin.
        pass
