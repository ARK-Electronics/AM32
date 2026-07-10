"""Flight Stand backends."""
from .base import SafetyLimits, StandSafetyTripped, StandSample, ThrustStand  # noqa: F401
from .grpc_client import FlightStandGrpc, SignalMap  # noqa: F401

# Lazy: simulator imports RigSimulator from hwci.sim, which imports StandSample
# from this package — a top-level SimulatedStand import would circular-import.
def __getattr__(name: str):
    if name == "SimulatedStand":
        from .simulator import SimulatedStand
        return SimulatedStand
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
