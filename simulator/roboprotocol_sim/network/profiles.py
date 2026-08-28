"""Named "standard consumer network connection" profiles.

The spec (NFR-1.2) only pins down two target numbers -- control RTT
<30ms on local Wi-Fi, <60ms over 5G -- and NFR-2.1's 2-20Mbps video
bitrate envelope. Everything else here (jitter, baseline loss,
asymmetric bandwidth) is a documented simulator assumption, chosen to
be realistic for a WAN path between an operator workstation and a
robot (see simulator/SPEC.md "Assumptions" for the write-up).

Bandwidth is deliberately asymmetric and upload-constrained on the
robot's side, since the robot is the one uploading the larger flow
(video Channel A) back to the operator.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class NetworkProfile:
    name: str
    base_owd_ms: float          # one-way propagation delay, per direction
    jitter_std_ms: float        # stddev of a normal jitter draw, clipped >= 0
    baseline_loss: float        # probability, independent per datagram
    robot_upload_mbps: float    # robot -> operator (video + telemetry + haptic)
    operator_upload_mbps: float  # operator -> robot (commands are tiny, rarely the bottleneck)


HOME_BROADBAND_WIFI6 = NetworkProfile(
    name="home_broadband_wifi6",
    base_owd_ms=12.0,
    jitter_std_ms=3.0,
    baseline_loss=0.001,
    robot_upload_mbps=20.0,
    operator_upload_mbps=20.0,
)

CELLULAR_5G = NetworkProfile(
    name="cellular_5g",
    base_owd_ms=20.0,
    jitter_std_ms=8.0,
    baseline_loss=0.005,
    robot_upload_mbps=30.0,
    operator_upload_mbps=15.0,
)

PROFILES = {
    HOME_BROADBAND_WIFI6.name: HOME_BROADBAND_WIFI6,
    CELLULAR_5G.name: CELLULAR_5G,
}


def get_profile(name: str) -> NetworkProfile:
    try:
        return PROFILES[name]
    except KeyError as exc:
        raise ValueError(
            f"unknown network profile {name!r}; choices: {sorted(PROFILES)}"
        ) from exc
