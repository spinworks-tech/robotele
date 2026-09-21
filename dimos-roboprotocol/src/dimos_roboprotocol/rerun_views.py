"""Rerun rendering for RoboteleBridge streams.

DimOS's Rerun bridge only draws messages that have a `.to_rerun()`; `Float32`
and `JointState` do not, so we sideload converters via `visual_override` and
provide a blueprint with time-series panels. Module-level functions (not
lambdas) so they survive pickling into DimOS worker processes.
"""
from __future__ import annotations

import math

TELEMETRY = "telemetry"


def battery_to_rerun(msg):
    import rerun as rr

    return [(f"{TELEMETRY}/battery_percent", rr.Scalars(float(msg.data)))]


def joint_state_to_rerun(msg):
    import rerun as rr

    return [
        (f"{TELEMETRY}/joints/{name}", rr.Scalars(math.degrees(pos)))
        for name, pos in zip(msg.name, msg.position)
    ]


def telemetry_blueprint():
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Vertical(
            rrb.TimeSeriesView(
                origin=f"{TELEMETRY}/battery_percent",
                name="Battery %",
                axis_y=rrb.ScalarAxis(range=(0.0, 100.0)),
            ),
            rrb.TimeSeriesView(origin=f"{TELEMETRY}/joints", name="Joint angles (deg)"),
            row_shares=[1, 2],
        )
    )


RERUN_CONFIG = {
    "visual_override": {
        "world/battery_percent": battery_to_rerun,
        "world/joint_state": joint_state_to_rerun,
    },
    "blueprint": telemetry_blueprint,
}
