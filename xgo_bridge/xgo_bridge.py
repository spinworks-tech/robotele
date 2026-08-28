#!/usr/bin/env python3
"""JSON-lines bridge between robot-edge (Rust) and the XGO-Lite V2's
vendor `xgolib` Python SDK (serial-only, no Rust/C binding).

Protocol (see the plan's "xgo_bridge.py IPC design" and DESIGN.md's
Channel B command shape -- this is v0-specific, not part of the wire
protocol itself):

  robot-edge -> bridge (stdin, one JSON object per line):
    {"cmd":"move","x":<mm>,"y":<mm>,"seq":<int>}
    {"cmd":"turn","step":<deg/s>,"seq":<int>}
    {"cmd":"attitude","axis":"r"|"p"|"y","data":<deg>,"seq":<int>}
    {"cmd":"action","id":<1-255>,"seq":<int>}
    {"cmd":"motor","id":<11-43>,"angle":<deg>,"seq":<int>}
    {"cmd":"arm","x":<-80..155mm>,"z":<-95..155mm>,"seq":<int>}
    {"cmd":"claw","pos":<0-255>,"seq":<int>}
    {"cmd":"stop","seq":<int>}
    {"cmd":"heartbeat","seq":<int>}
    {"cmd":"query","seq":<int>}
    {"cmd":"estop","seq":<int>}
    {"cmd":"estop_clear","seq":<int>}

  bridge -> robot-edge (stdout, one JSON object per line):
    {"type":"ack","seq":<int>,"ok":true|false,"error":"..."?}
    {"type":"telemetry","ts":<float>,"motors":[15 floats -- 12 leg + 3 arm],
     "battery":<0-100>,"roll":<float>,"pitch":<float>,"yaw":<float>}
    {"type":"status","state":"ready"|"estopped"|"error","detail":"..."?}
    {"type":"log","level":"info"|"warn"|"error","msg":"..."}

Every received line (any command, including heartbeat) refreshes a local
watchdog independent of robot-edge's own network-side watchdog (Class D
blackout = 400ms, see DESIGN.md/REQUIREMENTS.md SR-4.1) -- this one
catches a hung/crashed *bridge* or a stalled stdin pipe, a distinct
failure mode from network loss. Default threshold: 400ms, matching the
Class D network watchdog rather than undercutting it -- an earlier 150ms
default was found (on real XGO-Lite V2 hardware) to fire spuriously
during normal operation, because blocking vendor SDK calls like
`action()` (e.g. a physical stand-up) can themselves take longer than
150ms to return, delaying the next heartbeat past the threshold even
under a healthy 100ms heartbeat cadence.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from typing import Optional, Protocol


class HardwareBackend(Protocol):
    def move(self, x: float, y: float) -> None: ...
    def turn(self, step: float) -> None: ...
    def attitude(self, axis: str, data: float) -> None: ...
    def action(self, action_id: int) -> None: ...
    def motor(self, motor_id: int, angle: float) -> None: ...
    def arm(self, x: float, z: float) -> None: ...
    def claw(self, pos: int) -> None: ...
    def stop(self) -> None: ...
    def read_telemetry(self) -> dict: ...


class XgoBackend:
    """Wraps the real vendor SDK. Only imports `xgolib` when instantiated,
    so `--stub` mode works on machines without it installed (any dev
    machine/CI, not just the CM4)."""

    def __init__(self, port: str, version: str) -> None:
        from xgolib import XGO  # local import: see class docstring

        self.dog = XGO(port=port, version=version)
        # xgolib's arm(x, z) writes the ARM_X/ARM_Z registers unconditionally
        # and never errors, but the arm doesn't actually move unless
        # ARM_MODE is set first -- claw() has no such gate, which is why claw
        # nudges work without this. Confirmed on real hardware 2026-08-21:
        # arm_mode(1) + a slow, isolated arm() call (no robot-edge/bridge in
        # the loop, ~2s between calls) *does* move the arm -- so 1 is the
        # correct "enabled" value. Still open: why arm() calls driven through
        # this bridge at robot-edge's full tick rate (~50Hz, every command
        # type resent every tick) haven't produced visible movement even
        # with this confirmed-correct mode -- under investigation.
        self.dog.arm_mode(1)
        self._last_arm: tuple[float, float] | None = None
        self._last_claw: int | None = None

    def move(self, x: float, y: float) -> None:
        # Unconditional: these are continuous velocity setpoints (see
        # stop()'s comment below), so x=0.0/y=0.0 is a real stop command,
        # not "nothing to do" -- `if x:` is False for 0.0 in Python and
        # would silently drop it, leaving the robot at its last nonzero
        # velocity forever.
        # Prints to stderr (never stdout -- the JSON-lines protocol pipe)
        # only when non-idle, same reasoning as StubBackend.move: robot-edge
        # sends this every tick even at rest, and printing every idle call
        # would flood the log without telling us anything new.
        if x != 0.0 or y != 0.0:
            print(f"[xgo] move(x={x}, y={y})", file=sys.stderr, flush=True)
        self.dog.move_x(x)
        self.dog.move_y(y)

    def turn(self, step: float) -> None:
        if step != 0.0:
            print(f"[xgo] turn(step={step})", file=sys.stderr, flush=True)
        self.dog.turn(step)

    def attitude(self, axis: str, data: float) -> None:
        self.dog.attitude(axis, data)

    def action(self, action_id: int) -> None:
        print(f"[xgo] action(id={action_id})", file=sys.stderr, flush=True)
        self.dog.action(action_id)

    def motor(self, motor_id: int, angle: float) -> None:
        self.dog.motor(motor_id, angle)

    def arm(self, x: float, z: float) -> None:
        # Position-commanded and sent unconditionally every tick (see
        # robot-edge's dispatch_teleop_command), so gate the print on
        # *change*, not on truthiness -- every tick would flood stderr
        # otherwise, unlike move()'s "only when nonzero" (which is silent
        # at rest by construction).
        if (x, z) != self._last_arm:
            print(f"[xgo] arm(x={x}, z={z})", file=sys.stderr, flush=True)
            self._last_arm = (x, z)
        self.dog.arm(x, z)

    def claw(self, pos: int) -> None:
        if pos != self._last_claw:
            print(f"[xgo] claw(pos={pos})", file=sys.stderr, flush=True)
            self._last_claw = pos
        self.dog.claw(pos)

    def stop(self) -> None:
        # Legs have continuous velocity-mode motion (move_x/move_y/turn)
        # that keeps going until re-commanded, so E-Stop must explicitly
        # zero it. The arm is Cartesian *position*-commanded (arm(x, z)),
        # not velocity-commanded -- it already holds wherever it was last
        # sent and needs no analogous "stop" input.
        print("[xgo] stop()", file=sys.stderr, flush=True)
        self.dog.move_x(0)
        self.dog.move_y(0)
        self.dog.turn(0)

    def read_telemetry(self) -> dict:
        return {
            "motors": self.dog.read_motor(),
            "battery": self.dog.read_battery(),
            "roll": self.dog.read_roll(),
            "pitch": self.dog.read_pitch(),
            "yaw": self.dog.read_yaw(),
        }


class StubBackend:
    """No serial hardware, no `xgolib` import. Used by the two-process
    localhost integration test and for dev-machine smoke testing of the
    JSON-lines protocol/watchdog logic itself.

    Move/turn/action/stop print to stderr (never stdout -- that's the
    JSON-lines protocol pipe robot-edge parses) so a human watching
    robot-edge's terminal output can see keyboard teleop actually
    reaching the backend, since a stub has no other visible effect.
    """

    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self._last_arm: tuple[float, float] | None = None
        self._last_claw: int | None = None

    def move(self, x: float, y: float) -> None:
        self.calls.append(("move", x, y))
        # robot-edge sends `move` unconditionally every tick during
        # FullTeleoperation (even x=y=0 at rest) -- only print when it's
        # an actual command, or this would flood stderr at ~50Hz idle.
        if x != 0.0 or y != 0.0:
            print(f"[stub] move(x={x}, y={y})", file=sys.stderr, flush=True)

    def turn(self, step: float) -> None:
        self.calls.append(("turn", step))
        # Also sent unconditionally every tick now -- same reasoning as move.
        if step != 0.0:
            print(f"[stub] turn(step={step})", file=sys.stderr, flush=True)

    def attitude(self, axis: str, data: float) -> None:
        self.calls.append(("attitude", axis, data))

    def action(self, action_id: int) -> None:
        self.calls.append(("action", action_id))
        print(f"[stub] action(id={action_id})", file=sys.stderr, flush=True)

    def motor(self, motor_id: int, angle: float) -> None:
        self.calls.append(("motor", motor_id, angle))

    def arm(self, x: float, z: float) -> None:
        self.calls.append(("arm", x, z))
        # Position-commanded, sent unconditionally every tick -- gate the
        # print on change, not truthiness (see XgoBackend.arm's comment).
        if (x, z) != self._last_arm:
            print(f"[stub] arm(x={x}, z={z})", file=sys.stderr, flush=True)
            self._last_arm = (x, z)

    def claw(self, pos: int) -> None:
        self.calls.append(("claw", pos))
        if pos != self._last_claw:
            print(f"[stub] claw(pos={pos})", file=sys.stderr, flush=True)
            self._last_claw = pos

    def stop(self) -> None:
        self.calls.append(("stop",))
        print("[stub] stop()", file=sys.stderr, flush=True)

    def read_telemetry(self) -> dict:
        return {"motors": [0.0] * 15, "battery": 100, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


class Bridge:
    def __init__(self, backend: HardwareBackend, watchdog_ms: float, out=sys.stdout) -> None:
        self.backend = backend
        self.watchdog_s = watchdog_ms / 1000.0
        self.out = out
        self.lock = threading.RLock()
        self.estopped = False
        self.last_command_time = time.monotonic()
        self._stop_watchdog = threading.Event()
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)

    def start(self) -> None:
        self._watchdog_thread.start()
        self._emit({"type": "status", "state": "ready"})

    def shutdown(self) -> None:
        self._stop_watchdog.set()
        self._watchdog_thread.join(timeout=1.0)
        with self.lock:
            try:
                self.backend.stop()
            except Exception:
                pass

    def _emit(self, obj: dict) -> None:
        self.out.write(json.dumps(obj) + "\n")
        self.out.flush()

    def _watchdog_loop(self) -> None:
        # Poll rather than a single-shot timer: simplest correct
        # implementation for a threshold in the hundreds-of-ms range,
        # negligible CPU cost.
        poll_interval = min(self.watchdog_s / 3.0, 0.05)
        while not self._stop_watchdog.wait(poll_interval):
            with self.lock:
                if self.estopped:
                    continue
                elapsed = time.monotonic() - self.last_command_time
                if elapsed >= self.watchdog_s:
                    self.estopped = True
                    try:
                        self.backend.stop()
                    except Exception as e:
                        self._emit({"type": "log", "level": "error", "msg": f"watchdog stop() failed: {e}"})
                    self._emit({
                        "type": "status",
                        "state": "estopped",
                        "detail": f"local watchdog: {elapsed * 1000:.1f}ms since last command (threshold {self.watchdog_s * 1000:.0f}ms)",
                    })

    def handle_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as e:
            self._emit({"type": "log", "level": "error", "msg": f"malformed JSON line: {e}"})
            return

        seq = msg.get("seq")
        cmd = msg.get("cmd")

        with self.lock:
            self.last_command_time = time.monotonic()

            if cmd == "estop":
                self.estopped = True
                try:
                    self.backend.stop()
                except Exception:
                    pass
                self._emit({"type": "ack", "seq": seq, "ok": True})
                self._emit({"type": "status", "state": "estopped", "detail": "explicit estop command"})
                return

            if cmd == "estop_clear":
                self.estopped = False
                self.last_command_time = time.monotonic()
                self._emit({"type": "ack", "seq": seq, "ok": True})
                self._emit({"type": "status", "state": "ready"})
                return

            if cmd == "heartbeat":
                self._emit({"type": "ack", "seq": seq, "ok": True})
                return

            if cmd == "query":
                try:
                    telemetry = self.backend.read_telemetry()
                except Exception as e:
                    self._emit({"type": "ack", "seq": seq, "ok": False, "error": str(e)})
                    return
                self._emit({"type": "ack", "seq": seq, "ok": True})
                self._emit({"type": "telemetry", "ts": time.time(), **telemetry})
                return

            if self.estopped and cmd in ("move", "turn", "attitude", "action", "motor", "arm", "claw"):
                self._emit({"type": "ack", "seq": seq, "ok": False, "error": "estopped: motion commands refused until estop_clear"})
                return

            try:
                self._dispatch_motion(cmd, msg)
            except Exception as e:
                self._emit({"type": "ack", "seq": seq, "ok": False, "error": str(e)})
                return
            self._emit({"type": "ack", "seq": seq, "ok": True})

    def _dispatch_motion(self, cmd: Optional[str], msg: dict) -> None:
        if cmd == "move":
            self.backend.move(float(msg.get("x", 0.0)), float(msg.get("y", 0.0)))
        elif cmd == "turn":
            self.backend.turn(float(msg.get("step", 0.0)))
        elif cmd == "attitude":
            self.backend.attitude(str(msg["axis"]), float(msg.get("data", 0.0)))
        elif cmd == "action":
            self.backend.action(int(msg["id"]))
        elif cmd == "motor":
            self.backend.motor(int(msg["id"]), float(msg["angle"]))
        elif cmd == "arm":
            self.backend.arm(float(msg.get("x", 0.0)), float(msg.get("z", 0.0)))
        elif cmd == "claw":
            self.backend.claw(int(msg["pos"]))
        elif cmd == "stop":
            self.backend.stop()
        else:
            raise ValueError(f"unknown cmd: {cmd!r}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port for xgolib (default: /dev/ttyAMA0)")
    parser.add_argument("--xgo-version", default="xgolite", choices=["xgolite", "xgomini"], help="xgolib hardware version string")
    parser.add_argument("--watchdog-ms", type=float, default=400.0, help="Local bridge watchdog threshold in ms (default: 400)")
    parser.add_argument("--stub", action="store_true", help="Use the stub backend instead of real xgolib/serial hardware")
    args = parser.parse_args()

    if args.stub:
        backend: HardwareBackend = StubBackend()
    else:
        try:
            backend = XgoBackend(port=args.port, version=args.xgo_version)
        except Exception as e:
            print(json.dumps({"type": "status", "state": "error", "detail": f"xgolib init failed: {e}"}), flush=True)
            sys.exit(1)

    bridge = Bridge(backend, watchdog_ms=args.watchdog_ms)
    bridge.start()

    try:
        for line in sys.stdin:
            bridge.handle_line(line)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
