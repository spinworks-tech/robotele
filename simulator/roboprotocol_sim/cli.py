"""Command-line entrypoint.

    python -m roboprotocol_sim run --scenario dual_arm --failure blackout
    python -m roboprotocol_sim list-scenarios
    python -m roboprotocol_sim list-failures
    python -m roboprotocol_sim sweep --out simulator/output
"""
from __future__ import annotations

import argparse
from pathlib import Path

from .metrics.report import generate_report
from .network.profiles import PROFILES
from .scenarios.definitions import SCENARIOS
from .scenarios.failure_presets import FAILURE_PRESETS
from .scenarios.runner import run_naive_simulation, run_simulation

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "output"


def _add_common_run_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--network", choices=sorted(PROFILES), default="home_broadband_wifi6")
    p.add_argument("--duration", type=float, default=20.0, help="simulated seconds")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    p.add_argument(
        "--compare-naive", action="store_true",
        help="also run the same scenario+failure over a naive WebSocket/TCP baseline "
             "(single ordered stream, no priority, no adaptive bitrate/FEC, no watchdog) "
             "and add a comparison plot + table to the report",
    )


def cmd_list_scenarios(_args: argparse.Namespace) -> None:
    for key, cfg in SCENARIOS.items():
        print(f"{key:24s} {cfg.label}")


def cmd_list_failures(_args: argparse.Namespace) -> None:
    for key, fn in FAILURE_PRESETS.items():
        doc = (fn.__doc__ or "").strip().splitlines()[0] if fn.__doc__ else ""
        print(f"{key:24s} {doc}")


def cmd_run(args: argparse.Namespace) -> None:
    run = run_simulation(
        scenario_key=args.scenario,
        failure_name=args.failure,
        network_profile_name=args.network,
        duration=args.duration,
        seed=args.seed,
    )
    naive_run = None
    if args.compare_naive:
        naive_run = run_naive_simulation(
            scenario_key=args.scenario,
            failure_name=args.failure,
            network_profile_name=args.network,
            duration=args.duration,
            seed=args.seed,
        )
    out_dir = args.out / f"{args.scenario}__{args.failure}__{args.network}"
    checks = generate_report(run, out_dir, naive_run=naive_run)
    print(f"Report written to {out_dir}")
    for c in checks:
        print(f"  [{c.status:4s}] {c.id:10s} {c.description} -- {c.detail}")


def cmd_sweep(args: argparse.Namespace) -> None:
    scenario_keys = list(SCENARIOS) if args.scenarios == "all" else args.scenarios.split(",")
    failure_keys = list(FAILURE_PRESETS) if args.failures == "all" else args.failures.split(",")

    rows = []
    for scen_key in scenario_keys:
        for fail_key in failure_keys:
            print(f"running {scen_key} x {fail_key} x {args.network} ...")
            run = run_simulation(
                scenario_key=scen_key,
                failure_name=fail_key,
                network_profile_name=args.network,
                duration=args.duration,
                seed=args.seed,
            )
            naive_run = None
            if args.compare_naive:
                naive_run = run_naive_simulation(
                    scenario_key=scen_key,
                    failure_name=fail_key,
                    network_profile_name=args.network,
                    duration=args.duration,
                    seed=args.seed,
                )
            out_dir = args.out / f"{scen_key}__{fail_key}__{args.network}"
            checks = generate_report(run, out_dir, naive_run=naive_run)
            n_pass = sum(1 for c in checks if c.status == "PASS")
            n_fail = sum(1 for c in checks if c.status == "FAIL")
            rows.append((scen_key, fail_key, n_pass, n_fail, out_dir))

    comparison_path = args.out / "comparison.md"
    lines = ["# Scenario x Failure sweep comparison", "", "| Scenario | Failure | PASS | FAIL | Report |", "|---|---|---|---|---|"]
    for scen_key, fail_key, n_pass, n_fail, out_dir in rows:
        rel = out_dir.name
        lines.append(f"| {scen_key} | {fail_key} | {n_pass} | {n_fail} | [{rel}/summary.md]({rel}/summary.md) |")
    args.out.mkdir(parents=True, exist_ok=True)
    comparison_path.write_text("\n".join(lines))
    print(f"\nComparison written to {comparison_path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="roboprotocol_sim")
    sub = parser.add_subparsers(dest="command", required=True)

    p_list_s = sub.add_parser("list-scenarios")
    p_list_s.set_defaults(func=cmd_list_scenarios)

    p_list_f = sub.add_parser("list-failures")
    p_list_f.set_defaults(func=cmd_list_failures)

    p_run = sub.add_parser("run")
    p_run.add_argument("--scenario", choices=sorted(SCENARIOS), required=True)
    p_run.add_argument("--failure", choices=sorted(FAILURE_PRESETS), default="none")
    _add_common_run_args(p_run)
    p_run.set_defaults(func=cmd_run)

    p_sweep = sub.add_parser("sweep")
    p_sweep.add_argument("--scenarios", default="all", help="comma-separated scenario keys, or 'all'")
    p_sweep.add_argument("--failures", default="all", help="comma-separated failure keys, or 'all'")
    _add_common_run_args(p_sweep)
    p_sweep.set_defaults(func=cmd_sweep)

    return parser


def main(argv=None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
