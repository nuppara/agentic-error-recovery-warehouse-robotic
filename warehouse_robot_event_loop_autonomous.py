#!/usr/bin/env python3
"""
Event-driven self-healing prototype: autonomous mobile robots (AMRs) meeting obstacles
in a fulfilment warehouse.

Components (mapped to the paper's blueprint):
  EventBroker       - in-memory pub/sub with schema validation on every publish
  AnalyticalLoop    - deterministic rule-table state machine (the edge fast path)
  MockLLMHealer     - stand-in for the Generative Optimization Engine: diagnoses a
                      fault, proposes a remediation, runs it through an evaluation
                      loop, and emits a config patch that is hot-swapped at runtime

This is a discrete-event simulation on a virtual clock. All latencies are SAMPLED
from the distributions in SimConfig, not measured on hardware or a real model.
Standard library only.  Usage:  python warehouse_robot_event_loop_autonomous.py [--n 500] [--seed 42]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
from collections import Counter, defaultdict
from dataclasses import dataclass

NUM = (int, float)


# --------------------------------------------------------------------------- config
@dataclass
class SimConfig:
    n_events: int = 500
    seed: int = 42
    n_robots: int = 12
    mean_arrival_gap_ms: float = 400.0   # Poisson arrivals
    # Analytical loop (edge) latency: lognormal
    edge_median_ms: float = 2.0
    edge_sigma: float = 0.30
    exec_fail_p: float = 0.04            # known maneuver fails physically (wheel slip, docking miss)
    # Generative engine latency per attempt: lognormal LLM call + gaussian eval replay
    llm_median_ms: float = 1400.0
    llm_sigma: float = 0.35
    eval_mean_ms: float = 180.0
    eval_sd_ms: float = 40.0
    patch_swap_ms: float = 5.0
    # Evaluation loop
    max_heal_attempts: int = 3
    eval_pass_p: float = 0.74            # at severity 1; drops 0.04 per severity level
    # Sensitivity study (all zero by default, which reproduces the always-correct mock engine)
    wrong_fix_p: float = 0.0             # probability that a generated candidate is wrong
    eval_catch_p: float = 0.80           # probability the evaluation loop rejects a wrong candidate
    monitor_catch_p: float = 0.0         # probability the on-robot monitor vetoes a wrong candidate
                                         # that slipped through evaluation (0 = monitor off)


# Rules the state machine ships with: obstacle_type -> maneuver
BASE_RULES = {
    "static_pallet": "reroute_adjacent_aisle",
    "parked_cart": "pass_with_offset_0_5m",
    "aisle_barrier": "replan_route",
    "conveyor_crossing": "wait_for_clear_signal",
    # People are never a planning problem: a fixed protective stop, not a generative remediation.
    "person_in_zone": "protective_stop",
}

# Obstacles the shipped rules do not cover, with the mock LLM's canned
# (diagnosis, remediation) for each.
NOVEL = {
    "moving_forklift": ("manually driven vehicle crossing the route", "yield_at_aisle_end_until_clear"),
    "spilled_carton": ("loose items on the floor in the travel path", "stop_reroute_flag_cleanup"),
    "stretch_wrap_debris": ("deformable debris likely to foul the drive wheels", "avoid_cell_and_reroute"),
    "reflective_floor": ("false-positive range return from a wet or polished floor", "fuse_3d_camera_downweight_lidar"),
    "overhanging_load": ("load protruding into the aisle above the scanner plane", "reduce_speed_widen_clearance"),
    "localization_loss": ("position uncertainty spike in a long uniform aisle", "switch_to_fiducial_localization"),
    "blocked_dropoff_station": ("target station occupied or obstructed", "select_alternate_station"),
}

OBSTACLE_WEIGHTS = {**{k: 12 for k in BASE_RULES}, **{k: 4.8 for k in NOVEL}}  # ~36% novel

SCHEMAS = {
    "telemetry.obstacle": {
        "event_id": str, "robot_id": str, "ts_ms": NUM, "zone": str,
        "obstacle_type": str, "severity": int, "distance_m": NUM,
        "battery_pct": NUM, "payload_kg": NUM,
    },
    "fault.detected": {
        "fault_key": str, "event_id": str, "ts_ms": NUM, "kind": str,
        "obstacle_type": str, "severity": int,
    },
    "heal.result": {
        "fault_key": str, "kind": str, "obstacle_type": str, "success": bool,
        "attempts": int, "llm_ms": NUM, "eval_ms": NUM, "done_ms": NUM,
        "diagnosis": str, "maneuver": str,
        "wrong_fix": bool,               # simulation ground truth only; a real engine cannot know this
    },
}


# --------------------------------------------------------------------------- broker
class EventBroker:
    """Synchronous in-memory pub/sub. Rejects payloads that fail the topic schema."""

    def __init__(self, schemas):
        self.schemas = schemas
        self.subs = defaultdict(list)
        self.published = Counter()
        self.rejected = Counter()

    def subscribe(self, topic, handler):
        self.subs[topic].append(handler)

    def publish(self, topic, msg) -> bool:
        schema = self.schemas[topic]
        if any(not isinstance(msg.get(k), t) for k, t in schema.items()):
            self.rejected[topic] += 1
            return False
        self.published[topic] += 1
        for handler in self.subs[topic]:
            handler(msg)
        return True


# --------------------------------------------------------------------------- events
def generate_events(cfg: SimConfig, rng: random.Random):
    types, weights = zip(*OBSTACLE_WEIGHTS.items())
    t, events = 0.0, []
    for i in range(cfg.n_events):
        t += rng.expovariate(1.0 / cfg.mean_arrival_gap_ms)
        events.append({
            "event_id": f"evt-{i:04d}",
            "robot_id": f"amr-{rng.randrange(cfg.n_robots):02d}",
            "ts_ms": round(t, 3),
            "zone": f"aisle-block-{rng.choice('ABCD')}",
            "obstacle_type": rng.choices(types, weights)[0],
            "severity": rng.randint(1, 5),
            "distance_m": round(rng.uniform(0.3, 8.0), 2),
            "battery_pct": round(rng.uniform(22.0, 100.0), 1),
            "payload_kg": round(rng.uniform(1.0, 30.0), 1),
        })
    return events


# --------------------------------------------------------------------------- healer
class MockLLMHealer:
    """Generative Optimization Engine stand-in: diagnose -> propose -> evaluate -> patch."""

    def __init__(self, broker, rng, cfg):
        self.broker, self.rng, self.cfg = broker, rng, cfg
        self.jobs = self.jobs_ok = 0
        self.eval_catches = self.monitor_vetoes = 0
        broker.subscribe("fault.detected", self.on_fault)

    def on_fault(self, fault):
        cfg, rng = self.cfg, self.rng
        otype = fault["obstacle_type"]
        if fault["kind"] == "unknown_obstacle":
            diagnosis, maneuver = NOVEL[otype]
        else:  # known rule failed in execution -> one-off parameter retune
            diagnosis, maneuver = "maneuver executed outside tolerance", f"retune:{BASE_RULES.get(otype, otype)}"

        pass_p = cfg.eval_pass_p - 0.04 * (fault["severity"] - 1)
        llm_ms = eval_ms = 0.0
        success, wrong, attempts = False, False, 0
        while attempts < cfg.max_heal_attempts and not success:
            attempts += 1
            llm_ms += rng.lognormvariate(math.log(cfg.llm_median_ms), cfg.llm_sigma)
            eval_ms += max(20.0, rng.gauss(cfg.eval_mean_ms, cfg.eval_sd_ms))
            if cfg.wrong_fix_p > 0 and rng.random() < cfg.wrong_fix_p:
                # A wrong candidate. It is accepted only if it slips past the evaluation
                # loop and, when the monitor is on, past the on-robot monitor as well.
                if rng.random() < cfg.eval_catch_p:
                    self.eval_catches += 1
                elif rng.random() < cfg.monitor_catch_p:
                    self.monitor_vetoes += 1
                else:
                    success = wrong = True
            else:
                success = rng.random() < pass_p  # correct candidate passes sandbox replay?

        self.jobs += 1
        self.jobs_ok += success
        swap = cfg.patch_swap_ms if success else 0.0
        self.broker.publish("heal.result", {
            "fault_key": fault["fault_key"], "kind": fault["kind"], "obstacle_type": otype,
            "success": success, "attempts": attempts, "llm_ms": llm_ms, "eval_ms": eval_ms,
            "done_ms": fault["ts_ms"] + llm_ms + eval_ms + swap,
            "diagnosis": diagnosis, "maneuver": maneuver, "wrong_fix": wrong,
        })


# --------------------------------------------------------------------------- edge loop
class AnalyticalLoop:
    """Deterministic fast path. Faults are published; patches are applied when they land."""

    def __init__(self, broker, rng, cfg):
        self.broker, self.rng, self.cfg = broker, rng, cfg
        self.rules = dict(BASE_RULES)
        self.config_version = 1
        self.pending_patches = []   # (activate_ms, obstacle_type, maneuver, wrong_fix)
        self.wrong_rules = set()    # ground truth: obstacle types whose installed rule is wrong
        self.heals = {}             # fault_key -> heal.result (latest)
        self.records = []
        broker.subscribe("telemetry.obstacle", self.on_telemetry)
        broker.subscribe("heal.result", self.on_heal_result)

    def on_heal_result(self, res):
        self.heals[res["fault_key"]] = res
        if res["success"] and res["kind"] == "unknown_obstacle":
            self.pending_patches.append((res["done_ms"], res["obstacle_type"], res["maneuver"], res["wrong_fix"]))

    def _apply_patches(self, now):
        """Hot-swap any rule whose patch has landed by `now` (no restart)."""
        still_pending = []
        for activate_ms, otype, maneuver, wrong in self.pending_patches:
            if activate_ms <= now:
                self.rules[otype] = maneuver
                self.config_version += 1
                if wrong:
                    self.wrong_rules.add(otype)
            else:
                still_pending.append((activate_ms, otype, maneuver, wrong))
        self.pending_patches = still_pending

    def _request_heal(self, ev, kind, key, detect_ms):
        self.broker.publish("fault.detected", {
            "fault_key": key, "event_id": ev["event_id"], "ts_ms": detect_ms,
            "kind": kind, "obstacle_type": ev["obstacle_type"], "severity": ev["severity"],
        })
        return self.heals[key]

    def on_telemetry(self, ev):
        cfg, rng = self.cfg, self.rng
        now, otype = ev["ts_ms"], ev["obstacle_type"]
        self._apply_patches(now)
        edge_ms = rng.lognormvariate(math.log(cfg.edge_median_ms), cfg.edge_sigma)
        heal, joined, fault_kind = None, False, ""

        if otype in self.rules:
            if rng.random() < cfg.exec_fail_p:
                fault_kind = "execution_anomaly"
                heal = self._request_heal(ev, fault_kind, ev["event_id"], now + edge_ms)
        else:
            fault_kind = "unknown_obstacle"
            in_flight = self.heals.get(otype)
            if in_flight and in_flight["done_ms"] > now:
                heal, joined = in_flight, True   # robot stays stopped until that heal lands
            else:
                heal = self._request_heal(ev, fault_kind, otype, now + edge_ms)

        if heal is None:
            path, resolved, total = "ANALYTICAL", True, edge_ms
            maneuver = self.rules[otype]
            wrong = otype in self.wrong_rules          # handled by a wrong installed rule
        else:
            resolved = heal["success"]
            total = heal["done_ms"] - now
            path = ("HEAL_JOINED" if joined else "HEALED") if resolved else "ESCALATED"
            maneuver = heal["maneuver"] if resolved else "protective_stop_operator_handoff"
            wrong = bool(resolved and heal["wrong_fix"])

        self.records.append({
            "run_id": len(self.records) + 1,
            "event_id": ev["event_id"],
            "robot_id": ev["robot_id"],
            "zone": ev["zone"],
            "arrival_ms": round(now, 3),
            "obstacle_type": otype,
            "severity": ev["severity"],
            "fault_detected": int(heal is not None),
            "fault_kind": fault_kind,
            "path": path,
            "heal_attempts": 0 if heal is None or joined else heal["attempts"],
            "edge_latency_ms": round(edge_ms, 3),
            "llm_latency_ms": 0.0 if heal is None or joined else round(heal["llm_ms"], 3),
            "eval_latency_ms": 0.0 if heal is None or joined else round(heal["eval_ms"], 3),
            "total_latency_ms": round(total, 3),
            "resolved": int(resolved),
            "wrong_fix": int(wrong),
            "maneuver": maneuver,
            "config_version": self.config_version,
        })


# --------------------------------------------------------------------------- reporting
def pct(values, p):
    s = sorted(values)
    if not s:
        return float("nan")
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def summarize(records, healer, loop, broker):
    n = len(records)
    faults = [r for r in records if r["fault_detected"]]
    healed = [r for r in faults if r["resolved"]]
    print(f"\nEvents processed        : {n}  (schema-rejected: {sum(broker.rejected.values())})")
    print(f"Overall resolved        : {sum(r['resolved'] for r in records)}/{n} "
          f"({100 * sum(r['resolved'] for r in records) / n:.1f}%)")
    print(f"Faults routed to healer : {len(faults)} ({100 * len(faults) / n:.1f}% of events)")
    print(f"Healing success (events): {len(healed)}/{len(faults)} ({100 * len(healed) / len(faults):.1f}%)")
    print(f"Healing success (jobs)  : {healer.jobs_ok}/{healer.jobs} ({100 * healer.jobs_ok / healer.jobs:.1f}%)")
    print(f"Rules learned at runtime: {len(loop.rules) - len(BASE_RULES)}/{len(NOVEL)} "
          f"(config v1 -> v{loop.config_version})")

    print(f"\n{'path':<12}{'count':>6}{'mean ms':>11}{'p50 ms':>11}{'p95 ms':>11}{'max ms':>11}")
    for path in ("ANALYTICAL", "HEALED", "HEAL_JOINED", "ESCALATED"):
        lat = [r["total_latency_ms"] for r in records if r["path"] == path]
        if lat:
            print(f"{path:<12}{len(lat):>6}{sum(lat) / len(lat):>11.2f}{pct(lat, 50):>11.2f}"
                  f"{pct(lat, 95):>11.2f}{max(lat):>11.2f}")
    lat = [r["total_latency_ms"] for r in records]
    print(f"{'ALL':<12}{n:>6}{sum(lat) / n:>11.2f}{pct(lat, 50):>11.2f}{pct(lat, 95):>11.2f}{max(lat):>11.2f}")

    print("\nFunction evolution (per 100-event window):")
    print(f"{'window':<12}{'fast-path %':>12}{'fault %':>10}{'mean ms':>11}")
    for start in range(0, n, 100):
        w = records[start:start + 100]
        fast = sum(r["path"] == "ANALYTICAL" for r in w)
        print(f"{start + 1:>4}-{start + len(w):<7}{100 * fast / len(w):>12.1f}"
              f"{100 * (len(w) - fast) / len(w):>10.1f}"
              f"{sum(r['total_latency_ms'] for r in w) / len(w):>11.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="500_synthetic_runs.csv")
    ap.add_argument("--events-out", default="synthetic_events.jsonl")
    args = ap.parse_args()

    cfg = SimConfig(n_events=args.n, seed=args.seed)
    events = generate_events(cfg, random.Random(cfg.seed))          # event pool
    sim_rng = random.Random(cfg.seed + 1)                           # separate stream for the run

    broker = EventBroker(SCHEMAS)
    healer = MockLLMHealer(broker, sim_rng, cfg)
    loop = AnalyticalLoop(broker, sim_rng, cfg)

    with open(args.events_out, "w") as f:
        for ev in events:
            f.write(json.dumps(ev) + "\n")
    for ev in events:                                               # the event loop
        broker.publish("telemetry.obstacle", ev)

    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(loop.records[0].keys()))
        writer.writeheader()
        writer.writerows(loop.records)

    summarize(loop.records, healer, loop, broker)
    print(f"\nWrote {args.out} and {args.events_out} (seed={cfg.seed})")


if __name__ == "__main__":
    main()
