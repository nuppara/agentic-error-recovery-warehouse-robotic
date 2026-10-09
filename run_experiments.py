#!/usr/bin/env python3
"""
Reproduces every number in the paper. Standard library only.

  python run_experiments.py            # 200 seeds x 500 events, all outputs into results/

Outputs (results/):
  experiment_summary.csv   Table III  four policies, mean and 95% confidence half-width
  fault_stop_time.csv      Section VI stop time of faulted events under the proposed policy (pooled)
  fault_rate_curve.csv     Fig. 2     fault rate by position in the run
  sensitivity_summary.csv  Table IV   wrong-fix sweep, monitor off and on

Policies
  rules_only      deterministic rule table; anything else -> protective stop + operator handoff
  generative_all  every event goes through the (mock) generative engine + evaluation loop
  heal_no_learn   proposed event flow, but accepted remediations are NOT installed as rules
  proposed        proposed event flow with runtime rule installation (function evolution)

All latencies are sampled from the distributions in SimConfig. The generative engine is a mock.
"""
import argparse, csv, math, os, random, statistics
from warehouse_robot_event_loop_autonomous import (SimConfig, SCHEMAS, BASE_RULES, EventBroker, MockLLMHealer,
                                      AnalyticalLoop, generate_events, pct)

BIN = 25                                   # events per bin in the fault-rate curve
WRONG_P = (0.0, 0.05, 0.10, 0.20)          # sensitivity sweep: share of wrong candidates
MONITOR = (0.0, 0.90)                      # monitor off / on (catch probability)


def t95(n):
    """Two-sided 95% Student-t critical value for n samples."""
    table = {30: 2.045, 50: 2.010, 100: 1.984, 200: 1.972}
    return table.get(n, 1.96 if n > 200 else 2.045)


def ci(values):
    m = statistics.mean(values)
    hw = t95(len(values)) * statistics.stdev(values) / math.sqrt(len(values)) if len(values) > 1 else 0.0
    return m, hw


class NoLearnLoop(AnalyticalLoop):
    def on_heal_result(self, res):
        self.heals[res["fault_key"]] = res        # heal applies to waiting events only


def run_event_flow(events, cfg, seed, loop_cls):
    """Returns (rows, loop, healer); row = (resolved, latency_ms, attempts, faulted, wrong_fix)."""
    rng = random.Random(seed)
    broker = EventBroker(SCHEMAS)
    healer = MockLLMHealer(broker, rng, cfg)
    loop = loop_cls(broker, rng, cfg)
    for ev in events:
        broker.publish("telemetry.obstacle", ev)
    rows = [(r["resolved"], r["total_latency_ms"], r["heal_attempts"], r["fault_detected"], r["wrong_fix"])
            for r in loop.records]
    return rows, loop, healer


def run_rules_only(events, cfg, seed):
    rng, out = random.Random(seed), []
    for ev in events:
        edge = rng.lognormvariate(math.log(cfg.edge_median_ms), cfg.edge_sigma)
        ok = ev["obstacle_type"] in BASE_RULES and rng.random() >= cfg.exec_fail_p
        out.append((int(ok), edge, 0, int(not ok), 0))
    return out


def run_generative_all(events, cfg, seed):
    rng, out = random.Random(seed), []
    for ev in events:
        edge = rng.lognormvariate(math.log(cfg.edge_median_ms), cfg.edge_sigma)
        pass_p = cfg.eval_pass_p - 0.04 * (ev["severity"] - 1)
        lat, ok, attempts = edge, False, 0
        while attempts < cfg.max_heal_attempts and not ok:
            attempts += 1
            lat += rng.lognormvariate(math.log(cfg.llm_median_ms), cfg.llm_sigma)
            lat += max(20.0, rng.gauss(cfg.eval_mean_ms, cfg.eval_sd_ms))
            ok = rng.random() < pass_p
        out.append((int(ok), lat + (cfg.patch_swap_ms if ok else 0), attempts, 1, 0))
    return out


POLICIES = {
    "rules_only": run_rules_only,
    "generative_all": run_generative_all,
    "heal_no_learn": lambda e, c, s: run_event_flow(e, c, s, NoLearnLoop)[0],
    "proposed": lambda e, c, s: run_event_flow(e, c, s, AnalyticalLoop)[0],
}


def metrics(rows):
    n = len(rows)
    lat = [r[1] for r in rows]
    return {
        "resolved_pct": 100 * sum(r[0] for r in rows) / n,
        "gen_calls": sum(r[2] for r in rows),
        "fault_pct": 100 * sum(r[3] for r in rows) / n,
        "fault_pct_first100": sum(r[3] for r in rows[:100]),
        "fault_pct_last100": sum(r[3] for r in rows[-100:]),
        "mean_ms": sum(lat) / n,
        "p50_ms": pct(lat, 50),
        "p95_ms": pct(lat, 95),
        "p99_ms": pct(lat, 99),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=200)
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--outdir", default="results")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    seeds = range(1, a.seeds + 1)
    pools = {s: generate_events(SimConfig(n_events=a.n, seed=s), random.Random(s)) for s in seeds}

    # ------------------------------------------------------------------ Table III + Fig. 2 + stop time
    per = {p: [] for p in POLICIES}
    curve = {p: [[] for _ in range(a.n // BIN)] for p in POLICIES}
    stop_ms = []
    for s in seeds:
        cfg = SimConfig(n_events=a.n, seed=s)
        for p, fn in POLICIES.items():
            rows = fn(pools[s], cfg, 10_000 + s)
            per[p].append(metrics(rows))
            for b in range(a.n // BIN):
                curve[p][b].append(100 * sum(r[3] for r in rows[b * BIN:(b + 1) * BIN]) / BIN)
            if p == "proposed":
                stop_ms += [r[1] for r in rows if r[3]]

    with open(os.path.join(a.outdir, "experiment_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["policy", "metric", "mean", "ci95_halfwidth", "min", "max", "seeds"])
        for p, ms in per.items():
            print(f"\n{p}")
            for k in ms[0]:
                v = [m[k] for m in ms]
                mean, hw = ci(v)
                w.writerow([p, k, f"{mean:.3f}", f"{hw:.3f}", f"{min(v):.3f}", f"{max(v):.3f}", len(v)])
                print(f"  {k:<20}{mean:>12.2f} +/- {hw:<8.2f} [{min(v):.2f}, {max(v):.2f}]")

    with open(os.path.join(a.outdir, "fault_stop_time.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["policy", "faulted_events", "median_ms", "mean_ms", "p95_ms", "seeds"])
        w.writerow(["proposed", len(stop_ms), f"{pct(stop_ms, 50):.1f}", f"{statistics.mean(stop_ms):.1f}",
                    f"{pct(stop_ms, 95):.1f}", a.seeds])
    print(f"\nproposed: {len(stop_ms)} faulted events, stop time median {pct(stop_ms, 50):.0f} ms, "
          f"mean {statistics.mean(stop_ms):.0f} ms, p95 {pct(stop_ms, 95):.0f} ms")

    with open(os.path.join(a.outdir, "fault_rate_curve.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["policy", "bin_start_event", "bin_end_event", "fault_pct_mean", "ci95_halfwidth", "seeds"])
        for p in POLICIES:
            for b, v in enumerate(curve[p]):
                mean, hw = ci(v)
                w.writerow([p, b * BIN + 1, (b + 1) * BIN, f"{mean:.3f}", f"{hw:.3f}", len(v)])

    # ------------------------------------------------------------------ Table IV: wrong-fix sensitivity
    print("\nSensitivity: share of wrong candidates x monitor")
    with open(os.path.join(a.outdir, "sensitivity_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["policy", "wrong_fix_p", "monitor_catch_p", "metric", "mean", "ci95_halfwidth", "seeds"])
        for name, cls in (("heal_no_learn", NoLearnLoop), ("proposed", AnalyticalLoop)):
            for wp in WRONG_P:
                for mon in MONITOR:
                    acc = {"gen_calls": [], "wrong_events_pct": [], "wrong_rules": [],
                           "run_has_wrong_rule_pct": [], "resolved_pct": [], "monitor_vetoes": []}
                    for s in seeds:
                        cfg = SimConfig(n_events=a.n, seed=s, wrong_fix_p=wp, monitor_catch_p=mon)
                        rows, loop, healer = run_event_flow(pools[s], cfg, 10_000 + s, cls)
                        acc["gen_calls"].append(sum(r[2] for r in rows))
                        acc["wrong_events_pct"].append(100 * sum(r[4] for r in rows) / len(rows))
                        acc["wrong_rules"].append(len(loop.wrong_rules))
                        acc["run_has_wrong_rule_pct"].append(100.0 if loop.wrong_rules else 0.0)
                        acc["resolved_pct"].append(100 * sum(r[0] for r in rows) / len(rows))
                        acc["monitor_vetoes"].append(healer.monitor_vetoes)
                    line = f"  {name:<14} wrong={wp:<5} monitor={mon:<4}"
                    for k, v in acc.items():
                        mean, hw = ci(v)
                        w.writerow([name, wp, mon, k, f"{mean:.3f}", f"{hw:.3f}", len(v)])
                        line += f" {k}={mean:.2f}±{hw:.2f}"
                    print(line)
    print(f"\nWrote results to {a.outdir}/")


if __name__ == "__main__":
    main()
