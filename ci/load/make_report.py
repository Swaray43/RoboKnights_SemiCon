#!/usr/bin/env python3
"""
ci/load/make_report.py -- Step 4/5 three-part report (base / Wave A+B / Wave C).

    python3 ci/load/make_report.py --load base.json ab.json c.json \
        --vivado base_summary.txt ab_summary.txt c_summary.txt --out report.html

Any of the Vivado summaries may be "-" (not run yet). Load JSONs come from
ci/load/run_workloads.py. Writes one self-contained HTML page.
"""
import html
import json
import re
import sys
from pathlib import Path

LABELS = ["Base (step4-base)", "Wave A+B", "Wave C"]

WL_ROWS = [
    ("W1", "SmartNIC IMIX, bursty", [("ch0_gbps", "NIC GB/s", 3, +1), ("ch0_mpps", "Mpps", 3, +1),
                                    ("pkt_lat50", "packet latency p50 (ns)", 0, -1), ("pkt_lat99", "packet latency p99 (ns)", 0, -1)]),
    ("W1_B2B", "SmartNIC back-to-back 64 B", [("ch0_gbps", "NIC GB/s", 3, +1), ("ch0_mpps", "Mpps", 3, +1),
                                              ("pkt_lat99", "packet latency p99 (ns)", 0, -1)]),
    ("W2_64K", "ML 64 KB tensors", [("ch1_gbps", "ML GB/s", 3, +1), ("ch1_lat99", "TLP latency p99 (ns)", 0, -1)]),
    ("W2_1M", "ML 1 MB tensor", [("ch1_gbps", "ML GB/s", 3, +1), ("ch1_lat99", "TLP latency p99 (ns)", 0, -1)]),
    ("W2_4K", "ML, 4 KB-crossing heavy", [("ch1_gbps", "ML GB/s", 3, +1)]),
    ("W3_11", "Mixed, weights 1:1", [("ch0_gbps", "NIC GB/s", 3, +1), ("ch1_gbps", "ML GB/s", 3, +1),
                                     ("pkt_lat99", "NIC p99 (ns)", 0, -1), ("share0", "NIC share", 3, 0), ("jain", "Jain fairness", 3, +1)]),
    ("W3_13", "Mixed, weights 1:3", [("ch0_gbps", "NIC GB/s", 3, +1), ("ch1_gbps", "ML GB/s", 3, +1),
                                     ("pkt_lat99", "NIC p99 (ns)", 0, -1), ("share0", "NIC share", 3, 0), ("jain", "Jain fairness", 3, +1)]),
    ("W3_17", "Mixed, weights 1:7", [("ch0_gbps", "NIC GB/s", 3, +1), ("ch1_gbps", "ML GB/s", 3, +1),
                                     ("pkt_lat99", "NIC p99 (ns)", 0, -1), ("share0", "NIC share", 3, 0), ("jain", "Jain fairness", 3, +1)]),
    ("W3_N300", "Mixed 1:3, noise 300 ppm bursts of 4 (stress)", [("ch1_gbps", "ML GB/s", 3, +1), ("pkt_lat99", "NIC p99 (ns)", 0, -1), ("replays", "replays", 0, -1)]),
    ("W3_N1K", "Mixed 1:3, noise 1000 ppm (stress)", [("ch1_gbps", "ML GB/s", 3, +1), ("pkt_lat99", "NIC p99 (ns)", 0, -1), ("replays", "replays", 0, -1)]),
]


def load(f):
    if f == "-" or not Path(f).exists():
        return None
    d = json.loads(Path(f).read_text())
    return next(iter(d.values()))


def vivado(f):
    if f == "-" or not Path(f).exists():
        return None
    t = Path(f).read_text()
    r = {}
    m = re.search(r"commit (\w+)", t); r["commit"] = m.group(1) if m else "?"
    for k, pat in (("lut", r"LUTs\s+(\d+)"), ("ff", r"Registers\s+(\d+)"), ("bram", r"BRAM_tiles\s+(\d+)")):
        m = re.search(pat, t); r[k] = int(m.group(1)) if m else None
    m = re.search(r"clk\s+WNS\s+(-?[\d.]+) ns at ([\d.]+) ns\s+-> Fmax ~ (\d+) MHz\s+levels (\d+)", t)
    if m:
        r["wns"], r["period"], r["fmax"], r["levels"] = float(m.group(1)), float(m.group(2)), int(m.group(3)), int(m.group(4))
    m = re.search(r"clk\s+failing endpoints (\d+).*?TNS (-?[\d.]+)", t)
    if m:
        r["fail"], r["tns"] = int(m.group(1)), float(m.group(2))
    m = re.search(r"aclk\s+WNS\s+(-?[\d.]+)", t); r["awns"] = float(m.group(1)) if m else None
    hist = re.findall(r"^\s+(\d+)\s+worst\s+(-?[\d.]+)\s+(\S+)$", t, re.M)
    r["hist"] = [(int(a), float(b), c) for a, b, c in hist if "clk" not in c]
    return r


def fmtv(v, nd):
    if v is None:
        return "&ndash;"
    return f"{v:,.{nd}f}" if nd else f"{v:,.0f}"


def ratio(a, b, better):
    if a in (None, 0) or b is None or better == 0:
        return ""
    r = b / a
    good = (r > 1) == (better > 0)
    txt = f"{r:.0f}&times;" if r >= 10 else (f"1/{1/r:.0f}" if r <= 0.1 else f"{(r-1)*100:+.0f}%")
    return f'<span class="{"good" if good else "bad"}">{txt}</span>'


def main():
    a = sys.argv[1:]
    loads, viv, out = [], [], Path("step4_report.html")
    i = 0
    while i < len(a):
        if a[i] == "--load":
            i += 1
            while i < len(a) and not a[i].startswith("--"):
                loads.append(load(a[i])); i += 1
        elif a[i] == "--vivado":
            i += 1
            while i < len(a) and not a[i].startswith("--"):
                viv.append(vivado(a[i])); i += 1
        elif a[i] == "--out":
            out = Path(a[i + 1]); i += 2
        else:
            i += 1
    while len(loads) < 3: loads.append(None)
    while len(viv) < 3: viv.append(None)

    # ---- hardware table ----
    hw_rows = [("commit", "commit", None, 0), ("clk WNS (ns, target 4.0 ns)", "wns", 3, +1), ("clk Fmax (MHz)", "fmax", 0, +1),
               ("logic levels on worst path", "levels", 0, -1), ("clk failing endpoints", "fail", 0, -1),
               ("clk TNS (ns)", "tns", 0, +1), ("aclk WNS (ns)", "awns", 3, +1),
               ("LUT", "lut", 0, 0), ("FF", "ff", 0, 0), ("BRAM tiles", "bram", 0, 0)]
    hw = []
    for lab, k, nd, better in hw_rows:
        vals = [v.get(k) if v else None for v in viv]
        if k == "commit":
            cells = "".join(f"<td class='n mono'>{html.escape(str(v)) if v else '&ndash;'}</td>" for v in vals)
            hw.append(f"<tr><td>{lab}</td>{cells}<td></td></tr>")
            continue
        cells = "".join(f"<td class='n'>{fmtv(v, nd)}</td>" for v in vals)
        last = next((v for v in reversed(vals) if v is not None), None)
        ch = ratio(vals[0], last, better) if k not in ("wns", "tns", "awns") else ""
        hw.append(f"<tr><td>{lab}</td>{cells}<td class='n'>{ch}</td></tr>")

    hist_html = ""
    for lab, v in zip(LABELS, viv):
        if v and v.get("hist"):
            rows = "".join(f"<tr><td class='n'>{n:,}</td><td class='n'>{w:.3f}</td><td class='mono'>{html.escape(m.split('/')[-1])}</td></tr>"
                           for n, w, m in v["hist"][:8])
            hist_html += f"<div><h3>{lab}: failing endpoints by module</h3><div class='tw'><table><tr><th>Endpoints</th><th>Worst ns</th><th>Module</th></tr>{rows}</table></div></div>"

    # ---- workload tables ----
    wl = []
    for sid, desc, mets in WL_ROWS:
        if not any(l and sid in l for l in loads):
            continue
        rows = []
        for k, lab, nd, better in mets:
            vals = [l[sid].get(k) if l and sid in l else None for l in loads]
            last = next((v for v in reversed(vals) if v is not None), None)
            rows.append(f"<tr><td>{lab}</td>" + "".join(f"<td class='n'>{fmtv(v, nd)}</td>" for v in vals) +
                        f"<td class='n'>{ratio(vals[0], last, better)}</td></tr>")
        ok = ["PASS" if (l and sid in l and l[sid]['pass']) else ("&ndash;" if not (l and sid in l) else "FAIL") for l in loads]
        rows.append("<tr><td>data check (every TLP, MSI-X order)</td>" + "".join(f"<td class='n'>{o}</td>" for o in ok) + "<td></td></tr>")
        wl.append(f"<section class='wl'><h3><span class='mono'>{sid}</span> {html.escape(desc)}</h3><div class='tw'><table>"
                  f"<tr><th>Metric</th>{''.join(f'<th>{x}</th>' for x in LABELS)}<th>base &rarr; latest</th></tr>{''.join(rows)}</table></div></section>")

    page = TEMPLATE.replace("{{HW}}", "".join(hw)).replace("{{HIST}}", hist_html).replace("{{WL}}", "".join(wl)) \
                   .replace("{{L0}}", LABELS[0]).replace("{{L1}}", LABELS[1]).replace("{{L2}}", LABELS[2])
    out.write_text(page)
    print(f"wrote {out}")


TEMPLATE = """<title>Step 4 Results</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans+Condensed:wght@600&display=swap">
<style>
/* Layout: one reading column; hardware table, then workload tables by scenario. */
:root{--bg:#f5f7f8;--panel:#fff;--ink:#18232b;--muted:#56646f;--rule:#d6dde2;--accent:#0f6e74;--good:#1e7a46;--bad:#b3261e;--code:#edf1f3;
--sans:"IBM Plex Sans",system-ui,-apple-system,"Segoe UI",sans-serif;--cond:"IBM Plex Sans Condensed","IBM Plex Sans",system-ui,sans-serif;--mono:"IBM Plex Mono",ui-monospace,Consolas,monospace}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#11171b;--panel:#182127;--ink:#e2e9ee;--muted:#9aa8b2;--rule:#2a363e;--accent:#6cc3c8;--good:#5cc38a;--bad:#f2847b;--code:#1f2a31;color-scheme:dark}}
:root[data-theme="dark"]{--bg:#11171b;--panel:#182127;--ink:#e2e9ee;--muted:#9aa8b2;--rule:#2a363e;--accent:#6cc3c8;--good:#5cc38a;--bad:#f2847b;--code:#1f2a31;color-scheme:dark}
body{background:var(--bg);color:var(--ink);font:15px/1.6 var(--sans)}
.wrap{max-width:960px;margin:0 auto;padding-block:40px 64px;padding-left:max(16px,env(safe-area-inset-left,0px));padding-right:max(16px,env(safe-area-inset-right,0px));display:grid;gap:30px}
h1{font:600 32px/1.15 var(--cond);margin:0;text-wrap:balance} h2{font:600 22px/1.25 var(--cond);margin:0;padding-bottom:6px;border-bottom:1px solid var(--rule)}
h3{font:600 15px var(--sans);margin:0 0 8px} p{margin:0;max-width:72ch} .lede{color:var(--muted)} .mono{font-family:var(--mono);font-size:.9em}
section{display:grid;gap:12px;min-width:0} .tw{overflow-x:auto;border:1px solid var(--rule);border-radius:6px;background:var(--panel)}
table{border-collapse:collapse;width:100%;font-size:14px} th,td{text-align:left;padding:7px 12px;border-bottom:1px solid var(--rule);vertical-align:top}
tr:last-child td{border-bottom:0} th{font-size:12px;text-transform:uppercase;letter-spacing:.05em;color:var(--muted);background:var(--bg)}
td.n{text-align:right;font-family:var(--mono);font-size:13px;font-variant-numeric:tabular-nums;white-space:nowrap}
.good{color:var(--good);font-weight:600} .bad{color:var(--bad);font-weight:600}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,420px),1fr));gap:16px}
.wl{gap:6px}
</style>
<div class="wrap">
<header style="display:grid;gap:10px"><div class="mono" style="color:var(--accent)">RoboKnights SemiCon &middot; Phase 2 &middot; Step 4 / Step 5</div>
<h1>Step 4 results: base, Wave A+B, Wave C</h1>
<p class="lede">Hardware numbers are Vivado 2026.1 on xcu250, out of context, place and route, clk target 4.0 ns (250 MHz). Workload numbers are from the Step 5 load analyser (Icarus, same checker as the regression), rates at 250 MHz. Every column passed the full regression before it was measured.</p></header>
<section><h2>Hardware (Vivado)</h2><div class="tw"><table><tr><th>Metric</th><th>{{L0}}</th><th>{{L1}}</th><th>{{L2}}</th><th>base &rarr; latest</th></tr>{{HW}}</table></div>
<div class="grid">{{HIST}}</div></section>
<section><h2>Workloads (Step 5)</h2>{{WL}}</section>
</div>
"""

if __name__ == "__main__":
    main()
