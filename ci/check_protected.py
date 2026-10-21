#!/usr/bin/env python3
"""
ci/check_protected.py -- static guards that run before any simulation.

1. Protected lines (ci/protected_lines.txt): every past fix that must not be
   silently undone is still present (or a known-bad pattern is still absent).
2. File lists: every file named in bridge_top/bridge_top.f and
   src-link_layer/hdl/files.f exists.
3. Duplicate modules: no module name is defined twice in the bridge_top build
   (catches "file A accidentally holds a copy of module B", as happened with
   link_layer_top.v in Phase 2).

Usage:  python3 ci/check_protected.py      (from anywhere; exit 0 = all good)
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
errors = []


def norm(s):
    return re.sub(r"[ \t]+", " ", s.replace("\r", "")).strip()


def read(p):
    return (ROOT / p).read_text(errors="replace")


# ---- 1. protected lines ----
rules = 0
for n, raw in enumerate(read("ci/protected_lines.txt").splitlines(), 1):
    if not raw.strip() or raw.lstrip().startswith("#"):
        continue
    parts = [x.strip() for x in raw.split("|", 3)]
    if len(parts) != 4 or parts[2] not in ("require", "forbid"):
        errors.append(f"protected_lines.txt:{n}: malformed rule")
        continue
    rid, path, kind, text = parts
    rules += 1
    f = ROOT / path
    if not f.exists():
        errors.append(f"[{rid}] {path}: file missing")
        continue
    # comments are stripped first, so a rule can't be satisfied by a comment
    # that merely mentions the old code
    body = "\n".join(norm(l.split("//", 1)[0]) for l in f.read_text(errors="replace").splitlines())
    found = norm(text) in body
    if kind == "require" and not found:
        errors.append(f"[{rid}] {path}: required line is gone -> {text}")
    if kind == "forbid" and found:
        errors.append(f"[{rid}] {path}: forbidden pattern is back -> {text}")

# ---- 2. file lists ----
def bridge_list():
    out = []
    for l in read("bridge_top/bridge_top.f").splitlines():
        l = l.strip()
        if l and not l.startswith("//") and not l.startswith("+"):
            out.append(l)
    return out

listed = bridge_list()
for p in listed:
    if not (ROOT / p).exists():
        errors.append(f"bridge_top.f: listed file missing: {p}")

hdl = ROOT / "src-link_layer/hdl"
phy = ROOT / "protocol_PHY_layer/rtl"
for l in read("src-link_layer/hdl/files.f").splitlines():
    l = l.split("#", 1)[0].strip()
    if not l:
        continue
    t = l.split()
    paths = [t[1]] if t[0] == "rtl" else t[2:] if t[0] == "tb" else []
    for p in paths:
        q = phy / p[4:] if p.startswith("PHY:") else hdl / p
        if not q.exists():
            errors.append(f"files.f: listed file missing: {p}")

# ---- 3. duplicate module definitions in the bridge build ----
seen = {}
mod_re = re.compile(r"^\s*module\s+([A-Za-z_][A-Za-z0-9_$]*)", re.M)
for p in listed:
    f = ROOT / p
    if not f.exists() or not p.endswith(".v"):
        continue
    for m in mod_re.findall(f.read_text(errors="replace")):
        if m in seen and seen[m] != p:
            errors.append(f"module '{m}' defined twice: {seen[m]} and {p}")
        seen.setdefault(m, p)

print(f"protected-line rules: {rules}, bridge files: {len(listed)}, modules: {len(seen)}")
if errors:
    print("\nFAILED:")
    for e in errors:
        print("  " + e)
    sys.exit(1)
print("All guards OK.")
