# CI and the regression gate (Phase 2, Step 3 A)

Every push and pull request runs `.github/workflows/regression.yml` on GitHub:

| Job | What it checks | Time |
|---|---|---|
| guards | `ci/check_protected.py`: no past fix silently undone (`ci/protected_lines.txt`), every file in `bridge_top.f` / `files.f` exists, no module defined twice | seconds |
| sim / link | all link-layer benches in `src-link_layer/hdl/files.f` | ~10 min |
| sim / interconnect | the 4 interconnect unit benches | <1 min |
| sim / phy | `protocol_PHY_layer/run_all.py` | ~5 min |
| sim / cocotb | `protocol_PHY_layer/cocotb` (9 tests) | ~2 min |
| sim / bridge | `tb_bridge_top`, `tb_bridge_ovf`, `tb_ingest_overflow_leak` | <1 min |
| sim / errors | `tb_stream_path` with bit-error injection and random backpressure | ~2 min |
| sim / asic | same benches with the smaller ASIC buffer sizes | ~2 min |
| long | `tb_bridge_top` 6 sessions x 50 beats (pull requests + nightly) | ~2 min |
| formal | every `.sby` proof (OSS CAD Suite) | varies |

The jobs run in parallel after `guards`. Results show in the PR's Checks tab and
as a table on each job's summary page.

## Run it locally before pushing

Needs Python 3 and Icarus Verilog 12 on PATH (cocotb: `pip install cocotb==2.1.0`).

    python3 ci/run_regression.py                 # everything except long + formal
    python3 ci/run_regression.py guards bridge   # just some suites
    python3 ci/check_protected.py                # the 1-second guard only

ModelSim stays the day-to-day simulator; CI uses Icarus because GitHub's runners
have no ModelSim licence. Every bench must print `RESULT: PASS` or `RESULT: FAIL`.

## Making it a merge gate (one-time, repo admin)

GitHub -> Settings -> Branches -> Add rule for `main`:
- Require a pull request before merging
- Require status checks to pass: select `guards` and every `sim / ...` job
- (optional) Require branches to be up to date before merging

## When a guard fails

`[G1] ...: required line is gone` means a fix from an earlier ticket is missing,
usually because the file was edited from an old copy. Restore it (`git log -p --
<file>` shows the commit that removed it). If the code was deliberately
redesigned, update the rule in `ci/protected_lines.txt` in the same PR and say
why. Add a rule for every new fix that must never regress.
