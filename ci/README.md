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
| sim / rand | `tb_bridge_rand` (F1-F9 + F7b: random traffic, 4 KB splits, weights, PAM4 noise, overflow, lane reversal, hot reset, cable pull / glitch, MSI-X masking, BAR0 via inbound MWr), 4 seeds: 2 clean, 2 noisy, then the coverage table | ~6 min |
| sim / rand sweep | nightly: 40 more seeds | ~25 min |
| coverage | nightly: the 40 seeds' coverage merged by `ci/coverage_report.py --strict`; fails if a required bin was never hit | seconds |
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

## The merge gate (free private repo: no enforced rulesets)

GitHub only enforces branch rulesets on private repos for paid plans, so the
gate has two halves that need no plan:

1. **Before pushing: `.githooks/pre-push`.** Blocks direct pushes to `main` and
   runs the guard on every push. Each person enables it once per clone:

       git config core.hooksPath .githooks

   Emergency override: `ALLOW_MAIN_PUSH=1 git push ...` (PowerShell:
   `$env:ALLOW_MAIN_PUSH=1; git push ...; Remove-Item env:ALLOW_MAIN_PUSH`).
2. **After merging: `.github/workflows/main-guard.yml`.** Every push to `main`
   runs guards + interconnect + bridge + link + errors (~15 min). If any fails,
   `main` is put back to the previous commit with one "Auto-revert" commit and
   an issue names the commit, its author and the failing log.

Team rule on top: merge a pull request only when its Checks tab is green.

If the repo ever moves to a paid plan, public visibility, or the owner's GitHub
Pro (free in the GitHub Student Developer Pack), import
`main_protection_ruleset.json` instead and keep the hook as a fast local check.

## When a guard fails

`[G1] ...: required line is gone` means a fix from an earlier ticket is missing,
usually because the file was edited from an old copy. Restore it (`git log -p --
<file>` shows the commit that removed it). If the code was deliberately
redesigned, update the rule in `ci/protected_lines.txt` in the same PR and say
why. Add a rule for every new fix that must never regress.

## Coverage (Step 3 C)

Each `tb_bridge_rand` run prints one `COV:` line. `ci/coverage_report.py` merges
them into a table (BAR0 registers, 4 KB split / no split, last-beat sizes,
contested arbiter wins per weight, credit stalls, FEC/CRC/replay, XLAT_MISS,
overflow, MSI-X masks, hot reset, cable pull, glitch, L0 exit reasons, lane
order, inbound BAR0 writes). The table is on the job summary page. Locally:

    python3 ci/run_regression.py rand                      # writes ci_out/cov_1.txt + prints the table
    RAND_SEEDS=40 COV_STRICT=1 python3 ci/run_regression.py rand   # gate like the nightly job

Bins tb_bridge_rand cannot reach (PHY framing/lane errors, TX translation miss,
L0 exit on TS1) are marked optional and name the directed suite that covers them.
