# BME280 Final Merge Gate Report

Date: 2026-06-02

## Repository State

| Field | Value |
| --- | --- |
| Branch | `hardening/bme280-industry-gap-closure` |
| Gate verification HEAD | `d25937b1370c41351ba3d4eec9a9560113572308` |
| Describe | `v1.6.1-7-gd25937b` |
| Upstream | `origin/hardening/bme280-industry-gap-closure` |
| Local/remote sync | Not synced; local branch is `ahead 2` |
| Remote branch HEAD | `6b517a59666c10a7350f21ed98d00b67a3e3db19` |
| Clean tree | No |

Untracked local docs present at gate start:

- `docs/BME280_GAP01_GAP05_HIL_REPORT_20260602.md`
- `docs/BME280_GAP_CLOSURE_BASELINE_REPORT.md`

These are documentation artifacts, not ignored build or HIL artifacts. Resolve
them before claiming a clean release tree.

## CI Status

- `gh` CLI was not installed in this shell.
- Public GitHub API check found no open PR for
  `janhavelka:hardening/bme280-industry-gap-closure`.
- Public GitHub API check found `total_count=0` GitHub Actions runs for this
  branch.
- Current local HEAD is not pushed, so current-head CI cannot have run.

Conclusion: **CI has not run for current HEAD. Do not claim merge readiness.**

CI configuration review: `.github/workflows/ci.yml` includes PlatformIO Arduino
builds for `esp32s3dev` and `esp32s2dev`, native tests, core timing guard, CLI
contract, HIL contract, IDF example contract, generated version check, release
metadata check, Doxygen generation, package pack/check, and ESP-IDF example
builds for `esp32s3` and `esp32s2` through `espressif/esp-idf-ci-action`.

## Version Consistency

| File | Observed version/status |
| --- | --- |
| `library.json` | `1.7.0` |
| `idf_component.yml` | `version: "1.7.0"` |
| `include/BME280/Version.h` | `BME280_VERSION_STRING "1.7.0"`, `VERSION_MINOR = 7`, `VERSION_CODE = 10700` |
| `Doxyfile` | `PROJECT_NUMBER = "1.7.0"` |
| `CHANGELOG.md` | Has `1.7.0`, but `[Unreleased]` still contains entries |
| `README.md` | Release wording uses validation-boundary language and does not claim physical hardware completion |

Version metadata is aligned at `1.7.0`, but release/tag readiness is blocked
until the non-empty `[Unreleased]` section is intentionally handled for the
intended `1.7.0` release or a later version.

## Local Gate Results

| Check | Result |
| --- | --- |
| `python tools/check_core_timing_guard.py` | PASS: `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS: `CLI contract PASSED` |
| `python tools/check_hil_contract.py` | PASS: `HIL contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS: `IDF example contract PASSED` |
| `python scripts/generate_version.py check` | PASS: `Version.h` up to date |
| `python -m platformio test -e native` | PASS: 97 test cases, 97 succeeded |
| `python -m platformio pkg pack` | PASS: generated `BME280-1.7.0.tar.gz` |
| `python tools/check_package_contents.py` | PASS: `Package contents PASSED (BME280-1.7.0.tar.gz)` |
| `git diff --check` | PASS |

Generated package archive was removed after validation.

## Package Result

Package validation passed for `BME280-1.7.0.tar.gz`.

The package checker requires the IDF example files discovered from the native
IDF example CMake/source graph. A direct spot check confirmed these files were
present in the archive:

- `examples/idf/basic/main/IdfI2cTransport.cpp`
- `examples/idf/basic/main/IdfI2cTransport.h`

## HIL Status

No passing default HIL result exists for the current post-fix branch state.

Latest relevant HIL evidence:

- `hil_logs/i2c_20260602_104703/summary.md`: completed default run after
  tightening `cfg`/`calib` evidence, verdict `FAIL`, failing command
  `identity-calibration / cfg`.
- `hil_logs/i2c_20260602_104924/serial_transcript.txt`: final post-read-loop
  attempt could not open `COM16`; no HIL verdict was generated.

HIL claim boundary:

- Do not claim a successful default HIL run.
- Do not claim environmental accuracy validation.
- Do not claim long soak, destructive fault, or shared-bus physical validation.
- Hardware evidence remains limited to recorded transcripts and operator review
  status.

## Documentation Claim Review

Claim scan result: maintained docs do not make affirmative claims of field
deployment proof, hardware qualification, current default-HIL success, or local
pure ESP-IDF success. The README and maintained reports use validation-boundary
language around physical hardware and IDF checks.

Safe release wording:

- Software-hardened BME280 driver with injected, non-owning I2C transport.
- HIL tooling and evidence capture are present.
- Local software checks and package validation passed.
- Physical HIL success, field qualification, and environmental accuracy are not
  claimed unless a matching hardware matrix and artifacts record them.
- Local pure ESP-IDF `idf.py` validation is not claimed unless exact local
  command output is recorded.

Unsafe claims to avoid:

- Field deployment proof.
- Hardware qualification.
- Default HIL success for current HEAD.
- Local pure ESP-IDF success.
- Release/tag readiness before current-head CI runs.

## Remaining Limitations

- Worktree was not clean due to two untracked docs.
- Branch is not pushed/synced; local HEAD is ahead of remote.
- No PR is open for this branch.
- No GitHub Actions runs exist for this branch.
- `CHANGELOG.md` still has non-empty `[Unreleased]` entries.
- No passing post-fix default HIL run is available.
- No local pure ESP-IDF `idf.py` build was run in this final gate.

## Final Recommendation

Ready for PR: **No, not yet.**

Resolve untracked docs and push the branch first. Then open a PR and let CI run
on the current branch head.

Ready for merge after CI: **Not yet.**

Merge should wait until the branch is pushed, PR CI passes on current HEAD, the
tree is clean, and the release/changelog decision is reconciled.

Ready for tag: **No.**

Tagging `v1.7.0` should wait until:

1. `CHANGELOG.md` release contents are final.
2. Current HEAD is pushed.
3. PR CI passes on the exact release commit.
4. A successful/default HIL claim is either captured or explicitly excluded from
   release claims.
5. The repository is clean except ignored generated artifacts.

## PR And Release Checklist

Prepare but do not run tags until explicitly approved:

```text
git status --short
# Resolve untracked docs first.
git push origin hardening/bme280-industry-gap-closure
# Open/update PR and wait for CI on the pushed HEAD.
git checkout <target-branch>
git pull --ff-only
git merge --ff-only hardening/bme280-industry-gap-closure
python scripts/generate_version.py check
python -m platformio pkg pack
python tools/check_package_contents.py
Remove-Item -LiteralPath BME280-1.7.0.tar.gz
git diff --check
git tag -a v1.7.0 -m "Release v1.7.0"
git push origin v1.7.0
```
