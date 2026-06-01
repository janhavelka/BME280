# BME280 Gap-Closure Baseline Report

Date: 2026-06-01

## Start State

- Starting branch: `main`
- Starting HEAD: `001790ca0306011506ffc3af5bba97e194944629`
- Starting describe: `v1.5.0-25-g001790c-dirty`
- New branch created: yes, `hardening/bme280-industry-gap-closure`

Recent starting commits:

```text
001790c fix: correct ESP-IDF example log macro
d17a8dc ci: align ESP-IDF build target version
3fc82ca docs: update documentation for clarity and maintainability; add README map
32663b5 Merge pull request #2 from janhavelka:hardening/bme280-industry-readiness
047cbcb Fix I2C HIL runner output matching
d7929ec docs: consolidate BME280 industry readiness docs
4683368 delete obsolete documentation files: BME280_PHASE_05_EXAMPLES_IDF_CI_DOCS_REPORT.md, BME280_PRE_HIL_READINESS_REPORT.md, BME280_PROMPTS_00_06_COMPREHENSIVE_REPORT.md, I2C_HIL_SELFTEST_REPORT.md; add BME280_INDUSTRY_HARDENING_SUMMARY.md to summarize industry readiness; update check_hil_contract.py to include new summary checks and remove references to deleted reports; modify IDF_PORT_IMPLEMENTATION.md for clarity on ESP-IDF checks; ensure consistency in command execution across documentation and scripts.
ceec963 Add I2C HIL self-test runner
da5946a hardening: update I2C HIL self-test report with new dry-run timestamp
038df90 Add BME280 I2C HIL runner and documentation
e4e859c hardening: prepare BME280 for HIL validation
2c4098e hardening: finalize BME280 industry-standard report
```

## Review Agents

This hygiene pass emulated the requested scoped roles:

- `repo-hygiene-agent`: classified dirty and untracked files, separating intentional docs cleanup from premature release and HIL-sequence work.
- `docs-staleness-agent`: identified stale reports, generated Doxygen output, package archives, and documentation references that needed neutral baseline wording.
- `release-baseline-agent`: verified version metadata state and confirmed the release bump should not be performed in Prompt 00.
- `integration-review-agent`: checked that this pass stayed hygiene-only, with no driver refactor, public API change, or HIL execution.

## Dirty Files Found

The worktree changed during the hygiene pass, so this table records every dirty path observed and how it was resolved.

| Path | Classification | Resolution |
| --- | --- | --- |
| `CHANGELOG.md` | Release-prep metadata plus CRLF/stat churn | Premature `1.6.0` release-section changes were restored to the current development state. No intentional content change remains for this file. |
| `Doxyfile` | Documentation hygiene | Kept strict Doxygen warning settings: undocumented items and missing parameter docs now warn, warnings are errors, and output is no longer quiet. `PROJECT_NUMBER` remains `1.5.0`. |
| `docs/BME280_HARDWARE_VALIDATION_MATRIX.md` | Later HIL-sequence work plus CRLF/stat churn | HIL sequence additions were restored because Prompt 00 is baseline-only. No intentional content change remains for this file. |
| `docs/BME280_INDUSTRIAL_READINESS_EXPLORATION_REPORT.md` | Stale intermediate audit artifact | Deleted. Its findings are superseded by maintained docs and this baseline report. |
| `docs/BME280_INDUSTRY_HARDENING_SUMMARY.md` | Documentation wording cleanup | Kept neutral release-gate wording: generated Doxygen is local, and public docs should be published only from a release commit whose metadata and generated header agree. |
| `docs/I2C_HIL_RUNBOOK.md` | Later HIL-sequence work plus CRLF/stat churn | HIL sequence additions were restored because Prompt 00 is baseline-only. No intentional content change remains for this file. |
| `docs/README.md` | Previously release-critical untracked doc | Already tracked in current `HEAD`; no action required in this prompt. |
| `idf_component.yml` | Premature release metadata | Restored to `1.5.0`. Version bump is left for a release prompt. |
| `include/BME280/BME280.h` | Public-header documentation hygiene | Kept Doxygen-only `@param`/`@return` comments. No API signature or behavior changes. |
| `include/BME280/CommandTable.h` | Public-header documentation hygiene | Kept Doxygen groups and constant comments. Register values and names are unchanged. |
| `include/BME280/Config.h` | Public-header documentation hygiene | Kept enum-value Doxygen comments. Enum values are unchanged. |
| `include/BME280/Status.h` | Public-header documentation hygiene | Kept Doxygen comments for fields and helpers. Struct layout and behavior are unchanged. |
| `include/BME280/Version.h` | Generated release metadata | Restored to generated `1.5.0` state from `library.json`. |
| `library.json` | Release source of truth | Restored to `1.5.0`. No version bump in Prompt 00. |
| `tools/run_i2c_hil.py` | Later HIL-sequence work | Restored because Prompt 00 does not run or modify HIL behavior. The forced-mode evidence gap remains for Prompt 01. |

No non-ignored untracked artifact files remained after cleanup. This baseline report is intentionally added and committed. Ignored local artifact directories still exist where expected, including `.pio/`, `.vscode/`, `hil_logs/`, and Python `__pycache__/` directories.

## Generated Artifacts

- `BME280-1.5.0.tar.gz` and `BME280-1.6.0.tar.gz`: removed as local package archives; package archives are not tracked by repo policy.
- `.pio/`: ignored build output; not committed.
- `docs/doxygen/`: removed as generated local documentation output.
- `hil_logs/`: ignored local HIL output; not promoted as validation evidence.
- `tools/__pycache__/`: ignored Python cache output; not committed.

## Version Metadata Baseline

Current metadata intentionally remains at `1.5.0`:

| File | Value |
| --- | --- |
| `library.json` | `"version": "1.5.0"` |
| `idf_component.yml` | `version: "1.5.0"` |
| `Doxyfile` | `PROJECT_NUMBER = "1.5.0"` |
| `include/BME280/Version.h` | `BME280_VERSION_STRING "1.5.0"`, `VERSION_MAJOR = 1`, `VERSION_MINOR = 5`, `VERSION_PATCH = 0`, `VERSION_CODE = 10500`, `VERSION_INT = 10500` |

Current describe before the baseline commit: `v1.5.0-25-g001790c-dirty`.

There is no `v1.6.0` tag in the local tag list. Release-version selection and changelog finalization remain out of scope for Prompt 00.

## Clean Baseline Status

This prompt resolves the dirty worktree by committing the intentional hygiene changes on `hardening/bme280-industry-gap-closure`. After the baseline commit, `git status --short` should be empty and `git describe --tags --dirty --always` should report a clean post-`v1.5.0` commit.

## Checks

Checks requested for this prompt:

```text
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python tools/check_hil_contract.py
python scripts/generate_version.py check
git diff --check
python -m platformio test -e native
```

Results:

| Check | Result |
| --- | --- |
| `python tools/check_core_timing_guard.py` | PASS: `Core timing guard PASSED` |
| `python tools/check_cli_contract.py` | PASS: `CLI contract PASSED` |
| `python tools/check_idf_example_contract.py` | PASS: `IDF example contract PASSED` |
| `python tools/check_hil_contract.py` | PASS: `HIL contract PASSED` |
| `python scripts/generate_version.py check` | PASS: `Version.h` up to date |
| `git diff --check` | PASS: exit code 0; Git printed CRLF replacement warnings for dirty files |
| `python -m platformio test -e native` | PASS: 88 test cases, 88 succeeded in 15.23 seconds |

## Remaining Work

For Prompt 01:

- Fix the HIL forced-mode evidence gap by adding post-`force` `status` and/or `reg 0xF4` capture to the runner, runbook, and matrix.
- Add `tools/check_hil_contract.py` to CI.
- Harden package validation so the ESP-IDF transport `.cpp` and `.h` are required in package contents.
- Decide raw diagnostic write dirty-state semantics.
- Decide sample freshness/generation handling after `recover()`.
- Prepare a separate release prompt for SemVer, version metadata, generated `Version.h`, changelog release section, tag, and release wording.
- Keep hardware validation claims blocked until real HIL evidence is recorded.
