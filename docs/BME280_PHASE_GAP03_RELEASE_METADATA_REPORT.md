# BME280 Phase GAP03 Release Metadata Report

Date: 2026-06-01

Branch: `hardening/bme280-industry-gap-closure`

Starting HEAD: `f00ad8184b886611212dce6ae9a4ce05eb6bc404`

Starting describe: `v1.6.1-2-gf00ad81`

Scope: Prompt 03 only. No HIL was run. No hardware validation is claimed.

## Chosen Version

Selected release version: `1.7.0`

## SemVer Rationale

Local tags already include `v1.6.0` and `v1.6.1`, and current metadata before
this prompt was synchronized at `1.6.1`. Reusing `1.6.0` or `1.6.1` would
conflict with existing release history.

The large post-`v1.5.0` public struct additions and hardening features were
already represented in the existing `1.6.0` changelog, so the repository history
has treated appended public fields and source-compatible hardening as a minor
release with migration warnings rather than a `2.0.0` reset. Current Prompt 01
and Prompt 02 changes add release-relevant HIL evidence contracts, package/CI
coverage, diagnostic raw-write dirty-state semantics, health-session wording,
NVM timing clarification, and recovery cache invalidation. Those are stronger
than a patch-only documentation change, so `1.7.0` is the next conservative
minor release.

Migration notes explicitly warn about public struct layout changes and
non-copyable driver instances for users upgrading from `v1.5.x`.

## Metadata Updated

- `library.json`: `"version": "1.7.0"`
- `idf_component.yml`: `version: "1.7.0"`
- `Doxyfile`: `PROJECT_NUMBER = "1.7.0"`
- `include/BME280/Version.h`: regenerated from `library.json`
- `CHANGELOG.md`: moved `[Unreleased]` content into `## [1.7.0] - 2026-06-01`
  and updated compare links

Version update command used:

```bash
python scripts/generate_version.py set 1.7.0
```

`scripts/generate_version.py update` is not supported by this repository; the
script supports `sync`, `check`, `bump`, and `set`.

## Tooling Coverage

No new release metadata checker was needed. Existing CI already runs both:

- `python scripts/generate_version.py check`, which verifies generated
  `Version.h` against `library.json`.
- `python tools/check_release_metadata.py`, which verifies `library.json`,
  `Version.h`, `idf_component.yml`, `Doxyfile`, and changelog release links are
  synchronized.

## Migration Notes Summary

The release notes and README now tell users upgrading from `v1.5.x` to:

- check per-channel validity flags before using measurement fields;
- rebuild dependent firmware because public sample/snapshot struct layouts have
  changed since `v1.5.0`;
- keep `BME280::BME280` instances owned in one place and pass references or
  pointers instead of copying/moving them;
- use typed setters for normal configuration and treat raw register writes as
  diagnostic escape hatches;
- resync with `recover()` or `begin()` after diagnostic raw writes to
  reset/control/config registers;
- request a fresh sample after successful `recover()` or any `softReset()`
  attempt before using cached sample data;
- avoid claiming local pure ESP-IDF validation unless exact `idf.py` build
  commands were run and recorded;
- keep hardware validation claims blocked until real board evidence exists.

## Checks Run

- `python scripts/generate_version.py check` - PASS: `Version.h` up to date
- `python tools/check_release_metadata.py` - PASS: `Release metadata PASSED (1.7.0)`
- `python tools/check_core_timing_guard.py` - PASS: `Core timing guard PASSED`
- `python tools/check_cli_contract.py` - PASS: `CLI contract PASSED`
- `python tools/check_idf_example_contract.py` - PASS: `IDF example contract PASSED`
- `python tools/check_hil_contract.py` - PASS: `HIL contract PASSED`
- `python -m platformio test -e native` - PASS: 97 test cases, 97 succeeded
- `python -m platformio run -e esp32s3dev` - PASS: `esp32s3dev SUCCESS`
- `python -m platformio run -e esp32s2dev` - PASS: `esp32s2dev SUCCESS`
- `python -m platformio pkg pack` - PASS: wrote `BME280-1.7.0.tar.gz`
- `python tools/check_package_contents.py` - PASS:
  `Package contents PASSED (BME280-1.7.0.tar.gz)`
- `idf.py --version` - NOT RUN: `idf.py` is not available in this shell, so
  local pure ESP-IDF builds were not run
- `git diff --check` - PASS: no whitespace errors; Git emitted expected CRLF
  working-copy warnings only

The generated `BME280-1.7.0.tar.gz` archive was removed after package
validation.

## Remaining Release Blockers

- Push the release-prep branch and wait for CI before tagging any release
  commit.
- Do not mark hardware matrix rows as passing until physical HIL artifacts
  record board, module, wiring, rails, pull-ups, command transcript,
  environmental reference, and operator sign-off.
- Local pure ESP-IDF builds remain unrecorded in this prompt because `idf.py`
  is unavailable in this shell; rely on CI or rerun locally where ESP-IDF is
  installed before claiming local pure ESP-IDF validation.
