# BME280 Documentation Map

This directory keeps the maintained supporting documentation for the BME280
library. It is split into operator-facing docs, engineering reference docs, and
source evidence.

## Maintained Docs

- `../README.md`: public usage, API, build, validation, and limitations.
- `../CHANGELOG.md`: release-facing change history.
- `../AGENTS.md`: repository engineering rules for future changes.
- `IDF_PORT.md`: ESP-IDF component and native example boundary.
- `BME280_Register_Reference.md`: register, bitfield, timing, and calibration
  notes used by the implementation.
- `BME280_INDUSTRY_HARDENING_SUMMARY.md`: current summary of the hardening work
  and remaining release gates.
- `PRODUCTION_SHARED_BUS_GUIDE.md`: production shared-bus integration guidance
  for application-owned bus, locking, scheduling, timeout, and recovery policy.
- `I2C_HIL_RUNBOOK.md`: serial HIL procedure and evidence rules.
- `I2C_HIL_TARGET_TEMPLATE.md`: per-target evidence form for a physical run.
- `BME280_HARDWARE_VALIDATION_MATRIX.md`: committed hardware validation ledger.

The repository metadata is prepared for the `2.0.0` major release candidate.
`v1.7.0` remains the latest published immutable tag until the exact candidate
commit passes remote CI and is tagged. Keep release-facing history in
`CHANGELOG.md`; do not infer publication from metadata alone.

## Source Evidence

- `BME280_datasheet.pdf`: vendor datasheet used for implementation checks.
- `extracted-md/`: compact extracted notes used for quick review.
- `pdf-extracted-md/`: full extracted datasheet text.

The extracted markdown is not a second user manual. Treat it as reference
material when changing register behavior, compensation math, timing, or hardware
contracts.

## Audit Records

Prompt-scoped audit, phase, and merge-gate reports are intentionally not kept as
release-facing documentation. Durable conclusions from that work are folded into
the maintained docs above and the current release notes in `../CHANGELOG.md`.

## Local Artifacts

The following outputs are local artifacts and should not be committed:

- `docs/doxygen/`
- `hil_logs/`
- `.pio/`
- generated `BME280-*.tar.gz` package archives unless intentionally publishing
  a release artifact

## Validation Claims

Software checks verify build, test, package, and documentation contracts. They do
not prove physical wiring, pull-up values, sensor accuracy, humidity handling,
fault recovery, or long-duration stability.

Only update `BME280_HARDWARE_VALIDATION_MATRIX.md` with observed hardware
results. Use `NOT RUN` or `unknown` rather than guessing.
