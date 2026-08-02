# BME280 Documentation

This directory contains maintained integration, hardware-validation, and
chip-reference documentation. Public API details live in the Doxygen comments
under `include/BME280/`; release history lives in `../CHANGELOG.md`.

## Maintained Documents

- `../README.md`: installation, quick start, API overview, behavior, and
  validation commands.
- `IDF_PORT.md`: native ESP-IDF component and transport boundary.
- `PRODUCTION_SHARED_BUS_GUIDE.md`: application-owned bus, locking, scheduling,
  deadlines, and recovery guidance.
- `HARDWARE_VALIDATION.md`: the single HIL procedure, evidence schema, current
  status, and qualification boundary.
- `BME280_Register_Reference.md`: implementation-facing register, calibration,
  and timing notes.
- `BME280_datasheet.pdf`: Bosch BME280 datasheet used as primary device
  evidence. Verify critical values against the PDF when changing protocol,
  compensation, timing, or electrical contracts.

Completed hardening and product-fit audits are not maintained as parallel user
manuals. Their durable results are represented by the current code, public API
comments, `../README.md`, and `../CHANGELOG.md`; Git history retains the original
review records.

## Generated and Local Artifacts

These paths are intentionally ignored and are not release evidence by default:

- `docs/doxygen/` — generated HTML API documentation;
- `hil_logs/` — HIL plans and scratch run artifacts;
- `.pio/` — PlatformIO builds, dependencies, and disposable dry-run plans;
- `BME280-*.tar.gz` — generated package archives.

Dry-run plans may be deleted. Preserve every real or review-worthy HIL run as
one complete package—including its transcript, manifest, and supporting
artifacts—by promoting it to durable tracked storage or an immutable release
asset before cleaning `hil_logs/`. The tracked Bosch datasheet and maintained
register reference are source evidence and must not be treated as generated
documentation.

Software checks and generated serial summaries do not prove physical wiring,
bus margin, sensor accuracy, humidity handling, fault recovery, or field
stability. Use `HARDWARE_VALIDATION.md` and record `NOT RUN` or `unknown` for
facts that were not observed.
