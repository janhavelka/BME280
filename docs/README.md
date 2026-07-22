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
  ledger, and historical-result boundary.
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
- `hil_logs/` — HIL plans and run artifacts;
- `.pio/` — PlatformIO builds and dependencies;
- `BME280-*.tar.gz` — generated package archives.

Software checks and generated serial summaries do not prove physical wiring,
bus margin, sensor accuracy, humidity handling, fault recovery, or field
stability. Use `HARDWARE_VALIDATION.md` and record `NOT RUN` or `unknown` for
facts that were not observed.
