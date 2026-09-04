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
- `MIGRATION_3X.md`: the backlog of deliberate 3.x breaking changes, with the
  coordinated edits each one needs.
- `CODE_AUDIT.md`: the code audit, with each finding's verified status and the
  corrections made to the audit itself. Internal engineering record; not part
  of the distributed package.
- `BME280_datasheet.pdf`: Bosch BME280 datasheet used as primary device
  evidence. Verify critical values against the PDF when changing protocol,
  compensation, timing, or electrical contracts.

## Generated and Local Artifacts

These paths are generated or local and are not release evidence by default:

- `docs/doxygen/` - generated HTML API documentation;
- `hil_logs/<run>/serial_transcript.txt` - selected raw hardware evidence,
  retained when the corresponding ledger row is maintained; the release set is
  the clean-source campaign, expanded long campaign, and final flashed gate;
- other files under `hil_logs/` - generated plans, summaries, matrices, CSVs,
  manifests, and checklists;
- `.pio/` - PlatformIO builds, dependencies, and disposable dry-run plans;
- `BME280-*.tar.gz` - generated package archives.

Dry-run plans and reproducible derived HIL reports may be deleted. Retain each
selected raw transcript and its concise result/evidence boundary in
`HARDWARE_VALIDATION.md`. A formal qualification claim additionally requires
the complete package, including its manifest and supporting artifacts, in
durable tracked storage or an immutable release asset. The tracked Bosch
datasheet and maintained register reference are source evidence and must not
be treated as generated documentation.

Software fault injection proves driver behavior at the transport callback
boundary; it does not prove electrical bus behavior. Generated serial summaries
likewise do not prove physical wiring, bus margin, sensor accuracy, humidity
handling, or field stability. `HARDWARE_VALIDATION.md` defines the release
scope and records only claims supported by retained evidence.
