# Planned 3.x Breaking Changes

Everything here is a deliberate deferral: valid to change, but source- or
ABI-breaking, so it waits for a major version. Nothing in this list is a defect
in 2.x — the current behaviour is correct, just wider than it needs to be.

Keep this file as the single backlog for 3.0 planning. Add to it whenever a 2.x
review declines a change purely because it breaks compatibility.

## Remove compatibility aliases

Each of these is a second spelling of something else. Four are unreferenced
outside their own definition and can be deleted with a header edit. **Three are
load-bearing** and need coordinated changes across examples and CI checkers —
that is the reason they survived 2.x, not oversight.

| Symbol | Canonical form | What else must change |
|---|---|---|
| `Err::CONVERSION_NOT_READY` (`Status.h`) | `Err::MEASUREMENT_NOT_READY` | nothing |
| `JobKind::RECOVERY` (`BME280.h`) | `JobKind::RESYNC` | nothing |
| `BME280::driverState()` (`BME280.h`) | `BME280::state()` | nothing |
| `cmd::REG_DIG_H5_LSB` (`CommandTable.h`) | `cmd::REG_DIG_H5_MSB` | nothing. **Also misnamed**: `0xE6` holds `dig_H5[11:4]`, the *high* bits (datasheet Table 16). The low nibble is in `0xE5[7:4]`, correctly named `REG_DIG_H4_H5`. |
| `BME280::startRecoveryJob()` | `BME280::startResyncJob()` | both shipped examples, 11 tests, and the mandatory-token lists in `tools/check_cli_contract.py` and `tools/check_idf_example_contract.py` |
| `JobPollResult::instructionsUsed` | `JobPollResult::callbacksUsed` | both shipped examples print it; the `"Instructions:"` output token is required by `tools/check_idf_example_contract.py` |
| `VERSION_INT` (`Version.h`) | `VERSION_CODE` | emitted by `scripts/generate_version.py` and asserted verbatim by `tools/check_release_metadata.py` |

## Drop the duplicated `SettingsSnapshot` sample fields

`SettingsSnapshot` carries six fields that `SampleEnvelope sample` already
contains: `sampleTimestampMs`, `tFine`, `rawSample`, `compSample`,
`sampleSequence` and `sampleConfigGeneration`. That is roughly 48 bytes of the
struct duplicated, filled from the same members by `getSettings()`.

They agree today and nothing enforces that they keep agreeing. Keep
`SampleEnvelope sample` and delete the six flattened copies.

## Make `getSettings()` return `void`

`BME280::getSettings(SettingsSnapshot&)` cannot fail — the implementation
unconditionally returns `Status::Ok()` and the header documents
"`Status::Ok()` always". The four call sites in the examples correctly write
`(void)device.getSettings(...)`, but the signature invites the reader to think a
failure is possible. Change it to `void`.

## Consider: split `tools/run_i2c_hil.py`

Not an API break, but the same "wait for a natural boundary" situation. The
runner is ~4700 lines doing six unrelated jobs; the proposed split is the
command catalogue, the artifact writers, and the parsers into separate modules.

The blocker is that `tools/check_hil_contract.py` asserts twenty literal
substrings against the runner's *source text*, and the CLI/IDF contract checkers
do the same to the two `main.cpp` files. Any refactor trips those for reasons
unrelated to behaviour, so the checkers must be reworked in the same change.
