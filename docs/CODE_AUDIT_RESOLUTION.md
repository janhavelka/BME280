# Code Audit Resolution Report

Date: 2026-08-30

Audit reviewed: `docs/CODE_AUDIT.md`

Review baseline: `main` at `558a31f`, synchronized with `origin/main`

Completed-work baseline independently re-audited: `184eee9`

## Scope and method

Every item in the audit was checked against the current implementation, native
tests, examples, contract checkers, retained Bosch BME280 datasheet revision
1.24 (February 2024), and the repository's public compatibility rules. The
audit's proposed change was adopted only when it was both correct and the
smallest robust solution. Where a proposal would weaken device correctness,
break the 2.x API, add hidden I2C work, or solve an unreachable condition, the
item was retained, deferred, or resolved differently and is recorded below.

## Fresh independent re-audit of the completed work

The complete 951-line audit was reread, and the completed-work commit and its
full diff from `558a31f` were reviewed again without relying on the earlier
summary. Three parallel read-only reviews covered core requirements and edge
cases, tooling/examples and evidence semantics, and adversarial scope/API/test
quality. Their findings were then checked directly against the current source,
tests, generated command plans, and Bosch status semantics.

The re-audit and its final verification confirmed six gaps in the completed
work:

1. Init and non-reset resync still checked `im_update` and read calibration
   before quiescing a potentially normal-mode device. The unsafe proposal to
   skip `im_update` was still rejected, but the liveness/coherency issue is now
   fixed correctly: request sleep, confirm idle, retain the NVM gate, read
   calibration, then apply settings. Existing staged phases were rerouted, so
   no public enum value or worst-case callback cap changed.
2. Six HIL typed-setter callback profiles still described the old pre-readback
   implementation. They now assert the exact current profiles, and the parser
   suite checks every command-to-profile mapping.
3. Two mixed-stress plans considered `Restore status:` complete even though
   required `Health delta:` evidence is printed later. Both now wait for the
   final health line, with a chunked-serial regression test.
4. The embedded `parser_self_test()` duplicated a weaker subset of the
   maintained parser unit suite. It, its CLI flag, duplicate contract call, and
   duplicate documentation path were removed; the larger low-value module
   split remains deliberately deferred.
5. Two tests named as begin/recover NVM transport-error coverage injected a
   failure by register, so the new earlier idle-status read consumed it and
   allowed a false-positive. They now fail the exact later callback, and assert
   that identity and quiesce completed before the NVM status read failed.
6. Parallel final-gate dry-runs exposed a time-of-check/time-of-use race in
   timestamped artifact-directory creation. Allocation now validates the base
   path once, attempts each child name atomically, and retries only a concurrent
   child-name claim; deterministic regressions cover the race and invalid base.

The re-audit also strengthened enabled-sentinel coverage for temperature and
humidity, added warm-normal init/resync and caller-retry regressions, removed
the unreachable duplicate `APPLY_WAIT_IDLE` polling body while retaining its
public numeric enum value, and made a staged job's terminal root cause replace
only provisional dirty evidence created by that same job.

## Findings A-G: fixes already present at the review baseline

| Item | Disposition | Verification |
| --- | --- | --- |
| A | Confirmed fixed | The 12-bit humidity coefficient sign extension stays within defined integer ranges, and boundary decoding is covered by native tests. |
| B | Confirmed fixed | Package member normalization removes the exact `./` prefix instead of stripping a character set. |
| C | Confirmed fixed | Package validation requires the Arduino example and all of its `examples/common/` headers. |
| D | Confirmed fixed | The core timing guard directly bans framework timing calls/includes, including `delay`, `vTaskDelay`, and `esp_timer_get_time`. |
| E | Confirmed fixed | The CLI contract checker uses the actual command-handler grammar and a single command catalog. |
| F | Confirmed fixed and regression-tested | A zero reconnect budget returns without closing the serial handle. |
| G | Confirmed fixed | The Arduino diagnostic CLI still presents help and a prompt after I2C or driver bring-up fails, so recovery commands remain discoverable. |

## Core-driver findings

| Finding | Assessment and action |
| --- | --- |
| 1. Skipped-channel sentinels | **Valid; fixed with a simpler rule.** Bosch's disabled-channel sentinel values are also valid ADC codes when a channel is enabled. Sample validity now follows the configured oversampling selection. No conditional register read was added: that would create a surprising extra callback and violate staged callback budgeting. Defensive compensation guards remain. Tests cover enabled pressure, temperature, and humidity sentinel-shaped values plus actually skipped channels. |
| 2. Multi-byte register writes | **Valid; fixed.** BME280 I2C writes do not auto-increment across register addresses. Multi-register writes are encoded as repeated address/value pairs in a bounded stack buffer. Single-register writes keep their existing transaction. The fake bus now models pair decoding strictly, and tests check exact payloads and malformed writes. |
| 3. `im_update` outside reset | **Observation valid; fixed with a safer solution than proposed.** Skipping the gate could cache torn calibration while the image is being copied. Synchronous begin/recover and staged init/resync now request sleep and confirm idle before the retained bounded `im_update` gate and calibration bursts. This prevents normal-mode pulses from causing a liveness loop or racing calibration, while stuck NVM copy still reports observable `BUSY`/`TIMEOUT`. Warm-normal regressions cover all four paths. |
| 4. Apply while measuring | **Valid; fixed.** The driver issues the legal sleep request first and then checks idle before sleep-only work. If a synchronous queued transition is incomplete it returns `BUSY` for caller-directed retry; tests prove a later retry succeeds and clears dirty state. Staged jobs poll the same transition within their callback/deadline bounds. |
| 5. Lost `ctrl_hum` write | **Valid; fixed.** Individual setters use the same ordered sequence as whole-settings application, so `ctrl_hum` is always latched by the required following `ctrl_meas` write. |
| 6. Four duplicated sequences | **Valid, but the proposed always-write-full-tuple helper and late cache exposure were unnecessarily disruptive.** One shared selective apply engine now owns ordering, verification, and the documented cache/dirty transitions. It writes only the groups required by the requested change, avoiding needless `config` writes that reset IIR history. The re-audit split synchronous quiesce from post-quiesce writes so calibration can safely occur between them, and deleted the unreachable duplicate legacy wait body. Staged jobs retain the 2.x desired-settings contract. |
| 7. No configuration readback | **Valid; fixed.** A single `0xF2..0xF5` read verifies the owned bits after application. Reserved/status bits are ignored; the requested `ctrl_meas` mode is verified exactly. A mismatch preserves the original evidence in `Status::detail`, marks configuration dirty, returns `RESYNC_REQUIRED`, and has a distinct staged `APPLY_VERIFY` phase appended without renumbering existing phases. Transport failures remain distinguishable. |
| 8. Normal freshness tolerance | **Valid; fixed.** The private freshness budget includes Bosch's maximum standby tolerance (+25%, rounded up). Public timing accessors continue to report nominal configured timing. |
| 9. Discarded diagnostic messages | **Valid as an internal cleanup, overstated as an API defect.** Unused internal message arguments were removed where practical. The public message-bearing `Status` construction/overload remains for 2.x source compatibility and still stores only library-owned canonical static message text. |
| 10. Compatibility aliases | **Removal rejected for 2.x.** These aliases are public, used by repository consumers/examples, and therefore are compatibility surface regardless of their history. Removal belongs in a documented 3.x migration. The incorrect historical `REG_DIG_H5_LSB` wording is clarified. |
| 11. Duplicate `SettingsSnapshot` sample fields | **Valid design debt; deferred to 3.x.** Removing public fields would break source/ABI compatibility. The duplication is deterministic and does not threaten device correctness. |
| 12. `READY` while unsynchronized | **Not a driver-state defect; documentation strengthened.** `DriverState` intentionally describes observed transport/device health, while `ConfigSyncState` and `CalibrationState` describe whether a measurement is legal. A proposed `canMeasure()` boolean would hide other relevant conditions (mode, running job, pending measurement, and timebase), so it was not added. `isOnline()` now explicitly documents its limited meaning. |

### Finding 13: minor core items

| Item | Assessment and action |
| --- | --- |
| `_waitForNvmReady()` timeout | **Audit claim rejected.** A bounded transport callback can consume enough time to cross the deadline between the two clock reads, so the `TIMEOUT` branch is reachable. The branch, contract, and direct regression test remain. |
| Settings validation detail | **Valid; fixed.** Public `SettingsValidationReason` values identify the first invalid field deterministically and propagate through `begin()` and apply paths. |
| IIR history after skipped T/P | **Valid documentation gap; fixed.** Setter documentation and the register reference explain that skipped channels retain filter history and that explicitly changing the filter is the reset escape hatch. Selective setting writes also avoid unrelated filter resets. |
| Sleep-only `config` wording | **Valid; fixed.** The reference now uses the datasheet's "may be ignored" wording for normal-mode writes and labels the sleep/write/restore sequence as driver practice. |
| `0xE8..0xF1` range | **Valid ambiguity; documented.** `0xE1..0xE7` is the mapped humidity calibration block used by the driver. The reference notes the datasheet table's wider calibration-image label for `0xE8..0xF0`, its lack of a public byte mapping, and that the driver neither depends on nor writes it; `0xF1` is likewise left untouched. |

## Examples, tooling, and packaging findings

| Finding | Assessment and action |
| --- | --- |
| 14. Classification on truncated output | **Valid latent risk; fixed without retaining a second unbounded transcript.** Recovery classification now consumes structured parsed evidence, including driver state and hardware-dirty status. CSV and manifest behavior remains bounded, and regression coverage uses relevant evidence beyond the former 1,000-character tail. |
| 15. Job budget parser | **Parser defect valid, current result impact overstated; fixed.** Non-numeric verbs such as start/cancel/status are now recognized semantically as zero-callback operations. Existing row validators still enforce actual callback usage. |
| 16. `output_has_expected()` semantics | **Valid; fixed and renamed around completion evidence.** Explicit completion tokens govern read termination when supplied; otherwise `expected`/`expected_any` provide that evidence. Early idle handling now requires the applicable completion evidence, closing the reset-command gap, while final classification still independently enforces expected output. The re-audit also corrected both `stress_mix` plans to wait for the final required health line rather than the earlier restore line. |
| 17. `SKIPPED_UNSAFE` verdict | **Proposed behavior change rejected.** The state is emitted only during dry-run planning, whose verdict is already `INCOMPLETE`; it is unreachable in a live final verdict. Treating it as a new live failure would add dead policy. |
| 18. Checkout-directory-dependent IDF component | **Valid; fixed more simply.** The special ESP-IDF `main` component automatically depends on discovered components, so its hard-coded `REQUIRES BME280` entry was removed while explicit ESP-IDF dependencies remain. The contract checker prevents its reintroduction. |
| 19. Native stubs | **Both valid gaps fixed.** The fake sensor decodes repeated address/value writes rather than imaginary auto-increment writes. The Arduino Wire stub exposes the platform's `I2C_BUFFER_LENGTH` (128 for the pinned Arduino-ESP32 core), the example adapter uses that bound, and boundary tests cover 128/129-byte requests. |
| 20. Monolithic HIL runner | **Design debt confirmed; broad split deferred, simple deletions completed.** Moving tightly coupled plan/parser/live/reporting sections would add behavior-neutral risk to this correctness pass. The genuinely dead duration helper and the redundant 174-line embedded parser self-test were removed. `tools/test_run_i2c_hil_parser.py` is now the single maintained parser/classifier verification path. |

### Finding 21: minor example and tooling items

| Item | Assessment and action |
| --- | --- |
| HIL tombstone assertions | Removed. Maintained documents are checked for active contracts rather than names deleted in old releases. |
| Tautological reconnect test | Replaced with checks of the real argument parser's default and explicit override. |
| `getSettings()` return type | Deferred to 3.x because changing `Status` to `void` is a public source/ABI break. |
| Arduino self-test double count | Fixed by skipping restore when baseline capture did not succeed, matching the IDF behavior. |
| Raw mode bits `2` | No change. The typed CLI intentionally accepts canonical `Mode` values `0`, `1`, and `3`; raw register display still decodes both forced-mode encodings correctly. |
| CRLF empty-line behavior | Clarified with a source comment: suppressing a second prompt for the LF half of CRLF is intentional. |
| Health state color | Fixed to derive color from `DriverState`, not a failure counter that describes a different dimension. |
| IDF progress/input parity | Fixed. IDF stress jobs emit matching progress and accept the same 127-character maximum command, using a 129-byte buffer to distinguish an overlong line from a full legal line plus terminator. Contract checks cover both. |
| Short-read adapter mapping | Confirmed correct and already documented in the production shared-bus guide: the adapter returns the actual short count and lets the core classify `I2C_SHORT_TRANSFER` without inventing a transport cause. |
| PlatformIO inheritance noise | Removed redundant `extends = env`; host-native configuration clears embedded-only debug, upload, and monitor settings. |
| README include path | Corrected to the explicit repository path `examples/common/I2cTransport.h`. |
| Datasheet in packages | Retained deliberately. The PDF is primary source evidence and the curated package contract explicitly includes maintained documentation; changing distribution/storage policy was outside this correctness pass. |
| Retained HIL transcripts | Kept as documented evidence. `.gitignore` now allow-lists only the three maintained run IDs, preventing accidental addition of new large transcripts without an explicit retention decision. |

## Additional repository hygiene

The preceding audit commit accidentally tracked a top-level generated CMake
`build/` tree containing cache files, compiler-identification binaries, and a
configure-error response. The generated tree was removed from version control
and `/build/` was added to `.gitignore`. The files are recoverable from Git and
reproducible by configuring the project; no source or retained evidence was
deleted.

## Validation boundary

The integrated change set passed:

- 191/191 native Unity tests through `scripts/pio.cmd`;
- 87/87 HIL parser/tooling unit tests;
- core timing, CLI, HIL, ESP-IDF example, release-metadata, version-sync, and
  package-content contract checks;
- Python bytecode compilation for the maintained HIL/release tools;
- strict Doxygen generation;
- pinned Arduino-ESP32 builds for both `esp32s3dev` and `esp32s2dev`;
- PlatformIO package creation and curated archive-content validation;
- standard and staged-job HIL dry-runs, both reporting the expected
  `INCOMPLETE` verdict because no physical run was requested; and
- `git diff --check`.

The native ESP-IDF build was not run because `idf.py`/`IDF_PATH` is unavailable
in this environment. No sensor was connected and no live serial/HIL,
electrical-bus, accuracy, or long-duration validation was performed. The
retained historical transcripts do not validate this commit.
