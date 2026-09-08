# BME280 Code Audit

Audit of `include/BME280/*.h`, `src/BME280.cpp`, the examples, and the Python
tooling, against the Bosch BME280 datasheet BST-BME280-DS001-24 rev. 1.24
(`BME280_datasheet.pdf`, cover page and history table; the body-page footers
still read 1.23, which is Bosch's own stale footer).

| | |
|---|---|
| Audited baseline | `3b3fad1` (v2.1.0) |
| Fixes landed in | `558a31f`, `184eee9`, `de39968`, `d4d491f`, `99a0af7` |
| Previous re-verification | `084b045`, 2026-09-04, with coverage/docs follow-ups through `5837d0b` |
| Latest verification baseline | `30ad987`, 2026-09-08; changes from this pass described below |
| Items | 28 (A–G plus findings 1–21, with sub-items) |

Every finding was re-verified independently against the current source and
the relevant historical source, datasheet clauses, and tests. Several were
checked by compiling and running the driver. **The audit as
originally written contained errors.** They are corrected in place below, and
listed in full in [Corrections to this document](#corrections-to-this-document)
so the change history is not lost.

## 2026-09-08 verification report

Fetched all remotes with pruning and ran `git merge --ff-only origin/main`.
The working tree was clean; `main` at `30ad987` was already the newest remote
branch tip. Reviewed every A–G and 1–21 finding, all thirteen minor sub-items,
O1–O4, the withdrawn proposals, and the September 7 checker follow-ups. Three
parallel reviews covered core behavior, tooling, and examples/API contracts;
the integration review checked the diffs, compensation formulas against the
local Bosch PDF, and the validation results.

**Result:** the existing fixes use appropriate designs, but sleep verification,
two source checkers, and IDF health coloring still had gaps. These were reproduced
and fixed. An existing native regression was also strengthened to test the actual
failure contract. Public signatures, compensation math, and the release version
remain unchanged. This section records this pass; earlier dated sections retain
their historical results.

### Changes and simplest-solution decisions

- **3/5/7 — an idle status did not prove SLEEP.** Bosch §3.3.1 allows a pending
  mode transition to ignore a subsequent sleep request. Once the prior command
  completes into NORMAL, `measuring` can be clear during standby. The old
  readiness check accepted that state; a synthetic reproduction made `begin()`
  return OK after reading calibration in NORMAL mode. Final settings readback
  is too late to protect those earlier calibration reads. The existing readiness
  callback now burst-reads `status` and `ctrl_meas` together, requiring both idle
  and SLEEP before proceeding. A non-SLEEP idle mode returns the existing
  `RESYNC_REQUIRED` with packed `0xF4`/expected-zero/actual-mode evidence, marks
  dirty state, and leaves recovery to the caller. The staged path reuses the
  same helper, keeping callback caps, phases, and bounded BUSY waits intact.
  This extends the existing quiesce owner rather than adding a retry or a second
  callback. The fake sensor now exposes the ignored-write scenario and returns
  both bytes of the status/mode burst; regression evidence is listed below.
- **D — timing calls hidden by string contents.** The checker removed comments
  before recognizing strings. A line such as
  `const char* url = "https://example.com"; delay(1);` therefore passed the old
  checker: `//` inside the literal swallowed the real call. Paired `"/*"` and
  `"*/"` literals could similarly hide intervening code. A single combined regex
  now consumes strings, character literals, and comments in source order,
  replacing each with whitespace. This keeps the existing lightweight guard
  and avoids a parser dependency. Tests exercise the real checker entry point
  in both core directories, including legitimate timing text inside literals
  and comments as positive controls.
- **E/18 — help text masked missing IDF handlers.** The IDF checker unioned
  dispatch commands with aliases extracted from help text. Removing a handler
  while retaining its help entry could still pass. Removed the union and its
  now-unused helper; mandatory commands must appear in dispatch comparisons.
  Help parity remains separately checked. The regression removes each of the
  45 required IDF dispatch comparisons while leaving help intact. It replaces
  a test that removed both help and the handler, which concealed the defect.
- **6/O1 — inaccurate configuration-failure evidence.** Two native test names
  still described a sleep-write failure and mode restoration, although both
  injected at the subsequent `config` write. Renamed the timeout case and
  changed the data-NACK case to start in NORMAL mode, then verify exactly two
  attempted writes, only the successful sleep write in the device log, hardware
  SLEEP, unchanged cached settings, and the original error in both the return
  value and dirty-state diagnostic. This strengthens an existing test instead
  of adding another overlapping case. It proves this specific failure path;
  it does not promise hardware SLEEP after every possible failure.
- **10/21/O3 — stale migration claims.** `startRecoveryJob()` currently appears
  in 13 distinct native tests, not 11. The examples have 14 `getSettings()` calls,
  of which four explicitly discard the status. Updated the migration notes and
  O3 to describe the coordinated callers without fragile counts, and corrected
  the stale `_updateHealth()` source reference.
- **21 — IDF health color was still inconsistent.** Its `drv` output selected
  green for zero failures and yellow otherwise, displaying UNINIT as green and
  OFFLINE as yellow. A small local `stateColor(DriverState)` switch now matches
  the state: reset/green/yellow/red. No suitable IDF helper already existed;
  sharing Arduino glue would violate the native-IDF boundary. Existing output
  labels and formatting are preserved.

The source guards remain lightweight repository contract checks, not full C++
or CMake parsers. Their regression tests establish the concrete behavior above;
they do not establish exhaustive detection of every possible source spelling.

### Finding-by-finding disposition

Evidence below names current functions and tests rather than drifting line
numbers. **Retain** means the diagnosis and shipped resolution were checked
and no additional implementation change was justified.

| Finding | Current verification and decision |
|---|---|
| A | Retain range-safe subtraction in `signExtend12()`; signed calibration/nibble tests and strict conversion-warning compilation pass. |
| B | Retain exact `./` prefix removal and original tar-member lookup; flat, dot-prefixed, directory-prefixed, and forbidden-path archive regressions pass. |
| C | Retain all seven required example headers and independently specified package fixtures; missing-path tests pass. |
| D | Retain the simplified zero-platform-call rule; fix literal/comment tokenization as described above. |
| E | Retain the single Arduino command list and handler checks; close the equivalent remaining IDF help/dispatch gap. |
| F | Retain the zero-retry early return before `ser.close()`; parser-suite tests pin the untouched handle and cleanup behavior. |
| G | Retain unconditional help/prompt after bring-up and checked `Wire.begin()`/`setClock()` results; setup-failure/retry regression passes. Bus recovery remains application-owned. |
| 1 | Retain configuration-derived validity; enabled ADC sentinel values publish samples, configured-off channels remain invalid. |
| 2 | Retain bounded register/address pairs and range checks; exact-payload and malformed-range tests match Bosch §6.2.1. |
| 3 | Retain quiesce before NVM gating and add actual SLEEP readback before calibration. Skipping the gate remains an unsafe proposal. |
| 4 | Retain sleep write before the readiness check; the busy-device test proves caller retry can converge without an internal wait loop. |
| 5 | Retain `ctrl_hum` then `ctrl_meas` latching; now require verified SLEEP as well as idle first, including when a preceding mode command ignored the sleep write. |
| 6 | Retain one synchronous write owner plus the separately budgeted staged phases. Strengthen evidence that a failed config write stops without a mode-restoration write. |
| 7 | Retain final owned-bit readback and packed mismatch evidence; also check mode before protected accesses, because final verification cannot protect earlier calibration reads. |
| 8 | Retain the exact eight-entry +25% standby table; the all-standby and wraparound tests pass. Nominal public timing remains distinct from the freshness bound. |
| 9 | Retain code/detail-only construction and canonical static messages; keep legacy message signatures for 2.x compatibility. Ownership/copy tests pass. |
| 10 | Keep the seven compatibility aliases for 2.x; three have repository consumers. Deletion belongs in a coordinated major-version change. |
| 11 | Keep the six duplicate snapshot fields for compatibility; their later removal remains in `MIGRATION_3X.md`. |
| 12 | Retain the explicit health/readiness distinction in headers and README. The proposed partial `canMeasure()` predicate still overpromises and remains withdrawn. |
| 13.1 | Remains withdrawn: a transport callback can cross the NVM deadline. The existing wraparound regression exercises the reachable TIMEOUT branch. |
| 13.2 | Retain seven ordered validation reasons and propagation through `begin()`; exhaustive settings validation passes. |
| 13.3 | Retain the documented IIR history after skipping/re-enabling T/P, consistent with Bosch §3.4.4. |
| 13.4 | Retain normal-mode writes “may be ignored” wording and the explicit calibration-map discrepancy; do not resolve contradictory datasheet tables by assumption. |
| 14 | Retain full-output structured evidence for reclassification. Tests deliberately omit evidence from excerpts and still classify correctly. Historical output-size corrections are not new hardware measurements. |
| 15 | Retain verb-aware zero start/cancel budgets and poll/run budgets; parser-suite callback validators pass. |
| 16 | Retain separate completion and expected-result checks, `MATCHED_COMPLETION`, and the widened idle guard. Completion does not assert success. |
| 17 | Remains withdrawn: unsafe skips originate only in dry runs, whose verdict is already INCOMPLETE. No unreachable verdict branch added. |
| 18 | Retain absolute-path normalization before deriving the component name. Host CMake evaluation passes under `BME280`, `renamed-driver`, and `renamed driver`; existing positive/negative name guards remain pinned. |
| 19 | Retain pair-aware FakeBus writes, reset-register clearing, and the 128-byte Wire boundary; malformed writes, reset reapply, and 128/129-byte tests pass. |
| 20 | Retain dead-code removal; defer the multi-module HIL split. It fixes no demonstrated behavior defect and would require coordinated source-checker changes. |
| 21 | All thirteen sub-items verified separately below. |
| O1 | Existing post-sleep poll/deadline bounds, humidity failure, standby table, and reset-reapply tests remain valid; strengthen the separate no-restore regression. |
| O2 | Retain/document full-tuple staged applies and their IIR-history consequence; deriving partial writes from an untrusted cache would weaken resynchronization. |
| O3 | Keep the major-version backlog; correct its caller claims without removing compatible APIs. |
| O4 | Same disposition as 20; source-text coupling is a real maintenance constraint, not a reason for a speculative split in this pass. |

| Finding 21 sub-item | Verified decision |
|---|---|
| Tombstone assertions | Remain removed. |
| Tautological reconnect check | Actual argparse default/override tests remain. |
| `getSettings()` return type | Keep `Status` in 2.x; document all caller categories for the later `void` migration. |
| Selftest double-count | Both CLIs restore only after a successful baseline capture. |
| Raw mode bits `2` | Keep canonical typed input and decode both hardware FORCED encodings. |
| CRLF prompt suppression | Keep intentional one-prompt behavior. |
| Health state color | Common/Arduino behavior remains correct; fix the IDF `drv` output to distinguish UNINIT and OFFLINE correctly. |
| IDF stress/input parity | Progress exists on success and failure paths; both command buffers accept 127 characters. |
| Short-read transport OK | Keep both documented count-reporting paths; core converts insufficient counts to `I2C_SHORT_TRANSFER`. |
| PlatformIO noise | Redundant inheritance remains removed; native flags remain separate from embedded flags. |
| README include path | Matches the example helper include path. |
| Packaged datasheet | Retain as primary source evidence. |
| Tracked transcripts | Three explicit allowlisted files total 10,384,702 bytes; retain evidence and defer storage migration absent a concrete need. |

### Validation evidence for this pass

The canonical command list remains the [README validation gate](../README.md#validation).

| Check | Result from this pass |
|---|---|
| Native Unity through `scripts/pio.cmd` | 199/199 passed; 198 at the starting commit. |
| Python regression suites | 87 parser, six package, and 25 checker tests passed (118 total); includes all 45 IDF handler-removal subcases. |
| Timing, Arduino CLI, IDF, HIL, version and release checks | Passed; version remains 2.1.0. |
| Python compilation | `python -m compileall -q tools scripts` passed. |
| Strict core C++17 syntax | Passed with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Werror`. |
| Arduino ESP32-S3 and ESP32-S2 | Both final-source builds passed through the repository wrapper and pinned platform; final retry log is `.pio/audit_embedded_retry.log`. |
| Doxygen | 1.13.2 completed without warnings. |
| Package and final metadata | `pio.cmd pkg pack` and the archive checker passed; tracked `idf_component.yml` remained unchanged. Generated archive removed after checking. |
| HIL dry runs | Default and job-API plans completed with INCOMPLETE, correctly making no hardware claim. |
| Ignored-SLEEP mutation | Disabling mode verification fails six independent one-operation tests; unmodified 20-test focused controls pass. |
| Existing core-fix mutations | Removing post-sleep bounds, swallowing the humidity-write error, shortening standby maxima, skipping reset reapply, rejecting enabled sentinels, and enabling a skipped channel each fail the relevant regression. |
| No-restore evidence | An erroneous restoration write passes the old regression and fails the strengthened one. |
| Checker mutations | Restoring comment-first stripping fails four directory/scenario subcases; disabling IDF dispatch validation fails 45 handler-removal subcases. Restored controls pass. |
| IDF color check | Extracted old/new output expressions compiled with strict C++17 flags: six fixtures produce three baseline mismatches and zero after the fix. |
| CMake checkout-name check | Actual example CMake evaluated with stubbed IDF registration under three checkout names, including spaces; all dependencies derive correctly. This is not an IDF build. |

Local build setup required only process-local environment changes. An initial
attempt using `%USERPROFILE%/.platformio` stopped during dependency extraction
at a long Windows path. The existing default `C:\pio` worked for the baseline
builds. During final rebuilding, shared tool setup changed the compiler location
and reported file contention; selecting the then-existing
`C:\pio\tools\toolchain-xtensa-esp-elf\bin` on process `PATH` allowed both final
builds to finish. Every invocation used `scripts/pio.cmd`; no additional Core
or persistent user configuration was installed or changed.

Native ASan/UBSan and full ESP-IDF builds were not run locally (`idf.py` is
absent); those remain Linux CI checks. No sensor was flashed or physically
tested. There is no new electrical fault-injection, runtime ESP-IDF, or accuracy
result in this report. Synthetic tests establish behavior at the transport
callback boundary. Disposable logs, mutation copies, and dry-run artifacts are
ignored under `.pio/`; committed regressions carry the reproducible acceptance
checks.

## 2026-09-05 verification report

Fetched all remotes and fast-forwarded `main` to its upstream; the clean checkout
was already at the newest remote commit, `5837d0b`. Reviewed every A–G and 1–21
finding, all thirteen minor sub-items, the O1/O2 follow-ups, and the withdrawn
and deferred proposals. Cross-checked the current source, original audit at
`558a31f`, historical source where relevant, the local Bosch PDF, and regression
tests. The summary and verification ledger below record each disposition.

The implemented core fixes remain appropriate. No change to `src/BME280.cpp`,
compensation math, public signatures, or release version was needed. Two small
remaining defects were reproduced and fixed:

- **G: bus initialization could still report false success.** The CLI already
  printed help after a reported initialization failure, but `initWire()` threw
  away the boolean results of `Wire.begin()` and `Wire.setClock()`. It now returns
  `false` when either fails. The Wire stub models both results, and a regression
  checks both failures and a subsequent successful retry. This extends the
  existing helper and caller error path without adding a new abstraction.
- **B: prefixed archive names could crash metadata validation.** Path comparison
  correctly normalized `./library.json`, but the checker then used the normalized
  name to look up the original tar member, raising `KeyError`. The map now keeps
  the original member name for extraction and the normalized name for comparison.
  Flat, `./`-prefixed, and directory-prefixed archives were exercised, along with
  missing-header and forbidden-path cases. The normal PlatformIO archive was
  unaffected by the defect.

Other completed changes:

- **9:** removed nine ignored message arguments from the Arduino/IDF examples,
  preserving error codes and details. The compatibility overload and its native
  test remain; their eventual removal is recorded in `MIGRATION_3X.md`.
- **12:** made the README and `DriverState::READY` enum comment agree with the
  existing `isOnline()` contract: health does not guarantee measurement readiness.
- **13.1:** repaired the NVM timeout test's initial clock. The added quiesce read
  had consumed its wrap before NVM polling began. The test now crosses wrap during
  the NVM read itself. The implementation was already correct.
- Corrected this report's false standby-symbol correction, unsupported sentinel
  frequency estimates, unconditional SLEEP-on-failure claim, remaining message
  count, callback-validator reach claim, and overly broad compensation wording.
  Repaired the stale O1/O2 links and made all minor-item decisions explicit.

**Simplest-solution decisions:** keep the common synchronous write helper and the
separate staged state machine; both have actual callers and different callback
budgets. Keep quiesce-then-NVM-gate, configuration readback, exact standby bounds,
and configuration-derived channel validity. Keep whole-tuple staged applies
explicit rather than deriving partial writes from an untrusted cache. Preserve
2.x compatibility instead of deleting aliases, snapshot fields, or changing
`getSettings()`'s return type. The large HIL module split remains a separate
maintenance task: it does not fix a demonstrated behavior defect and would
require coordinated source-contract checker changes.

### Validation evidence

Used the canonical [README validation gate](../README.md#validation), plus
targeted negative cases. Results below are from this pass; hardware results from
older campaigns are not results of this review.

| Check | Result |
|---|---|
| Native Unity suite | 198/198 passed (197 at the starting commit) |
| HIL parser/classifier suite | 87/87 passed |
| Core timing, Arduino CLI, HIL, IDF example contracts | Passed |
| Generated version and release metadata | Passed; version remains 2.1.0 |
| Python compilation | Passed |
| Core C++17 syntax with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Werror` | Passed |
| Arduino ESP32-S3 and ESP32-S2 builds | Passed using the repository wrapper and pinned platform |
| Doxygen | Passed, version 1.13.2 |
| Package and content checker | Passed; synthetic flat, `./`-prefixed and directory-prefixed layouts accepted; 12 forbidden-path and seven missing-header cases rejected manually in this pass (all now committed in `tools/test_check_package_contents.py`, expanded on 2026-09-07) |
| HIL dry runs, default and with job API | Completed with `INCOMPLETE`, as required for runs without hardware |
| Targeted mutations | Ignored begin/clock failures and unsigned NVM deadline comparison each fail the intended regression; controls pass |
| O1 mutations repeated | Disabled post-sleep bounds, swallowed humidity-write error, shortened standby table and skipped reset reapply each fail the named test(s); six-test control passes |
| Checker negative cases | Forbidden timing calls/includes and removed CLI dispatch comparisons rejected |
| Checkout-name CMake evaluation | `BME280`, `renamed-driver`, and `renamed driver` derive the correct component dependency using host CMake with stubbed IDF registration |

The initial embedded build attempts stopped before compilation because the
existing `PLATFORMIO_CORE_DIR` selected `C:\pio`, whose Python environment was
incomplete (`uv` setup exited 106). For the successful builds, a process-local
override selected the already installed `%USERPROFILE%\.platformio`; all
PlatformIO invocations still used `scripts\pio.cmd`. No other Core was installed
and no persistent user configuration was changed.

Native ASan/UBSan and full native ESP-IDF compilation are Linux CI checks; they
were not run locally (`idf.py` is absent). Host CMake evaluation is not an
ESP-IDF build. No board was flashed and no physical HIL, electrical fault
injection, runtime ESP-IDF, or accuracy validation was performed. Temporary
mutation builds and dry-run output remain ignored under `.pio/`; the generated
package is removed after validation.

## 2026-09-07 checker follow-up

The three cleanup findings against `ee28733` were valid. Tracked files were
clean and release metadata passed; the sole untracked file was
`idf_component.yml.orig`, identical to the tracked manifest after line-ending
normalization. Removed that backup and ignored `*.orig` alongside temporary
files. No driver source, public header, version, or HIL tooling changes.

**Reproduced manifest rewrite mechanism:** the installed, pinned pioarduino
`platform-espressif32` 55.03.311 calls
`ComponentManager.handle_component_settings()` from `builder/frameworks/arduino.py`.
In `builder/frameworks/component_manager.py`,
`ComponentHandler._get_or_create_component_yml()` falls back from the Arduino
framework manifest to the project source manifest. This repository sets
`src_dir = .`, so that fallback selects the root `idf_component.yml`.
`_create_backup()` copies it to `.orig`; `_load_component_yml()` uses
`yaml.load(..., Loader=SafeLoader)` and `_save_component_yml()` uses
`yaml.dump(data, f)`, even when neither adding nor removing dependencies.
This sorts keys, removes quotes, and changes list indentation. The Arduino
post-action restores `pioarduino-build.py`, not the project manifest; the
manifest restoration in `espidf.py` is conditional on ESP-IDF/custom-component
paths. Thus a completed Arduino build can leave both files behind. The package
test only writes metadata into temporary archives and is not this producer.
Calling the installed `ComponentHandler.handle_component_settings()` on a
temporary copy reproduced both a byte-identical backup and the reported YAML
rewrite, with dependency additions/removals disabled.

This was a direct component-manager reproduction, not a reproduction through
a complete Arduino build. Both successful Arduino builds in this pass left the
repository manifest unchanged. The mechanism matches the discovered backup
and formatting, but the historical invocation that created them was not captured.
An independent follow-up build stopped earlier in its local tool setup, so it
neither confirmed nor disproved the mechanism.

The README gate now rechecks release metadata and the tracked manifest after
building. Inspect any rewrite before restoring it and repacking; ignoring the
backup alone does not repair a rewritten manifest.

**Committed coverage:** the package suite now contains twelve forbidden-path
cases and seven missing-common-header cases. Its archive fixture uses an
independent expected-path set. `tools/test_checker_contracts.py`, run by
`validate-library` and the README gate, independently pins the seven forbidden
timing calls, 43 CLI commands, seven common headers, and 28 base package paths.
It also runs real checker entry points against synthetic forbidden calls and
framework includes in both core directories, missing CLI handlers, and each
missing package path. Positive controls verify accepted source, CLI, and
archive fixtures. All filesystem mutations stay in temporary directories.

**Mutation acceptance:** independently deleted `vTaskDelay` from
`FORBIDDEN_CALLS`, `help` from `COMMANDS`, and each of the seven common headers
from `BASE_REQUIRED_PATHS` in temporary checker copies. All nine runs exited 1
and failed the corresponding rule-preservation test. The unmodified and
restored controls each passed all nine checker self-tests. The package suite
also passed all six tests, including the expanded subcases.

Local validation passed: 198 native tests, 87 existing parser tests, all contract
and metadata checks, the README dry runs, strict C++17 core syntax, and both
Arduino targets. Doxygen 1.15.0 completed without warnings. The first Arduino
attempt could not find the existing compiler; successful builds used a
process-local `PATH` addition for
`%USERPROFILE%/.platformio/packages/toolchain-xtensa-esp-elf/xtensa-esp-elf/bin`
through `scripts/pio.cmd`. The installed esptool setup also emitted editable-
install warnings, but both builds subsequently generated their firmware images
with esptool 5.3.0. No Core was installed or persistent tool configuration
changed. Native sanitizers and full ESP-IDF builds remain CI checks because
`idf.py` is absent locally; no physical HIL was performed.

### Remaining checker self-tests

The follow-up correctly identified that the IDF example, release metadata, and
HIL contract checkers had no self-tests. Extended the existing
`tools/test_checker_contracts.py` CI entry point with positive controls and
synthetic violations for all three. IDF tests independently pin required native
tokens, forbidden Arduino tokens/patterns, and command names. Release tests
reject mismatched header fields, component/Doxygen versions, malformed SemVer,
and missing changelog metadata, including the unquoted component-version case.
HIL checker tests reject changed documented sequences, missing documentation
requirements, unsupported hardware claims, and missing final recovery.

These are representative contract regressions, not exhaustive tests of every
branch in the larger checkers. Text substitutions are confined to the checker
read boundary and release fixtures to temporary files. No checker implementation,
runner, driver source, public header, or release version changed; no physical
HIL is involved.

All 21 checker tests passed. On temporary checker copies, deleting the IDF
`Arduino.h` rule, disabling component-version validation, and deleting the HIL
`--require-pass` documentation requirement each made the new tests exit 1.
Unmodified and restored controls both passed all 21 tests.

The follow-up local gate passed 198 native tests, 87 parser tests, six package
tests, all contract/metadata checks, Doxygen, dry runs, and archive validation.
The Arduino rerun remained in tool setup while an older PlatformIO build was
still active; only the follow-up process was stopped. This follow-up does not
claim fresh local Arduino build results; embedded builds and sanitizers are
verified separately in CI.

**Component-name guard follow-up:** the three IDF CMake derivation literals now
have a module-level rule tuple and an independent test expectation. Synthetic
CMake tests reject each missing token and both `REQUIRES BME280` and
`PRIV_REQUIRES BME280`, retaining the positive token in a comment to isolate the
negative guard. This exposed literal backspace bytes in that guard's regex;
replacing them with `\b` word boundaries restored the intended rejection.
All 23 checker tests pass. Deleting the negative guard, the positive token loop,
or both produces three parseable checkers that still print `PASSED` against the
real repository, while each mutation fails the new regression suite.

## Status legend

| Mark | Meaning |
|---|---|
| **DONE** | Implemented and verified. An independent reviewer only needs to confirm the implementation matches the description and that the named test still pins it. No design decision is open. |
| **OPEN** | Still actionable. |
| **DEFERRED** | Valid, deliberately postponed, with the reason recorded. |
| **WITHDRAWN** | The finding was wrong. Kept so it is not "rediscovered". |

## Summary

| # | Finding | Status |
|---|---|---|
| A | `signExtend12()` implementation-defined narrowing | **DONE** |
| B | Package checker could not detect a `.pio`/`.git` leak | **DONE** |
| C | Package checker did not require `examples/common/*.h` | **DONE** |
| D | Core timing guard was mostly vestigial | **DONE** |
| E | CLI contract checker duplication and tombstones | **DONE** |
| F | Reconnect closed the port with a zero retry budget | **DONE** |
| G | Arduino CLI did not come up after a failed bring-up | **DONE** |
| 1 | Skipped-channel sentinels reject valid readings | **DONE** |
| 2 | Multi-byte `writeRegisters()` is an invalid transaction | **DONE** |
| 3 | `im_update` polled outside reset | **DONE** (fixed differently — original proposal was unsafe) |
| 4 | `_applyConfig()` fails `begin()` on a measuring device | **DONE** |
| 5 | `ctrl_hum` write can be silently dropped | **DONE** (trigger condition was misstated) |
| 6 | Config-write sequence implemented four times | **DONE** (staged path is always FULL, by design and now documented) |
| 7 | Cached configuration never verified against the device | **DONE** |
| 8 | Freshness budget ignores standby tolerance | **DONE** |
| 9 | Diagnostic messages constructed and thrown away | **DONE** (count was wrong; overload kept for 2.x) |
| 10 | Compatibility aliases | **DEFERRED** to 3.x — and 3 of 7 are load-bearing |
| 11 | `SettingsSnapshot` duplicates sample data | **DEFERRED** to 3.x |
| 12 | `_updateHealth()` reports `READY` while unusable | **DONE** as docs; `canMeasure()` proposal **WITHDRAWN** |
| 13.1 | `_waitForNvmReady()` TIMEOUT unreachable | **WITHDRAWN** — false |
| 13.2 | Validation detail lost | **DONE** |
| 13.3 | IIR history after a skipped channel undocumented | **DONE** |
| 13.4 | Register-reference wording (`config`, `0xE8..0xF1`) | **DONE** |
| 14 | Classification ran on truncated output | **DONE** (all measurements were wrong) |
| 15 | `job_command_budget()` misattributes budgets | **DONE** |
| 16 | `output_has_expected()` reports completion, not expectation | **DONE** (title was overstated) |
| 17 | `final_verdict()` ignores `RESULT_SKIPPED_UNSAFE` | **WITHDRAWN** — unreachable |
| 18 | ESP-IDF component name depends on directory | **DONE** (the proposed snippet had a bug) |
| 19 | Native stubs do not model the hardware | **DONE** |
| 20 | `run_i2c_hil.py` is six modules in one file | **DEFERRED** (dead code removed) |
| 21 | 13 minor example/tooling items | 8 **DONE**, 1 **DEFERRED**, 2 **WITHDRAWN**, 1 partial, 1 no-action |

---

# Open work

The test-coverage gaps (O1) and the staged-apply contract (O2) are closed.
What remains is deliberately deferred, not pending.

## O1. Test-coverage gaps — CLOSED

All four gaps are closed. Each new test was mutation-verified: the fix it
covers was reverted and the suite confirmed to fail.

| Gap | Test added | Mutant that now dies |
|---|---|---|
| `APPLY_WAIT_AFTER_SLEEP` bounds | `test_staged_apply_wait_after_sleep_times_out_on_poll_cap`, `..._on_deadline` | replacing the guard with `if (false)` fails both |
| Synchronous `ctrl_hum` write failure | `test_humidity_ctrl_hum_write_failure_marks_dirty_and_preserves_error` | swallowing the error fails it (write count 17 vs 16) |
| Standby tolerance table | `test_normal_freshness_budget_covers_every_standby_tolerance` | zeroing six of the eight values fails it |
| `FakeBus` soft reset | `test_soft_reset_reapplies_control_registers_the_device_cleared`, `test_staged_soft_reset_reapplies_control_registers_and_verifies` | skipping the config re-apply in `softReset()` fails it (0x00 vs 0x04) |

Two supporting changes:

- `FakeBus` now models the power-on-reset: a `0xB6` write to `0xE0` clears
  `ctrl_hum`, `ctrl_meas` and `config` to `0x00`, and optionally raises
  `im_update` for a configurable number of status reads
  (`softResetImUpdateReads`). All pre-existing tests passed unchanged against
  the more honest device model, which is itself evidence the reset paths were
  already correct.
- `test_humidity_ctrl_hum_failure_marks_dirty_and_preserves_error` was renamed
  to `test_humidity_quiesce_write_failure_marks_dirty_and_preserves_error`,
  because it injects at write #1 — the quiesce `ctrl_meas`=SLEEP write — not at
  the `ctrl_hum` write its old name claimed. It is still a useful test; it was
  just misnamed.

Suite: 191 -> 197 tests.

## O2. Staged apply always writes `config` — documented, by design

`SettingsWritePlan` (`include/BME280/BME280.h`) is consumed only by
`_writeSettingsAfterQuiesceSynchronously()`. The staged `pollJob()` path has no
plan parameter, so `APPLY_CONFIG` writes `0xF5` unconditionally and the staged
path is permanently equivalent to `SettingsWritePlan::FULL`. Consequence:
`startApplySettingsJob()` with a mode-only change rewrites `config` and so
resets IIR filter history, while `setMode()` does not.

**Resolved as documentation rather than code.** On reflection the asymmetry is
correct, not a defect: `startApplySettingsJob(settings)` means "make the device
match this whole tuple", and a full write is the honest implementation of that,
whereas the single-field synchronous setters are selective precisely because
they change one field. Making the staged path diff against the cache would also
be wrong whenever the cached image is already untrusted. What was missing was
the contract, which is now stated on both staged entry points and in the README
timing/configuration notes.

If a future caller genuinely needs a staged partial apply, add a plan parameter
to `startApplySettingsJob()` rather than inferring one from a diff.

## O3. Deferred to 3.x

Tracked durably in [`MIGRATION_3X.md`](MIGRATION_3X.md), which survives this
document.

Breaking changes, deliberately not made in a 2.x correctness pass. **Three of
the seven aliases in finding 10 are load-bearing** and cannot be removed with a
header edit alone:

| Symbol | Blocker |
|---|---|
| `Err::CONVERSION_NOT_READY` (`Status.h:21`) | none — definition only |
| `JobKind::RECOVERY` (`BME280.h:48`) | none — definition only |
| `driverState()` (`BME280.h:672`) | none — definition only |
| `cmd::REG_DIG_H5_LSB` (`CommandTable.h:89`) | none — definition only, and **misnamed**: `0xE6` holds `dig_H5[11:4]`, the high bits (datasheet Table 16) |
| `startRecoveryJob()` (`BME280.h`) | both shipped examples, native tests, and required tokens in `check_cli_contract.py` and `check_idf_example_contract.py` |
| `JobPollResult::instructionsUsed` (`BME280.h:237`) | both shipped examples print it; the `"Instructions:"` token is required by `check_idf_example_contract.py:206` |
| `VERSION_INT` (`Version.h:67`) | emitted by `scripts/generate_version.py:235`, asserted verbatim by `check_release_metadata.py:41` |

Also for 3.x: the six duplicated `SettingsSnapshot` fields (finding 11) and
`Status getSettings()` -> `void` (finding 21).

## O4. HIL runner module split (finding 20)

`tools/run_i2c_hil.py` is 4717 lines, down from 4884 after the dead
`duration_command_fits()` and the 172-line embedded `parser_self_test()` were
removed. The structural split remains deferred: no current behavior defect
requires it, and the extraction would need coordinated checker changes.

Note the coupling that will fight any refactor: `check_hil_contract.py`
asserts 23 required literal substrings against the runner's *source text*, and
the CLI/IDF contract checkers do the same to the two `main.cpp` files. Those
checkers must be rewritten alongside.

---

# Implemented — verification ledger

For each item: what was actually wrong, what shipped, and what pins it.

## A. `signExtend12()` narrowing — DONE
`value |= 0xF000` on an `int16_t` computed in `int` and assigned back. Correct
on every two's-complement target but formally implementation-defined pre-C++20,
and the only `-Wconversion` warning in the core. Now stays inside each type's
value range (`src/BME280.cpp:308`), with the `dig_H4`/`dig_H5` register packing
stated in a comment. Core compiles clean under `-Wall -Wextra -Wpedantic
-Wshadow -Wconversion -Wsign-conversion`.

## B. Package checker forbidden-path scan — DONE
`normalize()` used `name.lstrip("./")`, which strips a character *set*, not a
prefix, so `.pio/build/x` became `pio/build/x` and `FORBIDDEN_PARTS` could never
match a root-level dot-entry. Latent rather than active, though not for the reason
first given: `pio pkg pack` emits flat, unprefixed members, and
`library.json`'s `export.include` allowlist admits no root-level dot-entry, so
no packaged name has ever begun with `.` or `/` and `lstrip` was a no-op on all
32 of them. The guard was one `export.include` change away from silently
passing. Now `removeprefix("./")` (`tools/check_package_contents.py`). The
2026-09-05 follow-up also preserves each original tar member name for extraction;
normalization is only for comparison. This prevents `KeyError` when an otherwise
valid archive uses `./` prefixes.

## C. Package checker required-file list — DONE
The Arduino example was required but none of the seven `examples/common/*.h`
headers it needs were (six included directly, `BuildConfig.h` transitively via
`Log.h`), so dropping that directory would have shipped an example that cannot
compile while CI printed `Package contents PASSED`. All seven added;
`pio pkg pack` plus the checker confirms they ship.

## D. Core timing guard — DONE
`ALLOWED_CALL_COUNTS` and `ALLOWED_INCLUDE_COUNTS` were empty dicts, so four
loops iterated over nothing and the effective rule was "zero forbidden calls or
includes". Rewritten to state that rule directly (122 -> 83 lines) and extended
to `delay`, `vTaskDelay` and `esp_timer_get_time`.

## E. CLI contract checker — DONE
`HANDLED_COMMANDS` duplicated `MANDATORY_COMMANDS` minus `"help"`; the
`\b<cmd>\b` word-search loop matched help text and comments and so could not
detect a removed command; the `cfg`/`settings` check was unreachable; and two
tombstones asserted the absence of `examples/00_smoke_boot` and
`examples/03_feature_walkthrough` — paths that appear nowhere in this
repository's history, so those assertions could never have failed. Collapsed to
one `COMMANDS` list checked with the real handler-pattern regex, which now also
covers `help` (359 -> 297 lines).

## F. Reconnect with a zero retry budget — DONE
`--reconnect-attempts` defaults to 0, and `reconnect_serial_in_place()` called
`ser.close()` before a retry loop that then never ran, returning "not recovered"
with the port closed. `run_live_plan()` — docstring "always perform required
final cleanup" — then ran `run_final_cleanup()` on a dead handle. All five rows
(`normal off`, `recover`, `cfg`, `status`, `drv`) fail at the first
`ser.write()` with `PortNotOpenError`, which `run_serial_command()` catches as
an `OSError` and records as `SERIAL_WRITE_EXCEPTION` / `FAIL` — not
`FINAL_CLEANUP_EXCEPTION`, which only that function's own `except` produces. The
trigger needs a duration soak (`--soak-duration-s` defaults to 0), after which
the default `--reconnect-attempts 0` leaves the sensor in whatever state the
soak abandoned and the mandatory safe-state evidence unobtainable. Now returns
immediately with the handle untouched (`tools/run_i2c_hil.py:3461`).

## G. Arduino CLI after a failed bring-up — DONE
`setup()` returned early on I2C-init or `begin()` failure, so `printHelp()` and
`printPrompt()` never ran — with `scan`, `addr` and `begin` being exactly the
recovery commands needed in that state. The ESP-IDF `app_main()` already did it
correctly, so this was also a parity break. Success-path output is unchanged.
The 2026-09-05 review additionally found that the Arduino helper always returned
`true` even if `Wire.begin()` or `Wire.setClock()` failed. Both results are now
checked and the native stub/test exercise both failures and retry. The Arduino
`begin` command retries sensor initialization; it does not reinitialize the
application-owned bus. The helper can be called again by the application, or
startup rerun after correcting the bus setup; this change does not add an
automatic bus recovery policy.

## 1. Skipped-channel sentinels — DONE
`_readRawData()` marked a channel invalid when its raw ADC equalled the Bosch
skipped value, and `_compensate()` then failed the whole sample. `0x80000` and
`0x8000` are the *reset and skipped* register values (Tables 18/20/23/24); the
datasheet nowhere states they cannot occur as genuine output.

For the test fixture's `dig_T1 = 27504`, `dig_T2 = 26435`, `dig_T3 = -1000`,
`adc_T = 0x80000` gives `t_fine = 135479`, T = 26.46 C. The enabled-sentinel
tests demonstrate successful temperature, pressure and humidity compensation
and sample publication. Pressure and humidity values depend on their own
calibration coefficients and on `t_fine`; the sentinel alone does not identify
a pressure, altitude, or humidity.

Earlier versions estimated rejection rates of ~40%, ~35%, ~3%, and ~2.5% from
typical noise and quantization. Those are not measured device failure rates.
They assume a distribution, its alignment with a quantization bin, particular
calibration and environmental conditions, and filter behavior. The datasheet
does not establish those assumptions or a 20–30 C sentinel temperature across
all parts. The correctness finding needs none of those estimates. Bosch
§3.4.2–3.4.4 do establish the oversampling-dependent unfiltered resolution and
20-bit filtered T/P output; humidity output is 16-bit.

**Shipped:** validity is derived from configuration alone
(`src/BME280.cpp:2941-2946`); zero references to `0x80000`, `0x8000` or
`RAW_*_SKIPPED` remain in `src/`. `_compensate()`'s guard survives as an
assertion-style early return. `cmd::RAW_*_SKIPPED` retained as documentation.
**Pinned by:** `test_enabled_raw_sentinel_is_a_valid_adc_sample` (sets all three
channels to the sentinel with all three enabled),
`test_enabled_raw_sentinel_commits_a_new_sample_envelope`, and
`test_skipped_sentinels_are_explicit_validity_flags` for the converse.
Restoring the comparison kills 3 tests; forcing all channels valid kills the
third. Both directions are pinned.

## 2. Multi-byte register writes — DONE
The old encoding assumed the register pointer auto-increments on writes. It does
not: §6 says "multiple byte write (using pairs of register addresses and
register data)", Figure 9 is captioned "I2C multiple byte write (not
auto-incremented)", and §6.2.1 says the master sends *pairs*. Reads do
auto-increment. So `writeRegisters(0xF2, buf, 4)` sent `F2 b0 b1 b2 b3` and the
device treated `b1` as the next register address.

**Shipped:** address/value pairs into `uint8_t payload[2U * MAX_WRITE_LEN]`
(`src/BME280.cpp:2412-2432`), plus an address-wrap guard that was not in the
proposal. Single-register writes are byte-identical to before.
**Pinned by:** `test_multi_register_writes_use_exact_pairs_and_reject_malformed_ranges`,
and `FakeBus` now rejects odd-length writes with `TransportErr::OTHER, -5`, so
reverting the encoding fails immediately rather than silently passing.

## 3. `im_update` polled outside reset — DONE, fixed differently
§5.4.4 / Table 21: `im_update` is set "at power-on-reset **and before every
conversion**". A device in normal mode pulses it continuously.

The original finding said `begin()` was sound because the device is in sleep.
**That was wrong** — `begin()` does not reset and did not quiesce, so it carried
the same hazard, as did the staged `INIT` path. The original text contradicted
finding 4 on the next page.

The original **proposal — skip the gate for non-reset resync — was unsafe**, and
was correctly rejected. The registers being written during the copy *are* the
calibration image registers, which is what the bit exists to protect;
`_validateCalibrationValues()` checks only `digT1`/`digP1` plausibility and
cannot detect a torn-but-plausible coefficient, and resync then commits it. That
trades a loud, retryable `BUSY` for a silent, permanent calibration corruption.

**Shipped (better):** quiesce to sleep *then* gate, on all four paths —
`begin()` `:624`, `recover()` `:793`, staged INIT `:1395`, staged RESYNC
`:1817`. In sleep the only `im_update` source is a reset copy, so the false
failures go away and the coherence guarantee is kept. It is free, because both
paths re-apply configuration afterwards and §5.4.6/§3.3.1 require sleep for that
anyway. **Pinned by:** `test_init_and_resync_quiesce_normal_mode_before_calibration_reads`;
removing the quiesce kills 6 tests (sync) and 5 (staged). The 2026-09-08 pass
closed a remaining gap: the post-write status check now also reads and verifies
SLEEP mode, since an ignored sleep request followed by normal standby can have
`measuring == 0`. See the latest report above for the reproduction and fix.

## 4. `_applyConfig()` failed `begin()` on a measuring device — DONE
The first `_ensureConfigWriteReady()` hard-failed `begin()` with `BUSY` on a
device left in normal mode. Writing `ctrl_meas` to sleep is legal at any time —
§3.3.1 says the transition is deferred, not rejected. The second check is
genuinely required (§5.4.6 for `config`, §3.3.1 for `ctrl_hum`), and the staged
INIT path already got this right.

Deleting the first check does not itself guarantee immediate success: the second
check still returns `BUSY` when `measuring` remains set. The real
benefit is convergence: the sleep write now reaches the device before the driver
gives up, §3.3.1 guarantees it executes at the end of the running measurement,
and the caller's retry then finds an idle device instead of re-rolling the same
dice forever. The previously quoted ~7% is only 9.30 / (125 + 9.30), using
the maximum x1/x1/x1 conversion time and nominal standby. It is not an observed
failure probability and additionally assumes request timing uniformly spans the
cycle; the retry-convergence argument is independent of that estimate.

**Shipped:** `_quiesceSettingsSynchronously()` (`src/BME280.cpp:2671`) writes
sleep first with no pre-check, then the retained second check.
**Pinned by:** `test_begin_queues_sleep_then_reports_busy_when_measuring_persists`,
which asserts dirty-then-clean across the retry. Re-inserting the old pre-check
kills 9 tests.

## 5. `ctrl_hum` write can be silently dropped — DONE, trigger was misstated
§3.3.1: "Further mode change commands **or other write commands to the register
`ctrl_hum` are ignored** until the mode change command has been executed."

The original text said the drop follows from the device merely measuring. **It
does not.** §3.3.1 requires an outstanding *delayed mode-change command*.
Verified by execution: at `3b3fad1`, `setOversamplingH()` wrote `ctrl_hum` as
write **#1**, before its own `ctrl_meas`, so it could not trigger its own drop.

The defect was real, but via a *preceding* caller: `setMode()`,
`setOversamplingT()` and `setOversamplingP()` all wrote `ctrl_meas` in place
with no sleep and no idle check, so a `setMode(NORMAL)` during a running forced
conversion left a queued mode change, and a `setOversamplingH()` inside the same
measurement period lost its `ctrl_hum` write, cached the new `osrsH`, and
returned `Status::Ok()` with nothing marked dirty. Finding 7 already stated this
condition correctly; the two contradicted each other.

**Shipped:** all setters route through `_writeSettingsSynchronously()`, so
`ctrl_hum` is only ever written with the device confirmed idle and in sleep,
with no mode change outstanding — the hazard is structurally excluded.
**Pinned by:** `test_humidity_oversampling_writes_ctrl_hum_then_ctrl_meas`
(asserts the exact 3-write log).

## 6. Four copies of the config-write sequence — DONE, see [O2](#o2-staged-apply-always-writes-config--documented-by-design)
The sequence existed in four places. Two corrections to the original text: the
table claimed `pollJob()` restores mode on failure "via `_failJob()`" — **it
issues no register write at all**, restoring only cached state. The hardware
state depends on the phase and which writes reached the device; a failure after
the final NORMAL-mode write can leave it in NORMAL, and an ambiguous sleep write
does not prove SLEEP. Dirty-state diagnostics preserve that uncertainty. And
`setFilter()`/`setStandby()` were not "byte-for-byte identical
apart from one argument": they are 57-line functions differing in 5 lines
(signature, validator, validator message, `buildConfig()` argument order, cache
commit), with a 39-line identical write block.

**Shipped:** two implementations remain — synchronous
(`_quiesceSettingsSynchronously` + `_writeSettingsAfterQuiesceSynchronously`)
and staged (`pollJob`'s `APPLY_*` phases). Every synchronous caller routes
through the first. Net reduction ~130 lines, not the ~200 estimated. Failure
behaviour was unified downward: no path restores the mode, matching the staged
path.

## 7. Configuration never verified against the device — DONE
Everything the driver believed rested on "the write returned OK". The datasheet
supplies two ways that can be false with no transport error: §5.4.6 (normal-mode
`config` writes "may be ignored") and §3.3.1 (`ctrl_hum` dropped while a mode
change is queued).

**Shipped:** `_verifySettingsReadback()` (`src/BME280.cpp:2611`) does one
`readRegs(0xF2, values, 4)`; `status` (`values[1]`) is read but never compared.
Masks are `ctrl_hum 0x07`, `ctrl_meas 0xFF`, `config 0xFD`. No mode masking is
needed — `registerModeForConfig()` maps `FORCED -> SLEEP`, so the expected image
already carries `mode = 00` (the original proposal's parenthetical prescribing a
mask was wrong). Mismatch returns `RESYNC_REQUIRED` with the register, expected
and actual packed into `Status::detail`, marks dirty, and does not commit the
cache. Staged equivalent is `JobPhase::APPLY_VERIFY`.
**Pinned by:** `test_synchronous_settings_readback_mismatch_preserves_cache_and_details`
and `test_staged_apply_verification_is_one_callback_and_keeps_desired_dirty` —
both make the fake report a byte *different* from what was written, so the
detection is genuinely exercised, plus
`test_settings_readback_ignores_status_and_reserved_bits`.

## 8. Standby tolerance — DONE
Table 1 gives standby time accuracy as typ +/-5 %, max +/-25 %. It explicitly
uses the symbol `Δt_standby`; §9.3 also distinguishes nominal `t_standby` from
its accuracy parameter `Δt_standby`. The previous re-verification's claim that
the delta symbol appeared nowhere was false. The worst case is
`remainder of running conversion + t_standby + full next conversion`, so the old
budget was short by up to 0.25 x standby — 250 ms at `MS_1000`, and 16 ms at
`MS_62_5` (nominal 63 ms against a 78.1 ms worst case), which the original text
did not mention. Every other term was already the Bosch maximum, so standby was
the only untoleranced quantity.

**Shipped:** `maximumStandbyTimeMs()` (`src/BME280.cpp:184`), an exact
eight-entry table rather than the proposed arithmetic — which is strictly
tighter, since `nominal + (nominal + 3) / 4` would have compounded the
pre-rounded nominal and given 2 ms for `MS_0_5`. Public accessors still report
nominal. **Pinned by:** `test_normal_mode_request_waits_for_fresh_cycle` and
`test_high_osr_normal_freshness_deadline_is_wrap_safe`, and the exhaustive
`test_normal_freshness_budget_covers_every_standby_tolerance`; see
[O1](#o1-test-coverage-gaps--closed).

## 9. Discarded diagnostic messages — DONE (count corrected)
`Status(Err, int32_t, const char*)` initialises `msg` from `toString(c)` and
voids `m`, so the text is discarded; the copy constructor and assignment
re-canonicalise too, so it cannot survive indirectly.

**The original count of 102 was wrong. The real figure is 138 call sites
carrying a literal (66 distinct strings), out of 160 `Status::Error(...)` sites.**
102 is what a naive one-line `grep` reports; 36 call sites wrap the literal onto
a continuation line. Independently recounted with balanced-paren extraction.

**The claimed overload ambiguity was also wrong.** `Status::Error(Err::X, 0)` is
not ambiguous: `0 -> int32_t` is an identity conversion (exact match) while
`0 -> const char*` is a pointer conversion, so the integer overload wins.
Verified by compiling it against both the old and current `Status.h` under
`-Wall -Wextra`: no diagnostic, resolves to `detail = 0`. It would become
ambiguous only where `int32_t` is a distinct type from `int` (e.g. AVR, where it
is `long`).

**Shipped:** 138 -> 0 literals in `src/`. The 2026-09-05 pass also removed nine
ignored message arguments from the examples; the previous count of six
remaining example/test sites was wrong. The message-bearing overload remains
for 2.x source compatibility and is deliberately exercised by the native
canonical-message ownership test.

## 12. `isOnline()` overpromises — DONE as documentation
`_updateHealth()` (`src/BME280.cpp`) sets `READY` on any successful tracked
transfer without consulting `_configSyncState` or `_calibrationState`, and
`isOnline()` returns true for `READY || DEGRADED`. The trap is real.

**The proposed `canMeasure()` was withdrawn, correctly.** Its own body checked
only four conditions while its docstring promised "a measurement can be
requested right now" — omitting the timebase (`Config::nowMs` is required), the
mode, staged-job exclusivity, and a pending measurement. A predicate returning
`true` while `requestMeasurement()` returns `BUSY` would be a worse trap than
`isOnline()`.

**Shipped:** the state machine is unchanged, as the finding itself asked, and
`isOnline()`'s documentation now states explicitly that it "does not imply
synchronized configuration, valid calibration, or that the next operation will
succeed" (`include/BME280/BME280.h:674`).

## 13.2 / 13.3 / 13.4 — DONE
- **Validation detail.** `validateSettings()` fused seven predicates into one
  condition and always returned `detail == 0`. Now a public
  `SettingsValidationReason` enum (`Config.h:225`) with the seven values the
  finding named, returned in declaration order, propagated through `begin()`.
- **IIR history.** §3.4.4: a skipped T or P channel keeps its filter memory, so
  the first sample after re-enabling is filtered against pre-skip history.
  Documented on `setOversamplingT/P` and in the register reference.
- **`config` wording.** The register reference said `config` is "writable only
  in sleep mode"; the datasheet says normal-mode writes "may be ignored". Fixed
  — in **both** places that said it, one of which the original finding missed.
- **`0xE8..0xF1`.** Table 18 labels `0xE1..0xF0` as `calib26..calib41` while
  §4.2.2 documents only `0xE1..0xE7`; the conflict spans `0xE8..0xF0` and is now
  noted rather than presented as settled. `0xF1` is unnamed in both sections.

## 14. Classification ran on truncated output — DONE (measurements corrected)
The mechanism was real: `output_excerpt` was `clean_output[-1000:]` and three
reclassifiers read it.

**Every measured number in the original text was wrong**, because they were
taken from the ANSI-coloured raw transcript plus ~42 characters of framing,
while `output_excerpt` is built from `strip_ansi(output)`. Re-measured across
all three retained transcripts: `recover` 459–620, `cfg` 349–355, `status`
85–86, `stress 50` 752, `stress_mix 70` 826–851, `selftest` 1007–1062, `help`
2413–3381. Only `help` and `selftest` exceed 1000, and neither is read by a
reclassifier. The reach claim was wrong too: `results.csv` has no
`output_excerpt` column (`extrasaction="ignore"` drops it) and `manifest.json`
carries no result rows — only `summary.json` holds it. And "one extra
health-diff line" would really take about fourteen. The defect was **latent**,
not active.

**Shipped:** `row_output()` deleted; all three reclassifiers read structured
`parsed_evidence` extracted from the full output. **Pinned by:**
`tools/test_run_i2c_hil_parser.py:307`, which builds rows whose excerpts
deliberately omit the evidence and asserts reclassification still succeeds.

## 15 / 16 / 18 / 19 — DONE
- **15.** `job_command_budget()` returned 1 for `job start init` and
  `job cancel owner`, which consume zero callbacks (confirmed in the driver, the
  CLI, and the retained transcripts: `Callbacks used: 0`). Now verb-aware. The
  inflated start/cancel values were latent because the shipped start and
  deadline-cancel specs use dedicated zero-callback validators, while the
  owner-cancel spec requires the `Callbacks used: 0` output token. The previous
  verification's explanation was false: twelve shipped `CommandSpec`
  construction sites use the generic budget validator, both at the original
  and current baselines, while custom `--commands` specs receive no validators.
- **16.** The original title, "does not check `spec.expected`", was overstated:
  `completion_tokens_match()` falls back to `expected_tokens_match()` whenever
  `spec.completion` is empty, which is 164 of 191 specs. The defect was real for
  the other 27 — a failing `selftest` still prints `Selftest result:`, so the
  row recorded `MATCHED_EXPECTED` while the expected tokens never matched. Now
  `output_is_complete()` / `MATCHED_COMPLETION`, with the idle guard widened to
  `not (spec.expected or spec.expected_any or spec.completion)`.
- **18.** The component name is the checkout directory's basename, so
  `REQUIRES BME280` only resolved when the directory was called `BME280`. **The
  original proposal's snippet contained a bug**: `get_filename_component(...
  NAME)` does not normalise `..`, so applied to
  `"${CMAKE_CURRENT_LIST_DIR}/../../../.."` it yields the literal `..`. That
  snippet was shipped verbatim and broke CI before being corrected. The working
  form resolves to an absolute path first. An intermediate attempt that removed
  `REQUIRES` entirely also broke both ESP-IDF jobs.
- **19.** `FakeBus` auto-incremented writes, and the Wire stub had a 64-byte
  capacity while the example adapter validated against 128. One correction:
  neither threshold was ever exercised — no native test issued a transfer large
  enough to reach 64, so the stub's cap was unreachable rather than an active
  false green. Both now share `I2C_BUFFER_LENGTH = 128`; reverting the stub is a
  compile error. **Pinned by:**
  `test_example_transport_honors_wire_128_byte_capacity_boundary` (128 accepted,
  129 rejected, distinguishing `detail == 129` from `detail == -4`).

## 21. Minor items — all thirteen reviewed

The original bullet order is retained here so every item has an explicit
disposition, including the partial storage proposal omitted from the old ledger.

| Original sub-item | Current verification and decision |
|---|---|
| Tombstone assertions | Removed; no further action |
| Tautological reconnect check | Replaced by real argparse default/override checks |
| `getSettings()` return type | Always returns OK; retain source compatibility and defer `void` return to 3.x |
| Selftest double-count | Both CLIs skip restore after a failed baseline capture |
| Raw mode bits `2` | Keep canonical typed input `0/1/3`; raw display correctly decodes both forced encodings |
| CRLF prompt suppression | Deliberate and commented; no behavioral change needed |
| Health state color | Common health output uses `DriverState`; the remaining IDF `drv` color error was corrected on 2026-09-08 with the same state-based decision |
| IDF stress/input parity | Progress emitted in success/failure paths; both accept 127 characters |
| Short-read transport OK | Correct in both short-read paths; counts let the core report `I2C_SHORT_TRANSFER`, already documented |
| PlatformIO noise | Redundant `extends = env` removed; embedded-only settings moved out of shared defaults |
| README include path | Already aligned with the example |
| Packaged datasheet | Retain primary source evidence deliberately |
| Tracked transcripts | Explicit three-run allowlist is implemented; retain existing evidence, defer external storage/LFS migration absent a current need |

The original color example, READY with one consecutive failure, violated the
health invariant. The state-based color solution is still appropriate, for
example when `offlineThreshold` differs from the old color helper's constant.

Two corrections to the original bullets: the short-read `OK` in
`I2cTransport.h` occurs in **two** places (`:181` short `requestFrom`, `:188`
drained `available()`), not one, and it was **already documented** in
`docs/PRODUCTION_SHARED_BUS_GUIDE.md:152` at the baseline — no action was
needed. And there are **three** tracked transcripts (5.05, 5.23 and 0.10 MB),
not "two ... ~10.4 MB total"; the exact three-file total is 10,384,702 bytes.

---

# Withdrawn findings

Kept so they are not rediscovered.

## 13.1. `_waitForNvmReady()`'s TIMEOUT branch is unreachable — FALSE

The branch is reachable, and acting on this finding would have deleted working,
contract-documented, test-covered behaviour.

The original reasoning said the second `_nowMs()` runs "a few microseconds
later". It does not: between the two clock reads sits a **complete I2C status
read through the application's transport callback**. That callback is bounded by
`i2cTimeoutMs`, whose default (50 ms) is five times `nvmReadyTimeoutMs` (10 ms),
and the driver only *passes* the timeout to the callback rather than enforcing
it. A successful-but-slow read — for example a shared-bus mutex wait or task
scheduling delay — can cross the
deadline. All three call sites run outside any `ScopedTimeContext`, so both
reads consult the caller's real clock. Bosch §6.2 explicitly says this sensor
does not perform clock stretching; that was an incorrect device-specific
example in the previous verification.

Decisively: `test/test_basic.cpp` **already contained a registered, passing test
that enters this branch** at the time the finding was written — it advances the
fake bus clock 2 ms inside the status read against a 1 ms deadline, across a
`uint32_t` wrap, and asserts `Err::TIMEOUT`. The claim was refutable against the
repository's own green suite. A later quiesce status read shifted that test's
wrap before the NVM check; this pass repaired the starting time so the NVM
read again crosses the deadline through `uint32_t` wrap.

## 17. `final_verdict()` ignores `RESULT_SKIPPED_UNSAFE` — UNREACHABLE

`final_verdict()` does omit the state from both buckets, but it cannot reach a
live verdict. `RESULT_SKIPPED_UNSAFE` has exactly one assignment in the file,
inside `dry_run_result()`, whose only two call sites are inside the
`if args.dry_run:` block; and `final_verdict()` returns `INCOMPLETE` before
either bucket is consulted whenever `dry_run` is set. Independently, destructive
specs are never added to `executable` in a live run without
`--include-destructive`, and with it they are executed rather than skipped.
Adding the state to the review bucket would be dead code.

## 21. Raw mode bits `2` — no change, correctly
`modeBitsToStr` decodes both `1` and `2` as FORCED per Table 25, while
`parseMode` accepts only `0`, `1`, `3`. That is deliberate: the typed CLI
accepts canonical `Mode` values, and raw register display still decodes both
encodings.

## 21. Datasheet in the package — retained, deliberately
The 1.6 MB PDF dominates package size (the previous verification measured
1,755,009 bytes with it and 130,671 without, about 92.6%). Exact archive size
changes with source and metadata. Retained as primary source evidence. This was the explicit
decision the finding asked for.

---

# Verified correct — no change needed

The three channel formulas in `_compensate()` follow Bosch §4.2.3, including
`t_fine`, the 64-bit pressure path with every shift constant, and the
humidity `>>15 / >>7 / >>4` correction chain and the `0 … 419430400` clamp.
Also verified: `dig_H4`/`dig_H5` shared-nibble packing and 12-bit sign extension; the 26-byte
`0x88..0xA1` and 7-byte `0xE1..0xE7` calibration bursts including the `0xA0`
gap; the coherent 8-byte `0xF7..0xFE` data burst; the `t_measure,max` formula;
the BME280-specific `t_sb` `110 = 10 ms` / `111 = 20 ms` encoding; the
config-apply write order; and the wrap-safe deadline arithmetic. The
`INT64_MIN / -1` case is explicitly guarded. This is an arithmetic and contract
review, not a claim of byte-identical source or exhaustive equality for arbitrary
calibration bytes: the implementation widens intermediates, checks overflow,
returns errors for undefined reference cases, and exposes integer pascals rather
than Bosch's Q24.8 pressure value. The synthetic-vector, signed-calibration,
overflow, zero-denominator, and humidity-clamp tests exercise those contracts.

---

# Corrections to this document

Errors in the audit as originally written, all found by targeted
re-verification. Listed so the record is honest.

| Where | Error | Correction |
|---|---|---|
| 3/5/7, September verification | Quiesce was treated as proven by `measuring == 0` after a sleep write | An ignored sleep request can leave NORMAL standby; September 8 adds mode readback before calibration/settings access |
| 21, health color | State-based coloring was treated as complete for both CLIs | IDF `drv` still used a zero/nonzero failure ternary; fixed September 8 |
| O3 / migration notes | 11 recovery-alias tests; four example settings callers | Baseline has 13 distinct tests and 14 callers, of which four discard status; replace drifting counts with caller categories |
| B | "real archive members carry a `BME280/` prefix" | False; `pio pkg pack` emits flat, unprefixed members. Still latent, but because `export.include` admits no dot-entry |
| C | "the seven headers it includes" | Six are included directly; `BuildConfig.h` arrives via `Log.h` |
| D | "122 to 84 lines" | 122 to **83** |
| E | tombstones for examples "deleted several releases ago" | Those paths appear nowhere in the repository's 128-commit history |
| F | cleanup rows become `FINAL_CLEANUP_EXCEPTION` | They are `SERIAL_WRITE_EXCEPTION` / `FAIL`; `PortNotOpenError` is an `OSError` caught one level lower |
| F | "under default flags" | Needs an opted-in duration soak first (`--soak-duration-s` defaults to 0) |
| 1 | Quantization/noise estimates presented as rejection rates | Resolution depends on filtering/oversampling; rates need additional statistical and device assumptions and are not established by the datasheet |
| 1 | Sentinel values treated as fixed physical conditions across devices | Compensation depends on calibration and `t_fine`; no universal pressure, altitude, humidity, or 20–30 C sentinel temperature follows |
| 3 | "For `begin()` ... the device is in sleep and the reading is sound" | False; `begin()` did not reset or quiesce, and contradicted finding 4 |
| 3 | "With a short standby ... this is not rare" | Not derivable; the datasheet gives no NVM-copy duration |
| 3 | Proposal: skip the `im_update` gate for non-reset resync | **Unsafe.** Would risk committing torn calibration. Replaced with quiesce-then-gate |
| 4 | "~8 % at 125 ms standby" | 9.30 / 134.3 is ~7%, but only under a simplified timing model; neither percentage is a measured failure probability |
| 4 | Deleting the first check removes the failure probability | It does not; the benefit is convergence on retry |
| 5 | Drop follows from the device measuring | §3.3.1 requires an outstanding delayed mode change; verified by execution |
| 6 | `pollJob()` "restores mode on failure via `_failJob()`" | `_failJob()` issues no register write; hardware mode depends on the writes that reached the device, and is not guaranteed SLEEP |
| 6 | `setFilter`/`setStandby` "byte-for-byte identical apart from one argument" | 5 differing lines of 57 |
| 6 | "removes roughly 200 lines" | ~130 net after shared helpers |
| 7 | Proposal prescribed masking the mode field | Unnecessary; `registerModeForConfig()` already maps FORCED -> SLEEP |
| 8 | Previous verification denied the symbol `Δt_standby` | That correction was false: Table 1 and §9.3 explicitly use it |
| 8 | `MS_62_5` unmentioned | Also affected: 63 ms nominal against 78.1 ms worst case |
| 9 | "102 diagnostic messages" | **138** call sites, 66 distinct strings; 102 was a naive one-line grep |
| 9 | `Status::Error(Err::X, 0)` is ambiguous | **False**; verified by compiling. Only ambiguous where `int32_t != int` |
| 9 | Six remaining example/test message sites | Nine example literal arguments removed in this pass; native compatibility test retained |
| 10 | "Nothing in this repository or its history uses the older name" | **False**; 3 of 7 are used, 2 of them enforced by CI gates |
| 11 | "five" duplicated fields | Six |
| 12 | Proposed `canMeasure()` | Withdrawn; it omitted timebase, mode, job exclusivity and pending measurement |
| 13.1 | TIMEOUT branch unreachable | **False**; a passing test already covered it |
| 13.1 | BME280 clock stretching as the delay example | §6.2 says the sensor does not stretch the clock; scheduling/shared-bus waits can cross the deadline |
| 14 | All four measured output sizes | All wrong — measured on the ANSI-coloured transcript, not `clean_output` |
| 14 | Excerpt reaches `results.csv` and `manifest.json` | Neither; only `summary.json` |
| 14 | Corrected `cfg` length of 347–353 | 349–355 when preserving serial CRLF like the runner; universal-newline translation lost two characters |
| 15 | No shipped budget validators; reachable via `--commands` | Twelve shipped construction sites use them; start and deadline-cancel have separate zero-callback validators, owner-cancel requires a zero-callback output token, and custom commands have none |
| 16 | Title "does not check `spec.expected`" | Overstated; true for 27 of 191 specs |
| 17 | Whole finding | Withdrawn as unreachable |
| 18 | Proposed CMake snippet | Contained a real bug; `NAME` does not normalise `..` |
| 19 | "exercises the short-write branch at a threshold that cannot occur" | Neither threshold was exercised |
| 20 | Two table rows and the flag count | Writers row conflated with the live driver; 52 flags, not 48 |
| O4 | Exactly 20 runner source assertions | 23 required substrings in the current checker |
| Verified correct | Four routines match Bosch exactly | Three channel formulas in one routine, with intentional checked arithmetic and pressure-unit conversion |
| 21 | Short-read `OK` is "the one place" | Two places, and already documented at the baseline |
| 21 | "Two 5 MB transcripts (~10.4 MB total)" | Three transcripts; ~10.4 MB is the three-file total |
| various | Line citations | Drifted 7–18 lines; regenerate rather than spot-fix |

A pattern worth noting for future audits: the errors cluster in **quantitative
claims measured with the wrong tool** (grep counts, ANSI-coloured text) and in
**prescriptions** rather than diagnoses. Findings 3, 10 and 12 had sound
diagnoses and unsound proposals, and the implementers improved on all three.
