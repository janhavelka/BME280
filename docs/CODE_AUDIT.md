# BME280 Code Audit

Audit of `include/BME280/*.h`, `src/BME280.cpp`, the examples, and the Python
tooling, against the Bosch BME280 datasheet BST-BME280-DS001-24 rev. 1.24
(`BME280_datasheet.pdf`, cover page and history table; the body-page footers
still read 1.23, which is Bosch's own stale footer).

| | |
|---|---|
| Audited baseline | `3b3fad1` (v2.1.0) |
| Fixes landed in | `558a31f`, `184eee9`, `de39968`, `d4d491f`, `99a0af7` |
| Re-verified at | `084b045`, 2026-09-04 |
| Items | 28 (A–G plus findings 1–21, with sub-items) |

Every finding was re-verified independently: each factual claim was re-checked
against the datasheet and against both code baselines, and several were checked
by compiling and running the driver rather than by reading it. **The audit as
originally written contained errors.** They are corrected in place below, and
listed in full in [Corrections to this document](#corrections-to-this-document)
so the change history is not lost.

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
| 6 | Config-write sequence implemented four times | **DONE**, with [one residual](#o2-staged-apply-is-not-selective) |
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

This is the only section that requires code changes.

## O1. Four fixes have no test that would catch a regression

Proven by mutation: each fix was reverted in isolation and the suite re-run.
All four mutants **survived** at 191/191 green.

**Important framing:** in every case the shipped code is *correct* — I verified
each one by execution, not by reading. These are regression risks, not live
defects. Do not "fix the code"; add the missing test.

1. **`APPLY_WAIT_AFTER_SLEEP` bounds.** Replacing the deadline and poll-cap
   guard (`src/BME280.cpp:1546`) with `if (false)` keeps the suite green.
   The guard does work — driving a staged apply against a device whose
   `status.measuring` sticks high yields `TIMEOUT` in `APPLY_WAIT_AFTER_SLEEP`
   after 256 polls with a frozen clock (poll cap) and after 2 polls with the
   clock advancing 50 ms per poll (deadline). Add a test for **both** exits;
   covering one leaves the other unpinned.

2. **The synchronous `ctrl_hum` write-failure branch** (`src/BME280.cpp:2719`)
   can swallow its error and its `_markHardwareConfigDirty()` call with the
   suite still green. `test_humidity_ctrl_hum_failure_marks_dirty_and_preserves_error`
   (`test/test_basic.cpp:3201`) is **misnamed** — it injects at
   `failWriteOnCall = bus.writeCalls + 1u`, and the real write order for
   `setOversamplingH()`, captured from a running driver, is:

   ```
   write #1 -> 0xF4 = 0x24   ctrl_meas SLEEP (quiesce)
   write #2 -> 0xF2 = 0x03   ctrl_hum          <-- the intended target
   write #3 -> 0xF4 = 0x24   ctrl_meas final
   ```

   `_quiesceSettingsSynchronously()` writes unconditionally, so write #1 is
   never `ctrl_hum`. Change the injection to `+ 2u`, and keep the existing test
   too — it covers the quiesce write, which is also worth pinning. This is the
   one register the datasheet (§3.3.1) says can be silently dropped.

3. **The standby tolerance table** (`src/BME280.cpp:184`) is executed at only
   two of its eight values. Zeroing the other six plus the `default` keeps the
   suite green. All eight values are arithmetically correct against
   ceil(true nominal x 1.25) — `MS_0_5` 1, `MS_62_5` 79, `MS_125` 157,
   `MS_250` 313, `MS_500` 625, `MS_1000` 1250, `MS_10` 13, `MS_20` 25. Add a
   table-driven assertion over all eight.

4. **`FakeBus` does not model soft reset.** A `0xB6` write to `0xE0` is stored
   like any other byte. Real silicon returns `ctrl_hum`, `ctrl_meas` and
   `config` to `0x00` and raises `im_update`. Every `SOFT_RESET` job — including
   the reworked `APPLY_CTRL_MEAS_SLEEP -> APPLY_WAIT_AFTER_SLEEP -> NVM ->
   APPLY_VERIFY` path — therefore runs against a device that never resets.
   Teach the fake the reset, then fix whatever legitimately breaks; expect
   `APPLY_VERIFY` to have opinions about registers that reset underneath it.

## O2. Staged apply is not selective

Finding 6 collapsed four copies of the config-write sequence into two, which
was the goal. But `SettingsWritePlan` (`include/BME280/BME280.h:1042`) is
consumed only by `_writeSettingsAfterQuiesceSynchronously()`
(`src/BME280.cpp:2706`). The staged `pollJob()` path has no plan parameter:
`APPLY_CONFIG` (`src/BME280.cpp:1572`) writes `0xF5` unconditionally, so the
staged path is permanently equivalent to `SettingsWritePlan::FULL`.

Observable consequence: `startApplySettingsJob()` with a mode-only change
rewrites `config` and therefore **resets IIR filter history**; `setMode()` does
not. The header documents the selective behaviour on the setters
(`include/BME280/BME280.h:856-878`) and says nothing about this asymmetry.

Either give the staged path the same selectivity, or document it on
`startApplySettingsJob()` and `startApplyConfigJob()`. Prefer the former. Add a
test asserting a mode-only staged apply does not write `0xF5`.

## O3. Deferred to 3.x

Breaking changes, deliberately not made in a 2.x correctness pass. **Three of
the seven aliases in finding 10 are load-bearing** and cannot be removed with a
header edit alone:

| Symbol | Blocker |
|---|---|
| `Err::CONVERSION_NOT_READY` (`Status.h:21`) | none — definition only |
| `JobKind::RECOVERY` (`BME280.h:48`) | none — definition only |
| `driverState()` (`BME280.h:672`) | none — definition only |
| `cmd::REG_DIG_H5_LSB` (`CommandTable.h:89`) | none — definition only, and **misnamed**: `0xE6` holds `dig_H5[11:4]`, the high bits (datasheet Table 16) |
| `startRecoveryJob()` (`BME280.h:559`) | both shipped examples, 11 tests, and required tokens in `check_cli_contract.py:159` and `check_idf_example_contract.py:219` |
| `JobPollResult::instructionsUsed` (`BME280.h:237`) | both shipped examples print it; the `"Instructions:"` token is required by `check_idf_example_contract.py:206` |
| `VERSION_INT` (`Version.h:67`) | emitted by `scripts/generate_version.py:235`, asserted verbatim by `check_release_metadata.py:41` |

Also for 3.x: the six duplicated `SettingsSnapshot` fields (finding 11) and
`Status getSettings()` -> `void` (finding 21).

## O4. HIL runner module split (finding 20)

`tools/run_i2c_hil.py` is 4717 lines, down from 4884 after the dead
`duration_command_fits()` and the 172-line embedded `parser_self_test()` were
removed. The structural split is still open; the original proposal stands. It
was correctly deferred to keep the correctness pass behaviour-neutral.

Note the coupling that will fight any refactor: `check_hil_contract.py:413`
asserts exactly 20 literal substrings against the runner's *source text*, and
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
passing. Now `removeprefix("./")` (`tools/check_package_contents.py:100`).

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

## 1. Skipped-channel sentinels — DONE
`_readRawData()` marked a channel invalid when its raw ADC equalled the Bosch
skipped value, and `_compensate()` then failed the whole sample. `0x80000` and
`0x8000` are the *reset and skipped* register values (Tables 18/20/23/24); the
datasheet nowhere states they cannot occur as genuine output.

Recomputed with the Bosch reference routines:

- **Temperature.** With `dig_T1 = 27504`, `dig_T2 = 26435`, `dig_T3 = -1000`,
  `adc_T = 0x80000` gives `t_fine = 135479`, T = 26.46 C. The value is
  insensitive to `dig_T3`, but part-to-part `dig_T1`/`dig_T2` spread moves it
  across roughly 20–30 C — so *some* ordinary room temperature sits on the
  sentinel for essentially any part. Gradient 31.9 counts per 0.01 C; Table 14
  noise 0.005 C at x1 gives sigma ~16 counts. **With the IIR filter off** (the
  driver default) §3.4.3 gives resolution `16 + (osrs_t - 1)` bit, so at x1 the
  step is 16 counts — about one sigma, up to ~40 % of samples. With the filter
  on, resolution is always 20 bit (§3.4.4, Table 17), the step is 1 count and
  the rate falls to ~2.5 %.
- **Pressure** is the strongest case and was missing from the original text.
  `adc_P = 0x80000` compensates to ~81.9 kPa (~819 hPa, about 1750 m) — Denver,
  Mexico City, Bogota. Gradient 0.172 Pa/count against Table 12 noise 3.3 Pa
  (~19 counts) and a 16-count step: ~35 % of samples at a populated altitude.
- **Humidity** is real but an order of magnitude rarer than temperature.
  Humidity is always 16-bit, so there is no coarse bin. `adc_H = 0x8000` gives
  ~72 %RH; Table 11 noise 0.07 %RH at x1 is ~13 counts against a 1-count step,
  so ~3 % of samples.

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
removing the quiesce kills 6 tests (sync) and 5 (staged).

## 4. `_applyConfig()` failed `begin()` on a measuring device — DONE
The first `_ensureConfigWriteReady()` hard-failed `begin()` with `BUSY` on a
device left in normal mode. Writing `ctrl_meas` to sleep is legal at any time —
§3.3.1 says the transition is deferred, not rejected. The second check is
genuinely required (§5.4.6 for `config`, §3.3.1 for `ctrl_hum`), and the staged
INIT path already got this right.

Two corrections to the original text. The probability is **~7 %**, not 8 %: with
the default x1/x1/x1, `t_measure` is 9.30 ms (§9.1 max, matching
`estimateMeasurementTimeUs()` exactly) over a 134.3 ms cycle. And **deleting the
first check does not by itself lower that probability** — the second check
hard-fails on `BUSY` too, ~75 us later, when `measuring` is still set. The real
benefit is convergence: the sleep write now reaches the device before the driver
gives up, §3.3.1 guarantees it executes at the end of the running measurement,
and the caller's retry then finds an idle device instead of re-rolling the same
dice forever.

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

## 6. Four copies of the config-write sequence — DONE, see [O2](#o2-staged-apply-is-not-selective)
The sequence existed in four places. Two corrections to the original text: the
table claimed `pollJob()` restores mode on failure "via `_failJob()`" — **it
issues no register write at all**, restoring only cached state and leaving the
device in SLEEP, which is a fourth distinct failure behaviour rather than a
shared one. And `setFilter()`/`setStandby()` were not "byte-for-byte identical
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
Table 1 gives the standby time accuracy for `t_standby` as typ +/-5 %, max
+/-25 % (condition: full VDD range). The datasheet uses the symbol `t_standby`
and the name "Standby time accuracy"; the original text's `Δt_standby` notation
appears nowhere in the datasheet. The worst case is
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
`test_high_osr_normal_freshness_deadline_is_wrap_safe` — but see [O1.3](#o1-four-fixes-have-no-test-that-would-catch-a-regression).

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

**Shipped:** 138 -> 0 literals in `src/`. The message-bearing overload is
retained for 2.x source compatibility and is still used by 6 call sites in the
examples and tests.

## 12. `isOnline()` overpromises — DONE as documentation
`_updateHealth()` (`src/BME280.cpp:2617`) sets `READY` on any successful tracked
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
all three retained transcripts: `recover` 459–620, `cfg` 347–353, `status`
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
  CLI, and the retained transcripts: `Callbacks used: 0`). Now verb-aware. Worth
  noting the original text omitted: no shipped spec carried the budget
  validator, so the inflation was inert — reachable only through a `--commands`
  file.
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

## 21. Minor items — 8 DONE
Tombstone assertions removed; the tautological reconnect test replaced with
checks of the real argparse default and override; `runSelfTest` no longer
double-counts a failed baseline capture; the CRLF empty-line prompt suppression
is now commented as deliberate; `HealthView` colours driver state by state
rather than by failure count; the ESP-IDF CLI gained `printStressProgress` and
the matching 127-character input limit; `platformio.ini` lost the redundant
`extends = env` and the embedded-only keys moved out of `[env]`; the README
include path was corrected.

Two corrections to the original bullets: the short-read `OK` in
`I2cTransport.h` occurs in **two** places (`:181` short `requestFrom`, `:188`
drained `available()`), not one, and it was **already documented** in
`docs/PRODUCTION_SHARED_BUS_GUIDE.md:152` at the baseline — no action was
needed. And there are **three** tracked transcripts (5.05, 5.23 and 0.10 MB),
not "two ... ~10.4 MB total"; the ~10.4 MB figure is the three-file total.

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
it. A successful-but-slow read — clock stretching, or a shared-bus mutex wait,
exactly what `docs/PRODUCTION_SHARED_BUS_GUIDE.md` exists for — crosses the
deadline. All three call sites run outside any `ScopedTimeContext`, so both
reads consult the caller's real clock.

Decisively: `test/test_basic.cpp` **already contained a registered, passing test
that enters this branch** at the time the finding was written — it advances the
fake bus clock 2 ms inside the status read against a 1 ms deadline, across a
`uint32_t` wrap, and asserts `Err::TIMEOUT`. The claim was refutable against the
repository's own green suite.

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
The 1.6 MB PDF is 92.6 % of the exported tarball (1,755,009 bytes with it,
130,671 without). Retained as primary source evidence. This was the explicit
decision the finding asked for.

---

# Verified correct — no change needed

Checked line by line against the datasheet and recorded so it is not
re-litigated: all four compensation routines match the Bosch reference exactly,
including `t_fine`, the 64-bit pressure path with every shift constant, the
humidity `>>15 / >>7 / >>4` correction chain and the `0 … 419430400` clamp; the
`dig_H4`/`dig_H5` shared-nibble packing and 12-bit sign extension; the 26-byte
`0x88..0xA1` and 7-byte `0xE1..0xE7` calibration bursts including the `0xA0`
gap; the coherent 8-byte `0xF7..0xFE` data burst; the `t_measure,max` formula;
the BME280-specific `t_sb` `110 = 10 ms` / `111 = 20 ms` encoding; the
config-apply write order; and the wrap-safe deadline arithmetic. The
`INT64_MIN / -1` case is explicitly guarded.

---

# Corrections to this document

Errors in the audit as originally written, all found by targeted
re-verification. Listed so the record is honest.

| Where | Error | Correction |
|---|---|---|
| B | "real archive members carry a `BME280/` prefix" | False; `pio pkg pack` emits flat, unprefixed members. Still latent, but because `export.include` admits no dot-entry |
| C | "the seven headers it includes" | Six are included directly; `BuildConfig.h` arrives via `Log.h` |
| D | "122 to 84 lines" | 122 to **83** |
| E | tombstones for examples "deleted several releases ago" | Those paths appear nowhere in the repository's 128-commit history |
| F | cleanup rows become `FINAL_CLEANUP_EXCEPTION` | They are `SERIAL_WRITE_EXCEPTION` / `FAIL`; `PortNotOpenError` is an `OSError` caught one level lower |
| F | "under default flags" | Needs an opted-in duration soak first (`--soak-duration-s` defaults to 0) |
| 1 | 16-count quantisation step stated unconditionally | Only with the IIR filter off; with it on the step is 1 count and the rate falls to ~2.5 % |
| 1 | Humidity implied to have the same ~40 % rate as temperature | Humidity is always 16-bit; ~3 % |
| 1 | Pressure omitted | It is the strongest case: `0x80000` ~= 819 hPa ~= 1750 m |
| 3 | "For `begin()` ... the device is in sleep and the reading is sound" | False; `begin()` did not reset or quiesce, and contradicted finding 4 |
| 3 | "With a short standby ... this is not rare" | Not derivable; the datasheet gives no NVM-copy duration |
| 3 | Proposal: skip the `im_update` gate for non-reset resync | **Unsafe.** Would risk committing torn calibration. Replaced with quiesce-then-gate |
| 4 | "~8 % at 125 ms standby" | ~7 % (9.30 ms over 134.3 ms) |
| 4 | Deleting the first check removes the failure probability | It does not; the benefit is convergence on retry |
| 5 | Drop follows from the device measuring | §3.3.1 requires an outstanding delayed mode change; verified by execution |
| 6 | `pollJob()` "restores mode on failure via `_failJob()`" | `_failJob()` issues no register write; the device is left in SLEEP |
| 6 | `setFilter`/`setStandby` "byte-for-byte identical apart from one argument" | 5 differing lines of 57 |
| 6 | "removes roughly 200 lines" | ~130 net after shared helpers |
| 7 | Proposal prescribed masking the mode field | Unnecessary; `registerModeForConfig()` already maps FORCED -> SLEEP |
| 8 | Symbol `Δt_standby` | The datasheet uses `t_standby` / "Standby time accuracy" |
| 8 | `MS_62_5` unmentioned | Also affected: 63 ms nominal against 78.1 ms worst case |
| 9 | "102 diagnostic messages" | **138** call sites, 66 distinct strings; 102 was a naive one-line grep |
| 9 | `Status::Error(Err::X, 0)` is ambiguous | **False**; verified by compiling. Only ambiguous where `int32_t != int` |
| 10 | "Nothing in this repository or its history uses the older name" | **False**; 3 of 7 are used, 2 of them enforced by CI gates |
| 11 | "five" duplicated fields | Six |
| 12 | Proposed `canMeasure()` | Withdrawn; it omitted timebase, mode, job exclusivity and pending measurement |
| 13.1 | TIMEOUT branch unreachable | **False**; a passing test already covered it |
| 14 | All four measured output sizes | All wrong — measured on the ANSI-coloured transcript, not `clean_output` |
| 14 | Excerpt reaches `results.csv` and `manifest.json` | Neither; only `summary.json` |
| 16 | Title "does not check `spec.expected`" | Overstated; true for 27 of 191 specs |
| 17 | Whole finding | Withdrawn as unreachable |
| 18 | Proposed CMake snippet | Contained a real bug; `NAME` does not normalise `..` |
| 19 | "exercises the short-write branch at a threshold that cannot occur" | Neither threshold was exercised |
| 20 | Two table rows and the flag count | Writers row conflated with the live driver; 52 flags, not 48 |
| 21 | Short-read `OK` is "the one place" | Two places, and already documented at the baseline |
| 21 | "Two 5 MB transcripts (~10.4 MB total)" | Three transcripts; ~10.4 MB is the three-file total |
| various | Line citations | Drifted 7–18 lines; regenerate rather than spot-fix |

A pattern worth noting for future audits: the errors cluster in **quantitative
claims measured with the wrong tool** (grep counts, ANSI-coloured text) and in
**prescriptions** rather than diagnoses. Findings 3, 10 and 12 had sound
diagnoses and unsound proposals, and the implementers improved on all three.
