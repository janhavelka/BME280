# Code Audit Follow-Up — Work Order

Baseline: `main` at `99a0af7`. CI green (all six jobs), 191/191 native tests,
87/87 HIL parser tests, core compiles clean under
`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`.

This document is the remaining work after `docs/CODE_AUDIT.md` (21 findings) and
`docs/CODE_AUDIT_RESOLUTION.md` (the implementation pass). It is written to be
handed directly to an engineer or an AI coder.

---

## How this list was produced (and why you should not skip the verification steps)

The resolution report was **not** taken at face value. Two independent read-only
reviews re-derived the state from source:

- an adversarial read of `git diff 14ca400..HEAD -- src include`;
- a **mutation test**: the suite was rebuilt standalone, each fix was reverted
  one at a time, and the suite re-run to see whether any test actually died.

That is what produced the items below. Every P1 item comes with a mutation that
**survived** — meaning the current suite would not notice if that behaviour
regressed.

The same method found something the resolution report got wrong in a way local
validation could not see: **CI had been red for four consecutive commits.** The
report's "Validation boundary" section claimed strict Doxygen generation passed;
it passed on the local Doxygen 1.15 and failed on CI's older Doxygen. Separately,
the finding-18 fix was shipped without ever building ESP-IDF (the report says so
honestly) and broke both `esp-idf-build` jobs. Both are now fixed at `d4d491f`
and `99a0af7`.

**Ground rules for this work order:**

1. Do not trust `docs/CODE_AUDIT_RESOLUTION.md`, this document, or any comment.
   Re-derive each claim from source before changing anything.
2. A green local gate is not done. **Check `gh run list` after pushing.** Local
   Doxygen and CI Doxygen disagree; local has no `idf.py` at all.
3. For every fix, first write the test that fails, then make it pass. If you
   cannot make the suite fail by reverting your fix, the test is not a test.
4. Do not widen public API or remove public symbols. Items marked `3.x` are
   deliberately deferred.

---

## Confirmed genuinely fixed — no action needed

Recorded so nobody re-audits them. Each was verified by reverting it and
watching tests die.

| Finding | Evidence |
|---|---|
| 1. Skipped-channel sentinels | `src/BME280.cpp:2944-2946` derives validity from `_config.osrs*` only. Zero references to `0x80000`/`0x8000`/`RAW_*_SKIPPED` remain in `src/`. Restoring the sentinel comparison kills 3 tests; forcing all channels valid kills `test_skipped_sentinels_are_explicit_validity_flags`. Both directions pinned. |
| 2. Multi-byte writes | `src/BME280.cpp:2412-2432` emits address/value pairs into a 32-byte buffer bounded by `len > MAX_WRITE_LEN`; max index 31, no overflow. `len == 1` still emits exactly `[reg, value]`. The fake bus **rejects** odd-length writes (`test/test_basic.cpp:126`, detail `-5`), so reverting the encoding fails immediately. |
| 3. `im_update` outside reset | All four paths quiesce before the NVM gate: `begin()` `:624`, `recover()` `:793`, staged INIT `:1396→:1561`, staged RESYNC `:1816→:1563`. Note the implementation **rejected** the audit's proposal (skip the gate) in favour of quiescing first — that is the better fix. Removing the quiesce kills 6 tests (sync) / 5 (staged). |
| 4. Apply while measuring | `begin()` still returns `BUSY` mid-conversion, but it is now genuinely retryable: SLEEP is latched first, so no new conversion starts and a retry within ~112 ms succeeds. `test_begin_queues_sleep_then_reports_busy_when_measuring_persists` asserts dirty-then-clean across the retry. Re-inserting the old pre-sleep gate kills 9 tests. |
| 5. Lost `ctrl_hum` write | Only two sites write `0xF2`, both after confirmed quiescence and both unconditionally followed by `ctrl_meas`. `test_humidity_oversampling_writes_ctrl_hum_then_ctrl_meas` asserts the exact 3-write log. |
| 7. Config readback | `_verifySettingsReadback` `:2611-2654` reads `0xF2..0xF5` in one burst; `status` (`values[1]`) is read but never compared. Masks: `ctrl_hum 0x07`, `ctrl_meas 0xFF`, `config 0xFD`. FORCED→SLEEP expectation is correct. **The mismatch path is genuinely testable** — the fake reports a byte different from what was written via `settingsReadbackOverrideEnabled`, and both sync and staged mismatch tests exist. Every mask/comparison mutation dies. |
| 19. Wire stub | `test/stubs/Wire.h:8-10` exposes `I2C_BUFFER_LENGTH = 128`, shared with `examples/common/I2cTransport.h:103,157`. Reverting to 64 is a **compile error**. 128/129 boundary tests distinguish `detail == 129` from `detail == -4`. |

Also confirmed by hand against the Bosch reference: the compensation math is
still exactly right (temperature, int64 pressure with all shift constants,
humidity including the `>>15/>>7/>>4` chain and the `419430400` clamp),
`signExtend12`, and the shared-nibble H4/H5 decode.

One audit claim was **wrong** and is withdrawn: finding 17
(`RESULT_SKIPPED_UNSAFE` in the live verdict) — the value is assigned only
inside `dry_run_result()` (`tools/run_i2c_hil.py:2942`) and `final_verdict()`
returns `INCOMPLETE` before reaching the buckets when `dry_run` is true. The
rejection in the resolution report is correct. Finding 13's
`_waitForNvmReady()` "unreachable TIMEOUT" claim is likewise correctly rebutted:
a bounded transport callback can burn the whole `nvmReadyTimeoutMs` between the
two clock reads.

---

## P1 — the guard that would not notice if it broke

### 1. `APPLY_WAIT_AFTER_SLEEP` boundedness is completely unverified

**This is the most serious item in this document.**

`src/BME280.cpp:1546-1550`:

```cpp
if (deadlineReached(nowMs, _jobDeadlineMs) ||
    _jobWaitPolls >= MEASURING_READY_MAX_POLLS) {
  return _failJob(Status::Error(Err::TIMEOUT), instructionsUsed);
}
```

Replacing that condition with `if (false)` — turning the staged quiesce into an
**unbounded wait for `measuring == 0`** — leaves the suite at **191/191 green.**
Neither the deadline branch nor the poll cap is ever taken by any test.

This phase was *introduced by the finding-4 fix* and now sits on the critical
path of every staged init, apply-config, apply-settings, resync and soft-reset.
It is exactly the liveness failure mode finding 3 warned about, and
`docs/CODE_AUDIT_RESOLUTION.md` explicitly claims "Staged jobs poll the same
transition within their callback/deadline bounds". That claim is currently
unbacked.

`test_staged_successful_path_callback_caps_include_settings_readback`
(`test/test_basic.cpp:2088`) sets `measuringStatusReadsRemaining = 255`, which
sits exactly *at* `MEASURING_READY_MAX_POLLS` and exits naturally — it walks up
to the boundary without crossing it.

**Do:** add a staged apply/init test where the fake keeps `measuring` set past
the cap (or `statusReadNowAdvanceMs` pushes `nowMs` past `_jobDeadlineMs`), and
assert `JobState::FAILED` with `Err::TIMEOUT` originating in
`APPLY_WAIT_AFTER_SLEEP`. Cover **both** exit conditions — deadline and poll
cap — since either alone leaves the other unverified.

**Verify:** with the new test in place, `if (false)` must fail the suite.

### 2. The `ctrl_hum` write-failure branch is unguarded — a coverage regression

`src/BME280.cpp:2719-2726`:

```cpp
if (writeCtrlHum) {
  const uint8_t value = buildCtrlHum(settings.osrsH);
  st = writeRegs(cmd::REG_CTRL_HUM, &value, 1);
  if (!st.ok()) {
    _markHardwareConfigDirty(st);
    return st;
  }
}
```

Making that branch swallow the error (`st = Status::Ok()`), losing both the
error return **and** `_markHardwareConfigDirty()`, leaves the suite at
**191/191 green.**

The neighbouring branches *are* covered: the `config` write failure kills 9
tests, the final `ctrl_meas` restore failure kills
`test_humidity_latch_address_nack_after_ctrl_hum_marks_resync_required`. Only
`ctrl_hum` is unguarded — and `ctrl_hum` is precisely the register the datasheet
(§3.3.1) says can be silently dropped.

This is a **regression, not an old gap**: `test_partial_config_restore_failure_marks_hardware_dirty`
was deleted and its body reused verbatim for the new
`test_config_readback_failure_marks_hardware_dirty` (`test/test_basic.cpp:3248`)
with a read-failure injection substituted for the write-failure injection.

**Do:** restore a `failWriteOnCall` injection targeting the `ctrl_hum` write
itself (write #2 of the `CTRL_HUM` plan), asserting the transport error
propagates unchanged and `hardwareConfigDirty()` is set. Keep the new readback
test as well — they cover different branches.

### 3. Finding 8's standby tolerance table is verified at 2 of 8 values

`src/BME280.cpp:184-196`. gcov shows only `MS_125` and `MS_1000` ever execute.
Setting the other six cases **and the `default`** to `0` — fully reverting
finding 8 for those standby values — leaves the suite at **191/191 green.**

The entire point of finding 8 is the tolerance table.

**Do:** add a table-driven assertion over all eight `Standby` values that the
freshness budget includes the +25 % term. Expected values (ceil of nominal ×
1.25): `MS_0_5→1`, `MS_62_5→79`, `MS_125→157`, `MS_250→313`, `MS_500→625`,
`MS_1000→1250`, `MS_10→13`, `MS_20→25`.

### 4. `FakeBus` does not model soft reset at all

`fakeWrite` has no special case for `0xE0 = 0xB6`; it just stores the byte.
Real silicon runs a full power-on-reset: `ctrl_hum`, `ctrl_meas` and `config`
return to `0x00` and `im_update` is raised.

So **every** `SOFT_RESET` job — including the new
`APPLY_CTRL_MEAS_SLEEP → APPLY_WAIT_AFTER_SLEEP → NVM → APPLY_VERIFY` path — is
validated against a device that never actually resets. That is the largest
remaining gap between the fake and the silicon, and it sits under a code path
the last change set rewrote.

**Do:** teach `FakeBus` that a `0xB6` write to `0xE0` clears `ctrl_hum`,
`ctrl_meas` and `config` to `0x00` and raises `im_update` for a configurable
number of subsequent status reads. Then re-run the existing soft-reset tests and
fix whatever legitimately breaks — expect the new `APPLY_VERIFY` phase to have
opinions about registers that reset underneath it.

---

## P2 — real but lower-risk

### 5. Finding 6 is only half done: the staged path still resets IIR history

The resolution claims one shared apply engine that "writes only the groups
required by the requested change, avoiding needless `config` writes that reset
IIR history". That is true of the **synchronous** path only.

Two implementations of sleep→write→restore remain (4 → 2, not 4 → 1):

1. synchronous: `_quiesceSettingsSynchronously` + `_writeSettingsAfterQuiesceSynchronously` (`src/BME280.cpp:2671-2750`), which uses `SettingsWritePlan` and is selective;
2. staged: `pollJob` phases `APPLY_*` (`src/BME280.cpp:1505-1634`), which has **no** `SettingsWritePlan` and writes `0xF5` unconditionally at `:1572-1586`.

Observable consequence: `startApplySettingsJob()` with a **mode-only** change
resets the IIR filter history; `setMode()` does not. The header documents the
selective behaviour on the setters (`include/BME280/BME280.h:856-878`) but says
nothing about this asymmetry.

**Do:** either give the staged path the same `SettingsWritePlan` selectivity, or
document the asymmetry precisely on `startApplySettingsJob()` /
`startApplyConfigJob()`. Prefer the former; it removes the surprise instead of
naming it. Add a test asserting a mode-only staged apply does not write `0xF5`.

### 6. Sync and staged apply have different cache semantics, undocumented

The synchronous path calls `_invalidateSampleCache()` on success
(`src/BME280.cpp:2748`). The staged `COMPLETE` handler for `APPLY_CONFIG`
(`:1826-1828`) does not — it relies on `_advanceConfigGeneration()` marking the
sample `STALE_AFTER_CONFIG_CHANGE`.

This is intentional (`test_staged_apply_advances_generation_without_freshening_old_sample`
pins it) but the two halves have observably different post-apply state and
nothing says so.

**Do:** document it on both entry points, or unify.

### 7. Three unreachable branches in `_compensate`

`src/BME280.cpp:2961`, `:2995`, `:3074` — the `if (!raw.temperatureValid)` /
`pressureValid` / `humidityValid` guards. `isValidMeasurementSelection`
(`:265-276`) guarantees temperature is enabled for any accepted tuple, and the
P/H guards sit inside `else` branches whose condition derives from the *same*
`_config.osrs*` field that set the flag. They can never fire.

**Do:** delete them, or keep them and say in a comment that they are
defence-in-depth against a future `validateSettings()` change. Do not leave them
looking like live error handling.

### 8. `_waitForNvmReady()` does not wait

`src/BME280.cpp:2753-2771` reads status once and computes its deadline from
`_nowMs()` immediately before the read. The `TIMEOUT` branch is reachable (see
above) but rare; in practice the function returns `OK` or `BUSY`.

This is the documented contract — "returns BUSY or TIMEOUT instead of hiding a
polling loop" — so the behaviour is right. The **name** is not.

Note the sharp edge this creates: `softReset()` (`:2233`) writes `0xB6` and
reads status with no delay, so on real hardware `im_update` is still set (the
datasheet's own self-test allows 2 ms for a soft reset) and the call normally
returns `BUSY`, requiring a caller retry. The retained HIL evidence shows
exactly this as `PASS_WITH_RESET_BUSY_RECOVERED` rows, so it is known and
handled — but a first-time integrator will hit it immediately.

**Do:** rename to `_checkNvmReadyOnce()` (private, no API impact) and make
`softReset()`'s Doxygen say plainly that `BUSY` is the *expected* first result
on hardware and that `startSoftResetJob()` is the non-retrying path.

### 9. Remaining `FakeBus` permissiveness

Beyond item 4, in descending order of importance:

- **§3.3.1 and §5.4.6 are not modelled.** The fake never drops a `ctrl_hum`
  write while a mode change is queued, and never ignores a normal-mode `config`
  write. These are the exact mechanisms findings 5 and 7 exist for, so the
  finding-5 ordering fix is verified only as a byte-sequence assertion, never as
  an outcome.
- **Read-only registers are writable.** `test/test_basic.cpp:3471` asserts
  `bus.reg[REG_STATUS] == 0x02` and `:3494` asserts
  `bus.reg[REG_PRESS_MSB] == 0xAB`. `0xF3` and `0xF7..0xFE` ignore writes on
  hardware. Harmless today (the assertions are really about payload encoding)
  but it is the fake certifying a write that cannot land.
- **Auto-increment is incomplete** for `REG_CHIP_ID`, `REG_STATUS`,
  `REG_CALIB_H1` (`:181`, `:204`, `:213` fill only `rxData[0]`). The driver only
  does 1-byte reads there, so nothing is affected today.

**Do:** at minimum model the §3.3.1 drop, so finding 5 is verified by outcome.

### 10. One near-vacuous test iteration

`test_settings_readback_ignores_status_and_reserved_bits`
(`test/test_basic.cpp:3307`): the loop's second iteration overrides `REG_STATUS`
(`values[1]`), which `_verifySettingsReadback` never reads. That iteration
cannot fail under any driver mutation. The other two iterations are real.

**Do:** keep it (it documents intent) but add a comment saying it is a
negative control, or assert the exclusion directly.

---

## P3 — process and documentation

### 11. CI was red for four commits and nobody noticed

`validate-library` failed from `14ca400` through `de39968`; `esp-idf-build`
failed from `184eee9`. Meanwhile `docs/CODE_AUDIT_RESOLUTION.md` listed "strict
Doxygen generation" as passing.

Root cause is structural, not careless: the local gate and CI are not the same
gate. Local Doxygen is 1.15 and tolerant of the duplicate-`README.md` mainpage;
CI's Ubuntu Doxygen is not. Local has no `idf.py` at all, so a change to the
ESP-IDF build graph could not be validated before it shipped.

**Do:**
- Add "confirm `gh run list` is green for the pushed commit" as the final step
  of the validation gate in `README.md`, and make `CONTRIBUTING.md` point at it.
- Note in the gate that `doxygen Doxyfile` passing locally does not imply CI
  passes, and that the ESP-IDF build is CI-only.

### 12. The 3.x deferral backlog has no durable home

Findings 10 (compatibility aliases), 11 (duplicate `SettingsSnapshot` sample
fields) and 21's `getSettings()` return type are deferred to 3.x — recorded only
in prose inside a resolution report. Nothing in `CHANGELOG.md`, `README.md` or
`docs/README.md` mentions them. When 3.0 planning starts this will be
archaeology.

Still-present symbols to remove at 3.x:
`Err::CONVERSION_NOT_READY` (`Status.h:21`), `JobKind::RECOVERY`
(`BME280.h:48`), `startRecoveryJob()`, `driverState()` (`:672`),
`JobPollResult::instructionsUsed` (`:237`), `cmd::REG_DIG_H5_LSB`
(`CommandTable.h:89`, historically misnamed), `VERSION_INT`, the flattened
`SettingsSnapshot` sample fields, and `Status getSettings()` → `void`.

**Do:** create `docs/MIGRATION_3X.md` with that list and one line per item on
why, and link it from `docs/README.md`. Cheap now, expensive later.

### 13. `docs/CODE_AUDIT.md` is orphaned while its resolution ships to consumers

`docs/CODE_AUDIT_RESOLUTION.md` is in `docs/README.md`, `README.md` and
`library.json` `export.include`. `docs/CODE_AUDIT.md` is in none of them. So a
downstream consumer receives "Finding 7: fixed" with no way to see what finding
7 was.

Two coherent options — pick one:
- treat both as internal working documents: drop the resolution from
  `library.json` `export.include` and from the README documentation list, and
  reference both only from `docs/README.md`; or
- ship both: add `docs/CODE_AUDIT.md` alongside the resolution everywhere.

The first is more defensible — a firmware integrator consuming this library via
`lib_deps` has no use for an internal audit trail, and the package is already
90 % datasheet PDF.

### 14. Finding 20 (HIL runner split) remains deferred

`tools/run_i2c_hil.py` is 4717 lines, down from 4884 (the dead
`duration_command_fits` helper and the 174-line embedded `parser_self_test` were
removed). The structural split — command catalogue, report writers, parsers — is
still open, as is the source-text coupling in `check_hil_contract.py:412` that
will fight any refactor. The original proposal in `docs/CODE_AUDIT.md` §20 still
stands; it was deliberately deferred to keep this correctness pass behaviour-
neutral, which was the right call.

---

## Definition of done for this work order

- P1 items 1–4 closed, each with a test that **fails when the fix is reverted**.
- P2 items 5–6 either fixed or documented on the public API.
- `docs/MIGRATION_3X.md` exists and is linked.
- Full gate green **and** `gh run list` green for the pushed commit.
- `docs/CODE_AUDIT_RESOLUTION.md` updated, or superseded by this document, so
  the two do not disagree.
