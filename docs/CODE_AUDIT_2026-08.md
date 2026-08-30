# BME280 Code Audit

# Part 1 - Core driver

Scope: `include/BME280/*.h`, `src/BME280.cpp`, audited against the Bosch BME280
datasheet BST-BME280-DS001-24 rev. 1.24 (`docs/BME280_datasheet.pdf`) and
`docs/BME280_Register_Reference.md`.

Baseline at audit time: 184/184 native tests pass; `src/BME280.cpp` compiles
clean under `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`.

What is **already correct** and needs no change (verified line by line against
the datasheet, recorded so nobody re-litigates it): all four compensation
routines match the Bosch reference exactly, including `t_fine`, the 64-bit
pressure path, the humidity `0 … 419430400` clamp and the final `>>12`; the
`dig_H4`/`dig_H5` nibble packing and 12-bit sign extension; the 26-byte
`0x88..0xA1` and 7-byte `0xE1..0xE7` calibration bursts including the `0xA0`
gap; the coherent 8-byte `0xF7..0xFE` data burst; the `t_measure,max` formula
(`1.25 + 2.3·osrs_t + [2.3·osrs_p + 0.575] + [2.3·osrs_h + 0.575]`); the
BME280-specific `t_sb` `110 = 10 ms` / `111 = 20 ms` encoding; the config-apply
write order (`ctrl_meas`←sleep, `config`, `ctrl_hum`, `ctrl_meas`); and the
wrap-safe deadline arithmetic.

Findings are ordered by severity. Each has a concrete proposal.

---

## Already fixed in this pass

**A. `signExtend12()` performed an implementation-defined narrowing.**
`value |= 0xF000` computed in `int` and assigned back to `int16_t`; correct on
every two's-complement target but formally implementation-defined pre-C++20 and
the only `-Wconversion` warning in the core. Rewritten to stay inside each
type's value range, and the `dig_H4`/`dig_H5` extraction now uses `uint16_t`
with the register-packing rule stated in a comment.
`src/BME280.cpp:277`, `src/BME280.cpp:349`.

---

## 1. Skipped-channel sentinels reject valid readings — data loss in the field

**Severity: high. This one silently throws away good samples.**

`_readRawData()` marks a channel invalid when the raw ADC equals the Bosch
"skipped" value:

```cpp
// src/BME280.cpp:2958
candidate.temperatureValid = (_config.osrsT != Oversampling::SKIP) &&
                             (candidate.adcT != cmd::RAW_TEMPERATURE_SKIPPED);
```

and `_compensate()` then fails the **entire** sample:

```cpp
// src/BME280.cpp:2977
if (!raw.temperatureValid) {
  return Status::Error(Err::COMPENSATION_ERROR, "Temperature sample skipped");
}
```

`0x80000` (T/P) and `0x8000` (H) are the *reset and skipped* register values.
The datasheet states this in Tables 18/20/23/24 — and nowhere states that they
cannot occur as genuine ADC output. They are ordinary mid-scale codes.

Concretely, for temperature: with typical trim (`dig_T1 ≈ 27504`,
`dig_T2 ≈ 26435`), `adc_T = 0x80000` compensates to roughly **26.5 °C** — the
middle of ordinary indoor room temperature. The gradient is ~32 ADC counts per
0.01 °C, and the datasheet quotes 0.005 °C RMS noise at the lowest
oversampling, i.e. σ ≈ 16 counts. At `osrs_t = x1` the low 4 bits are zero, so
the quantisation step is 16 counts — about one σ. Whenever the ambient sits
near that code, a large fraction of samples read exactly `0x80000` and the
driver discards the whole reading with `COMPENSATION_ERROR`. Humidity has the
same problem at `adc_H = 0x8000`, which is squarely inside the normal humidity
ADC range.

So the current behaviour is: *at some ordinary room temperatures and
humidities, this driver intermittently drops measurements.*

The sentinel cannot distinguish "channel was skipped" from "the ADC legitimately
read this value". Only the configuration knows that, and the driver already
knows it.

**Proposal.** Derive validity from configuration alone, and delete the sentinel
comparison from the validity decision:

```cpp
candidate.pressureValid    = (_config.osrsP != Oversampling::SKIP);
candidate.temperatureValid = (_config.osrsT != Oversampling::SKIP);
candidate.humidityValid    = (_config.osrsH != Oversampling::SKIP);
```

`validateSettings()` already guarantees temperature is enabled whenever
pressure or humidity is, so `_compensate()`'s `!raw.temperatureValid` guard
becomes unreachable for accepted settings and can be simplified to an
assertion-style early return.

This is not a loss of safety. The condition the sentinel was reaching for —
"the hardware is not configured the way the cache thinks" — is already covered,
properly and unambiguously, by `ConfigSyncState` / `hardwareConfigDirty()`. If
stronger detection is wanted, read back `ctrl_meas`/`ctrl_hum` after apply
(see finding 7); do not infer it from measurement data.

Keep `cmd::RAW_*_SKIPPED` in `CommandTable.h` as documentation of the reset
values.

**Test impact.** `test_enabled_raw_sentinel_rejects_compensated_sample`,
`test_skipped_sentinels_are_explicit_validity_flags` and
`test_raw_sentinel_failure_preserves_committed_sample_envelope` encode the
current behaviour and must be rewritten to assert the new contract: a
configured channel is always valid, a `SKIP`ped channel never is.

---

## 2. Multi-byte `writeRegisters()` produces an invalid I2C transaction

**Severity: high. Public API that cannot work on real silicon.**

```cpp
// src/BME280.cpp:2531
uint8_t payload[MAX_WRITE_LEN + 1] = {};
payload[0] = startReg;
std::memcpy(&payload[1], buf, len);
return _i2cWriteTracked(payload, len + 1);
```

This assumes the register pointer auto-increments on writes. The datasheet is
explicit that it does not:

> §6: "multiple byte write (**using pairs of register addresses and register
> data**)" — and Figure 9 is captioned "I2C multiple byte write (**not
> auto-incremented**)".

Reads *do* auto-increment; writes do not. So `writeRegisters(0xF2, buf, 4)`
sends `F2 b0 b1 b2 b3`, and the device writes `b0` to `0xF2` and then
interprets `b1` as the next register address, `b2` as its data, and so on —
scribbling over registers the caller never named.

The internal driver never hits this (every internal `writeRegs()` call passes
`len == 1`), which is why it has gone unnoticed. But `writeRegisters()` is
public and documented for blocks up to `MAX_WRITE_LEN`, and
`test/test_basic.cpp:3071` exercises a 4-byte block against a `FakeBus` that
*models the wrong behaviour* — the stub auto-increments, so the test passes
while real hardware would not.

**Proposal.** Emit the datasheet's address/data-pair form, which still fits one
transaction:

```cpp
Status BME280::writeRegs(uint8_t startReg, const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM);
  }
  if (len > MAX_WRITE_LEN) {
    return Status::Error(Err::INVALID_PARAM);
  }
  // The BME280 does not auto-increment the register pointer on writes
  // (datasheet section 6): a burst write is a sequence of address/data pairs.
  uint8_t payload[MAX_WRITE_LEN * 2] = {};
  for (size_t i = 0; i < len; ++i) {
    payload[2 * i]     = static_cast<uint8_t>(startReg + i);
    payload[2 * i + 1] = buf[i];
  }
  return _i2cWriteTracked(payload, len * 2);
}
```

Single-register writes are unchanged (`[reg, value]`), so every existing
internal call site and every existing single-byte test keeps its exact byte
sequence.

**Also fix the test double.** `FakeBus` in `test/test_basic.cpp` must decode
writes as address/data pairs; otherwise the suite keeps certifying a protocol
the chip does not implement.

---

## 3. NVM-readiness polling misreads `im_update` outside reset

**Severity: medium-high. Spurious `BUSY`/`TIMEOUT` from `recover()` and resync.**

`_waitForNvmReady()` and the staged `NVM_POLL` phase treat `status.im_update == 1`
as "NVM copy after reset still in progress". Per the datasheet (§5.4.4,
Table 21) that bit means more than that:

> "Automatically set to '1' when the NVM data are being copied to image
> registers … **The data are copied at power-on-reset and before every
> conversion.**"

For `begin()` and `softReset()` the device is in sleep and the reading is
sound. But `recover()` (`src/BME280.cpp:701`) and `startResyncJob()` /
`JobPhase::RESYNC_NVM_START` do **not** reset the device — a device running in
`NORMAL` mode is cycling, so `im_update` pulses before every conversion. A
resync that lands on one of those pulses gets `BUSY` (sync path) or burns poll
budget and can hit `TIMEOUT` (staged path), reporting a device fault where
there is none. With a short standby the pulse rate is high and this is not rare.

**Proposal.** Make the check reflect what each caller actually needs.

- Reset-driven paths (`begin()`, `softReset()`, `SOFT_RESET_WRITE →
  RESYNC_READ_CHIP_ID → RESYNC_NVM_START`) legitimately wait for `im_update`;
  leave them as they are.
- Non-reset resync (`recover()`, `startResyncJob()`) should not gate on
  `im_update` at all. Calibration image registers are readable regardless, and
  the resync already re-reads and validates the coefficients — that validation
  is the real check. Skip the NVM phase and go straight to `CALIB_TP`.

Concretely: give `_waitForNvmReady()` the reset context it is really asking
about, and route `JobPhase::RESYNC_NVM_START` to `CALIB_TP` directly when
`_jobKind == JobKind::RESYNC`, keeping it only for `JobKind::SOFT_RESET`.

This deletes a phase transition rather than adding one, and removes a whole
class of false failures.

---

## 4. `_applyConfig()` fails `begin()` with `BUSY` on a device that is merely measuring

**Severity: medium. Init flakes against an already-running sensor.**

```cpp
// src/BME280.cpp:2699
Status st = _ensureConfigWriteReady();   // BUSY if status.measuring
if (!st.ok()) {
  return st;
}
st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeasSleep, 1);
```

The *second* readiness check — after the sleep write, before the `config` write
— is genuinely required, because §5.4.6 says normal-mode `config` writes may be
ignored, and §3.3.1 says a queued mode change causes subsequent `ctrl_hum`
writes to be dropped.

The *first* one is not. Writing `ctrl_meas` to sleep is legal at any time; the
datasheet says the transition is simply deferred to the end of the running
measurement. All the first check accomplishes is turning "the sensor was left
in normal mode by a previous session, a warm reboot, or another owner" into a
hard `begin()` failure — with probability roughly
`t_measure / (t_measure + t_standby)` per attempt, e.g. ~8 % at 125 ms standby.

The staged `INIT` path already gets this right: it writes sleep
unconditionally, then waits in `APPLY_WAIT_AFTER_SLEEP`. The synchronous path
should not be stricter than the staged one.

**Proposal.** Delete the first `_ensureConfigWriteReady()` in `_applyConfig()`.
Keep the second. Same change in `setFilter()` (`src/BME280.cpp:2172`) and
`setStandby()` (`src/BME280.cpp:2237`), which repeat the identical pattern.

---

## 5. `setOversamplingH()` can silently lose the `ctrl_hum` write

**Severity: medium.**

```cpp
// src/BME280.cpp:2124
const uint8_t ctrlHum = buildCtrlHum(osrs);
Status st = writeRegs(cmd::REG_CTRL_HUM, &ctrlHum, 1);
...
const uint8_t ctrlMeas = buildCtrlMeas(_config.osrsT, _config.osrsP,
                                       registerModeForConfig(_config.mode));
st = writeRegs(cmd::REG_CTRL_MEAS, &ctrlMeas, 1);
```

The `ctrl_hum` → `ctrl_meas` ordering is right, but the datasheet adds a
condition the code does not honour (§3.3.1):

> "If the device is currently performing a measurement, execution of mode
> switching commands is delayed until the end of the currently running
> measurement period. Further mode change commands **or other write commands to
> the register `ctrl_hum` are ignored** until the mode change command has been
> executed."

In `NORMAL` mode the device is periodically measuring, so `setOversamplingH()`
can have its `ctrl_hum` write dropped, then write `ctrl_meas`, then cache the
new `osrsH` and report `Status::Ok()`. The driver now believes humidity
oversampling changed when it did not, and nothing marks the config dirty.

`setOversamplingT()` and `setOversamplingP()` have the milder version of the
same gap: they write `ctrl_meas` in place, without the sleep-first sequence
that `setFilter()`/`setStandby()` use.

**Proposal.** Route all five setters through one shared sequence rather than
five hand-written variants. See finding 6 — this is the same refactor.

---

## 6. The same config-write sequence is implemented four times

**Severity: medium (maintainability). This is why findings 4 and 5 diverged.**

The "quiesce → sleep → confirm → write → restore mode" sequence exists in four
places with four slightly different behaviours:

| Location | Sleep first? | Waits for idle? | Restores mode on failure? |
|---|---|---|---|
| `_applyConfig()` `src/BME280.cpp:2688` | yes | one-shot `BUSY` | partially |
| `pollJob()` `APPLY_*` phases `src/BME280.cpp:1461` | yes | polls with deadline | via `_failJob()` |
| `setFilter()` `src/BME280.cpp:2161` | yes | one-shot `BUSY` | best-effort |
| `setStandby()` `src/BME280.cpp:2226` | yes | one-shot `BUSY` | best-effort |
| `setMode()`/`setOversamplingT/P/H()` | **no** | **no** | n/a |

`setFilter()` and `setStandby()` are byte-for-byte identical apart from which
argument reaches `buildConfig()` — about 45 duplicated lines.

**Proposal.** One private helper that owns the whole sequence, and make every
synchronous setter a thin wrapper:

```cpp
// Applies a complete desired settings tuple to the device using the
// datasheet-mandated order, leaving the device in the cached mode.
Status BME280::_writeSettingsToDevice(const SensorSettings& s);
```

Then:

```cpp
Status BME280::setFilter(Filter filter) {
  SensorSettings s = sensorSettings();
  s.filter = filter;
  return _applySettingsSynchronously(s);   // validates, writes, commits cache
}
```

`setStandby`, `setMode`, `setOversamplingT/P/H` collapse the same way. This
removes roughly 200 lines, makes finding 5 impossible to reintroduce (every
setter gets the sleep-first sequence for free), and leaves exactly two
implementations of the register order — one synchronous, one staged — instead
of four.

Going further and expressing `_applyConfig()` as a synchronous drive of the
same phase machine used by `pollJob()` would reduce it to one, but that is a
larger change; the helper above captures most of the benefit.

---

## 7. Cached configuration is never verified against the device

**Severity: medium (design gap).**

`hardwareConfigDirty()` is inferred purely from write outcomes. Everything the
driver believes about `ctrl_meas`, `ctrl_hum` and `config` rests on "the write
returned OK, therefore the register holds what I sent". The datasheet supplies
at least two ways that can be false without any transport error: normal-mode
`config` writes "may be ignored" (§5.4.6), and `ctrl_hum` writes are dropped
while a mode change is queued (§3.3.1) — finding 5 is exactly this.

**Proposal.** Add a read-back verification step at the end of a successful
apply — one extra `readRegs(0xF2, buf, 4)` covering `ctrl_hum`, `status`,
`ctrl_meas`, `config` — and compare against the intended images (masking the
mode field, which the chip clears on its own after a forced conversion). On
mismatch, set `hardwareConfigDirty()` with a distinct detail instead of
reporting success.

Cost: one transaction per apply. Expose it as a new terminal `JobPhase`
(`APPLY_VERIFY`) in the staged path and a final step in `_applyConfig()`. This
turns the driver's central correctness assumption into something it actually
checks, and it is the honest way to get the guarantee finding 1's sentinel was
gesturing at.

---

## 8. Normal-mode freshness budget ignores the standby-time tolerance

**Severity: low-medium.**

```cpp
// src/BME280.cpp:176
if (settings.mode == Mode::NORMAL) {
  return (2U * measurementMs) + standbyTimeMs(settings.standby);
}
```

The reasoning is right — worst case is the tail of the running conversion, one
standby, and a full next conversion. But `t_standby` is a nominal value. Table 1
gives `Δt_standby` as **typ ±5 %, max ±25 %**. At `MS_1000` the real standby can
be 1250 ms, so the computed interval is up to 250 ms short and the "fresh
relative to the request" guarantee does not hold: `tick()` can find
`measuring == 0` during a longer-than-nominal standby and return the *previous*
cycle's sample as fresh.

**Proposal.** Apply the datasheet tolerance where the freshness claim is made:

```cpp
// Datasheet Table 1: t_standby accuracy is +/-25% maximum.
static uint32_t standbyTimeMaxMs(Standby standby) {
  const uint32_t nominal = standbyTimeMs(standby);
  return nominal + (nominal + 3U) / 4U;   // +25%, rounded up
}
```

and use it in `measurementReadinessIntervalMs()`. Leave the public
`getStandbyTimeMs()` reporting the nominal value, which is what callers expect.

---

## 9. 102 diagnostic messages are constructed and thrown away

**Severity: low (readability), but it makes the code actively misleading.**

`Status` deliberately owns its message storage:

```cpp
// include/BME280/Status.h:81
constexpr Status(Err c, int32_t d, const char* m)
    : code(c), detail(d), msg(toString(c)) {
  (void)m;
}
```

That is a sound decision — no borrowed pointers. But `src/BME280.cpp` still
passes 102 descriptive literals into it. Every one is discarded. Reading the
source, `Status::Error(Err::TIMEOUT, "NVM ready polling limit reached", …)` and
`Status::Error(Err::TIMEOUT, "Measurement idle wait timeout")` look like
distinguishable diagnostics; at runtime both are simply `TIMEOUT`.

**Proposal.** Delete the ignored parameter from the call sites and the
`Status::Error(Err, const char*, int32_t)` overload, keeping
`Status::Error(Err, int32_t detail)`. Where the discarded text carried real
information, promote it:

- to the `detail` field, where the value is genuinely useful (`chipId`,
  `nvmReadyTimeoutMs`, `NVM_READY_MAX_POLLS`, `BusyReason`) — several call
  sites already do this;
- to a `//` comment on the return, where it only explained intent to a reader.

Mechanical, ~102 single-line edits, no behaviour change, and it removes a
standing invitation to believe the driver reports more than it does.

Note the overload also has a latent hazard: `Status::Error(Err::X, 0)` is
ambiguous between the `int32_t` and `const char*` overloads, since `0` is a null
pointer constant. Removing the message overload removes that too.

---

## 10. Compatibility aliases with nothing to be compatible with

**Severity: low (API surface).**

The public headers carry duplicate spellings introduced "for cross-library
uniformity" or "for older repositories". Nothing in this repository or its
history uses the older name:

| Alias | Canonical | Used anywhere? |
|---|---|---|
| `Err::CONVERSION_NOT_READY` `Status.h:21` | `MEASUREMENT_NOT_READY` | no |
| `JobKind::RECOVERY` `BME280.h:48` | `RESYNC` | no |
| `BME280::startRecoveryJob()` `BME280.h:555` | `startResyncJob()` | tests/docs only |
| `BME280::driverState()` `BME280.h:668` | `state()` | no |
| `JobPollResult::instructionsUsed` `BME280.h:235` | `callbacksUsed` | tests/docs only |
| `cmd::REG_DIG_H5_LSB` `CommandTable.h:89` | `REG_DIG_H5_MSB` | no — and the name is *wrong*: `0xE6` holds `dig_H5[11:4]`, the high bits |
| `VERSION_INT` `Version.h:70` | `VERSION_CODE` | no |

`REG_DIG_H5_LSB` is the one that can actively mislead: it names `0xE6` "LSB"
when the datasheet assigns it `dig_H5[11:4]`.

**Proposal.** Delete all seven at the next major version; delete
`REG_DIG_H5_LSB` now, since an incorrect name is worse than no name. Keep the
`maxInstructions` parameter name of `pollJob()` (it is in the documented
signature) but drop the duplicate `instructionsUsed` result field.

---

## 11. `SettingsSnapshot` carries the same sample data twice

**Severity: low (maintainability).**

```cpp
// include/BME280/BME280.h:404-408
uint32_t sampleTimestampMs;  int32_t tFine;
RawSample rawSample;         CompensatedSample compSample;
SampleEnvelope sample;       // <- contains all five of the above again
```

`getSettings()` fills both copies (`src/BME280.cpp:826-833`), so the struct is
larger than it needs to be on a device where it is passed around by value, and
two fields can in principle disagree after a future edit.

**Proposal.** Keep `SampleEnvelope sample` and drop the five flattened
duplicates, along with `sampleSequence` / `sampleConfigGeneration`, which
`SampleEnvelope` also carries. Breaking change; schedule with finding 10.

---

## 12. `_updateHealth()` reports `READY` while the device is unusable

**Severity: low, but it is a real integration trap.**

```cpp
// src/BME280.cpp:2607
if (st.ok()) {
  ...
  _driverState = DriverState::READY;
  return st;
}
```

Any successful tracked transfer promotes the driver to `READY` and hence
`isOnline() == true`, regardless of `_configSyncState` and `_calibrationState`.
So after a failed config apply, an application polling `isOnline()` sees a
healthy sensor while every `requestMeasurement()` returns `RESYNC_REQUIRED`.

The header does call this "an observational health classification", and
separating transport health from device-state validity is a reasonable design.
The trap is that `isOnline()` *reads* like the question an integrator wants
answered.

**Proposal.** Do not change the state machine. Add one predicate that answers
the question people actually ask, and point `isOnline()`'s documentation at it:

```cpp
/// True when the driver is both transport-healthy and synchronized with the
/// device, i.e. a measurement can be requested right now.
bool canMeasure() const {
  return _initialized &&
         _configSyncState == ConfigSyncState::SYNCHRONIZED &&
         _calibrationState == CalibrationState::VALID &&
         _driverState != DriverState::UNINIT;
}
```

---

## 13. Minor items

- **`_waitForNvmReady()`'s `TIMEOUT` branch is unreachable.**
  `src/BME280.cpp:2758` computes `deadline = _nowMs() + nvmReadyTimeoutMs`, then
  a few microseconds later tests `deadlineReached(_nowMs(), deadline)`, which is
  false for any non-zero timeout. The function always returns `OK` or `BUSY`.
  The header (`BME280.h:474`) and README promise "`BUSY` or `TIMEOUT`". Either
  delete the dead branch and the local deadline and document `BUSY` only, or —
  if a real timeout is wanted — persist the deadline across calls. Deleting is
  simpler and matches the "no hidden polling loop" contract.

- **`Status::detail` lost on settings validation.** `_prepareBeginConfig()`
  (`src/BME280.cpp:544`) returns `Status::Error(Err::INVALID_CONFIG,
  settingsStatus.detail)`, but `validateSettings()` always yields `detail == 0`,
  so the caller cannot tell which field was rejected. Give `validateSettings()`
  a small reason code in `detail` (`OSRS_T`, `OSRS_P`, `OSRS_H`, `FILTER`,
  `STANDBY`, `MODE`, `SELECTION`) and propagate it.

- **IIR filter memory is not documented as surviving a skipped channel.**
  §3.4.4: a skipped T or P measurement leaves the filter memory unchanged, so
  the first sample after re-enabling is filtered against pre-skip history. The
  fix is a doc note on `setOversamplingT/P` plus a sentence in
  `docs/BME280_Register_Reference.md`; `setFilter()` already resets the
  hardware filter, which is the escape hatch the datasheet recommends.

- **`docs/BME280_Register_Reference.md` §4.4 overstates the config rule.** It
  says `config` is "writable only in sleep mode"; the datasheet says normal-mode
  writes "**may** be ignored". The table row on `0xF5` in the same document
  already phrases it correctly. Align §4.4 with the table, and mark the
  "switch to sleep → write → restore" sequence as driver practice rather than a
  datasheet-prescribed procedure.

- **`docs/BME280_Register_Reference.md` §8 vs. datasheet Table 18.** The doc
  lists `0xE8..0xF1` as reserved; Table 18 labels `0xE1..0xF0` as
  `calib26..calib41`, while §4.2.2 names only `0xE1..0xE7`. This is an internal
  datasheet contradiction. The doc's stricter reading is harmless — the driver
  reads only `0xE1..0xE7` — but the conflict should be noted rather than
  presented as settled.

---

## Suggested order of work

1. Finding 2 (`writeRegisters` protocol) and finding 1 (sentinels) — these are
   the two that produce wrong behaviour on real hardware. Both need matching
   test updates, including the `FakeBus` write decoding.
2. Findings 3, 4, 5 — datasheet-conformance fixes; all three are small and all
   three remove failure modes rather than adding code.
3. Finding 6 — the setter refactor. Do it after 4 and 5 so the corrected
   sequence is what gets consolidated.
4. Finding 7 — read-back verification, once 6 has given it a single place to
   live.
5. Findings 9, 8, 13 — mechanical cleanups, safe at any point.
6. Findings 10, 11, 12 — API surface; batch into the next major version.

---

# Part 2 — Examples, tooling, and packaging

Same rules: anything I could verify and fix safely is already applied and
listed first; everything else has a proposal.

## Already fixed in this pass

**B. `check_package_contents.py` could not detect a `.pio` or `.git` leak.**
`normalize()` used `name.lstrip("./")`, which strips a *character set*, not a
prefix: `.pio/build/x` became `pio/build/x`, so `FORBIDDEN_PARTS` could never
match a root-level dot-entry. Latent rather than active — real archive members
carry a `BME280/` prefix, so `lstrip` was a no-op on them — but the guard was
one packaging change away from silently passing. Now uses `removeprefix("./")`.
`tools/check_package_contents.py:96`.

**C. The package checker did not require the Arduino example's own headers.**
`examples/01_basic_bringup_cli/main.cpp` was required but none of the seven
`examples/common/*.h` it includes were, so a packaging regression that dropped
`examples/common/` would ship an example that cannot compile — and CI would
print `Package contents PASSED`. (`idf_example_required_paths()` already does
this dependency walk for the IDF example.) All seven added; `pio pkg pack`
followed by the checker passes, confirming they do ship today.
`tools/check_package_contents.py:27`.

**D. `check_core_timing_guard.py` was mostly vestigial.** `ALLOWED_CALL_COUNTS`
and `ALLOWED_INCLUDE_COUNTS` were empty dicts, so four of the file's loops
iterated over nothing and the effective rule was simply "zero forbidden calls
or includes". Rewritten to state that rule directly (122 to 84 lines), and while
the mechanism was being removed the forbidden set gained the calls it should
always have covered: `delay`, `vTaskDelay`, `esp_timer_get_time`. Still passes.

**E. `check_cli_contract.py` carried a 42-line duplicate command list and two
tombstones.** `HANDLED_COMMANDS` was `MANDATORY_COMMANDS` minus `"help"`, and
the loop over `MANDATORY_COMMANDS` only did a `\b<cmd>\b` word search, which
matches help text and comments and therefore could not detect a removed
command. There was also an unreachable `cfg`/`settings` check (both are already
required) and `ensure_missing()` assertions for `examples/00_smoke_boot` and
`examples/03_feature_walkthrough`, deleted several releases ago. Collapsed to
one `COMMANDS` list checked with the real handler-pattern regex — which now
also covers `help`, previously only word-matched (359 to 297 lines).

**F. `reconnect_serial_in_place()` closed the port even with a zero reconnect
budget.** `--reconnect-attempts` defaults to `0`, and the function did
`ser.close()` unconditionally *before* the retry loop. With `max_attempts == 0`
the loop body never ran, so it returned "not recovered" with the port closed.
`run_live_plan()` — whose docstring is "always perform required final cleanup"
— then ran `run_final_cleanup()` on a dead handle, turning all five cleanup
rows (`normal off`, `recover`, `cfg`, `status`, `drv`) into
`FINAL_CLEANUP_EXCEPTION`. One serial hiccup under default flags left the
sensor in whatever state the soak abandoned and made the mandatory safe-state
evidence unobtainable. Now returns immediately, leaving the handle untouched,
when there is no budget to spend. `tools/run_i2c_hil.py:3626`.

**G. The Arduino example CLI did not come up after a failed bring-up.**
`setup()` returned early on I2C-init or `begin()` failure, so `printHelp()` and
`cli::printPrompt()` never ran. `loop()` still serviced serial, so the device
was interactive — but with no banner and no prompt, and `scan` / `addr` /
`begin` are exactly the recovery commands an operator needs in that state. The
ESP-IDF example's `app_main()` already got this right, so this was also a
parity break between the two "parity" examples. Restructured to always reach
the banner; success-path output is byte-identical.
`examples/01_basic_bringup_cli/main.cpp:2618`.

---

## 14. Classification decisions run on a truncated copy of the output

**Severity: medium. Latent, and it fails silently.**

```python
# tools/run_i2c_hil.py:2920
"output_excerpt": clean_output[-1000:],
```

`row_output()` (`:2336`) returns that excerpt, and three post-hoc
reclassifiers read it: `row_proves_recover_ok()` (`:2577`),
`row_proves_readable_config()`, `row_proves_ready_clean_status()`.

So verdicts are computed from the **last 1000 characters** of a command's
output. Against the retained transcript, `recover` currently produces roughly
619–794 characters, so it fits — but `selftest` is about 1306, `stress_mix 70`
about 1300 and `stress 50` about 1029. Those rows are already losing their head
in `results.csv`, `summary.json` and `manifest.json`. One extra health-diff line
in `recover`'s output and `reclassify_recovered_reset_busy()` starts silently
failing to find `Status: OK`, downgrading a genuine pass.

**Proposal.** Separate the evidence from the display. Keep `output_excerpt` as
the human-facing tail, and have the classifiers read either the full output or
the already-structured `parsed_evidence`:

```python
def row_output(row: dict) -> str:
    # Evidence, not display: never the truncated excerpt.
    return strip_ansi(str(row.get("output_full", row.get("output_excerpt", ""))))
```

storing `output_full` on the row and omitting it from the CSV writer. This
keeps artifact sizes unchanged while making classification independent of the
excerpt limit.

---

## 15. `job_command_budget()` misattributes budgets for non-numeric job verbs

**Severity: low-medium.**

```python
# tools/run_i2c_hil.py:1875
if len(parts) >= 3 and parts[0].lower() == "job":
    try:
        return int(parts[2], 10)
    except ValueError:
        return JOB_CLI_DEFAULT_BUDGET
```

`job start init` and `job cancel owner` have a non-numeric `parts[2]`, so both
fall through to `JOB_CLI_DEFAULT_BUDGET = 1`. But `job start ...` consumes zero
transport callbacks (the start is admission-only — `startInitJob()` performs no
I2C) and `job cancel ...` is documented as zero-I2C. Budgeting them at 1
inflates the expected callback accounting.

**Proposal.** Key off the verb rather than blindly on `parts[2]`:

```python
JOB_ZERO_BUDGET_VERBS = {"start", "cancel", "status"}

def job_command_budget(command: str) -> int:
    parts = command.strip().split()
    if not parts or parts[0].lower() != "job":
        return JOB_CLI_DEFAULT_BUDGET
    verb = parts[1].lower() if len(parts) >= 2 else ""
    if verb in JOB_ZERO_BUDGET_VERBS:
        return 0
    if len(parts) >= 3:
        try:
            return int(parts[2], 10)
        except ValueError:
            pass
    return JOB_CLI_DEFAULT_BUDGET
```

---

## 16. `output_has_expected()` does not check `spec.expected`

**Severity: low-medium (correctness of recorded evidence).**

```python
# tools/run_i2c_hil.py:2623
def output_has_expected(spec: CommandSpec, output: str) -> bool:
    return completion_tokens_match(spec, output)
```

It is the read-loop exit predicate (`:2858`), and its result is stored in a
variable named `matched_expected` and recorded as
`completion="MATCHED_EXPECTED"`. Both can be true when the *expected* tokens
have not matched — only `classify_output()` consults the real
`expected_tokens_match()`. The artifacts therefore claim something the function
never checked.

Related: the idle-exit branch at `:2862` reads
`if saw_output and not spec.expected and now - last_rx >= idle_after_output_s`,
which ignores `spec.expected_any` and `spec.completion`. A spec carrying only
`completion=` tokens exits after `idle_after_output_s` (0.75 s default) with
`SERIAL_IDLE_NO_EXPECTED_TOKENS` before its completion token can arrive.

**Proposal.** Rename to `output_is_complete()` and rename the call-site variable
and the recorded completion string to match what is measured. Change the idle
guard to `not (spec.expected or spec.expected_any or spec.completion)`.

---

## 17. `final_verdict()` ignores `RESULT_SKIPPED_UNSAFE`

**Severity: low-medium.**

`tools/run_i2c_hil.py:4163` sorts results into a FAIL bucket and a review
bucket; `RESULT_SKIPPED_UNSAFE` is in neither. A live run in which a gated
command was skipped as unsafe still reports `PASS`, so the artifact overstates
coverage.

**Proposal.** Put `RESULT_SKIPPED_UNSAFE` in the review bucket, so the verdict
becomes `OPERATOR_REVIEW_REQUIRED` and the reason names the skipped commands.
That matches the repository's own rule that unrun checks must be recorded as
`NOT RUN`.

---

## 18. The ESP-IDF component name depends on the checkout directory name

**Severity: medium for consumers. Needs an `idf.py` run to confirm.**

`examples/idf/basic/CMakeLists.txt:3` sets `EXTRA_COMPONENT_DIRS "../../../"`
— the repository root — and the root `CMakeLists.txt` is itself a component
(`idf_component_register`). ESP-IDF names such a component after its directory
basename. `examples/idf/basic/main/CMakeLists.txt:4` then declares
`REQUIRES BME280`.

That only resolves when the repository directory happens to be called exactly
`BME280`. Clone into `bme280-driver`, or extract `BME280-2.1.0.tar.gz` into its
versioned folder, and the build fails with "component BME280 not found". CI
does not catch it because `espressif/esp-idf-ci-action` checks out into a
directory named after the repository.

**Proposal.** Derive the name instead of hard-coding it, in
`examples/idf/basic/main/CMakeLists.txt`:

```cmake
get_filename_component(BME280_COMPONENT_NAME "${CMAKE_CURRENT_LIST_DIR}/../../../.." NAME)
idf_component_register(
  SRCS "main.cpp" "IdfI2cTransport.cpp"
  INCLUDE_DIRS "." "../../../.."
  REQUIRES ${BME280_COMPONENT_NAME} esp_driver_i2c esp_driver_gpio esp_timer freertos
)
```

Verify with an actual build from a renamed directory, and add that rename to
`tools/check_idf_example_contract.py` as a regression guard.

---

## 19. Native test stubs do not model the hardware they stand in for

**Severity: medium. The suite currently certifies protocols the chip does not use.**

Two separate problems:

1. **`FakeBus` auto-increments register writes.** As established in finding 2,
   the BME280 does not. So
   `test_diagnostic_config_block_write_marks_dirty_when_range_overlaps_config`
   (`test/test_basic.cpp:3071`) passes against a model of a device that does not
   exist. Fixing finding 2 requires fixing the stub in the same change,
   otherwise the corrected driver fails the incorrect test.
2. **TX buffer size mismatch.** `test/stubs/Wire.h:21` gives the stub a 64-byte
   TX capacity, while `examples/common/I2cTransport.h:103,157` validates
   against 128 (the real ESP32 `Wire` buffer). Native tests therefore exercise
   the short-write branch at a threshold that cannot occur on hardware, and the
   128-byte guard that *can* trigger is never exercised.

**Proposal.** Make the stub's buffer size a single named constant shared with
the transport's guard, set to 128, and decode writes as address/data pairs.
A stub that lies about the bus is worse than no stub, because it converts a
protocol bug into a green test.

---

## 20. `run_i2c_hil.py` is six modules in one file (4,884 lines)

**Severity: low (no defect), high (maintenance cost).**

| Lines | approx. | Responsibility |
|---|---|---|
| 30–182 | 150 | Constants, ~40 module-level regexes, dataclasses |
| **184–1712** | **1,530 (31 %)** | **Inert command-catalogue data** |
| 1715–2010 | 300 | argparse validators, custom-command loading, plan assembly |
| 2025–2620 | 600 | Output parsing, evidence extraction, classification |
| 2634–3160 | 530 | pyserial I/O, plus a 175-line embedded self-test |
| 3162–4060 | 900 | Duration-soak scheduler and reconnect |
| 4062–4600 | 540 | Nine artifact writers |
| 4606–4884 | 280 | 48-flag `parse_args` and `main` |

**Proposal**, in decreasing value per unit of risk:

1. Move the 1,530-line command catalogue to `tools/hil_plan.py` (or a data
   file). It is inert data; `build_command_sequence()` is 100 lines of
   gate-and-extend on top of it. Plan changes then review as data diffs.
2. Move the nine artifact writers to `tools/hil_report.py`. They depend only on
   the `summary` dict and `results` list.
3. Move the parsers/classifiers to `tools/hil_parse.py`. This is what
   `test_run_i2c_hil_parser.py` actually tests; today that test file must
   import the whole 4,884-line module to reach it.
4. Delete `parser_self_test()` (`:2988`, ~175 lines). It is a weaker second
   copy of `tools/test_run_i2c_hil_parser.py`'s 82 tests, and CI runs both.
5. Delete `duration_command_fits()` (`:3329`). Superseded by group-level
   budgeting; its only callers are `check_hil_contract.py:285` and two
   assertions in the test file — a contract test for code that never runs.

**Do this before, not after, the other tooling work.**
`check_hil_contract.py:412` asserts that about 20 literal substrings appear in
`run_i2c_hil.py`'s *source text*, and `check_cli_contract.py` /
`check_idf_example_contract.py` do the same to the two `main.cpp` files (down to
`cmd.replace("  ", " ")` and `JOB_CLI_MAX_POLLS = 1024U`). Any refactor will
trip these for reasons unrelated to behaviour, so the checkers have to be
rewritten alongside — which is itself the strongest argument for doing the split
deliberately rather than discovering the coupling later.

---

## 21. Minor items (examples and tooling)

- **Tombstone assertions.** `tools/check_hil_contract.py:31` still lists four
  `REMOVED_HIL_DOCS` deleted several releases ago, asserts their absence, and
  greps them out of the maintained docs (`:477`). Delete once no maintained doc
  could plausibly reintroduce the names.

- **`check_hil_contract.py:297` is tautological.**
  `reconnect_args = runner_args(reconnect_attempts=3)` builds a
  `SimpleNamespace` from the overrides, so `!= 3` can never hold. It tests
  nothing about the runner. Replace with an assertion against the argparse
  default, or delete.

- **`getSettings()` cannot fail but returns `Status`.** The implementation
  (`src/BME280.cpp:804`) unconditionally returns `Status::Ok()` and the header
  documents "`Status::Ok()` always". The four `(void)device.getSettings(...)`
  call sites in the examples are therefore correct, not sloppy — but the
  signature invites the reader to think otherwise. Make it
  `void getSettings(SettingsSnapshot&) const` at the next major and drop the
  casts.

- **`runSelfTest` double-counts one root cause.**
  `examples/01_basic_bringup_cli/main.cpp:1266`: when `captureSensorSettings`
  fails, the same status is reported FAIL at "capture baseline settings" and
  again at "restore baseline settings". Skip the restore check when the capture
  never happened.

- **`parseMode` rejects raw mode bits `2`.** `modeBitsToStr`
  (`main.cpp:172`) correctly decodes both `1` and `2` as `FORCED` per datasheet
  Table 25, but `parseMode` (`:1380`) accepts only `1`. Self-consistent with the
  `(0|1|3)` help text, but a trap for anyone reading a raw `ctrl_meas` dump and
  typing the value back.

- **Empty-line prompt suppression is deliberate, not a bug.** `loop()` only
  prints a prompt when `inputBuffer.length() > 0`, which is what keeps a CRLF
  terminal from emitting two prompts per command. Worth one comment saying so,
  since it reads like an oversight.

- **`HealthView.h:152` colours the driver-state string with
  `failureColor(consecutiveFailures)`,** so a `READY` driver with one recent
  transient failure renders yellow. Cosmetic, but it feeds operator judgement
  during HIL review. Colour the state by state.

- **IDF CLI omits `printStressProgress`.** The Arduino CLI emits
  `Progress: N/M (..., ok=..., fail=...)` during `stress`/`stress_mix`; the IDF
  `tickApp()` does not, so the two parity examples produce different transcripts
  for the same command. `check_idf_example_contract.py` enforces help-text and
  job-field parity but not this. Also `CLI_INPUT_MAX_LEN = 127` (Arduino) vs
  `CLI_LINE_LEN = 160` (IDF); the contract only checks that the string
  `Command too long` exists in both.

- **`I2cTransport.h:176` returns `TransportErr::OK` with a short read count.**
  The comment says this is deliberate — let the core classify it as
  `I2C_SHORT_TRANSFER` rather than invent a NACK cause — and it matches
  `mapTransportResult()`. Correct, but it is the one place the adapter reports
  `OK` for a failed transfer; keep the comment and make sure
  `docs/PRODUCTION_SHARED_BUS_GUIDE.md` says the same.

- **`platformio.ini` noise.** `extends = env` (`:37`, `:51`) is redundant —
  `[env]` is applied to every environment automatically. `[env:native]`
  inherits `debug_tool = esp-prog`, `debug_init_break`, `upload_speed` and the
  `monitor_*` settings, none of which mean anything on a native host.

- **README include-path divergence.** `README.md:70` shows
  `#include "common/I2cTransport.h"` while the example uses
  `#include "examples/common/I2cTransport.h"`. Both resolve only because
  `-Iexamples` *and* the project root are on the include path; a consumer
  copying the README snippet without `-Iexamples` gets a different result. Pick
  one form.

- **`docs/BME280_datasheet.pdf` is 1.6 MB and is exported into every package**
  (`library.json` `export.include`), where it is over 90 % of the tarball.
  Keeping it tracked is right — it is source evidence. Shipping it to every
  consumer is a separate decision worth making explicitly.

- **Two 5 MB serial transcripts are tracked in git** (~10.4 MB total).
  `check_hil_contract.py:26` pins one of them by exact size *and* SHA-256, so
  any re-capture requires editing two constants, and the check proves only that
  the file is untouched — not that it is good evidence. Consider a release asset
  or Git LFS, and tighten `.gitignore`'s `!hil_logs/*/serial_transcript.txt` to
  an explicit allow-list of retained run IDs, so a stray `git add .` in a future
  run directory cannot commit another 5 MB blob.
