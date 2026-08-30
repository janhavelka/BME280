# BME280 Hardware Validation

This is the single maintained hardware-in-the-loop (HIL) procedure, evidence
schema, and result ledger for the library.

Host tests, builds, and serial parsing do not prove sensor accuracy, electrical
margin, wiring, pull-ups, humidity handling, shared-bus behavior, protected
fault recovery, or long-term stability. Record `NOT RUN` or `unknown` when the
corresponding physical evidence is absent.

## Current Validation Status

The flashed expanded-CLI dirty-tree snapshot completed a physical Arduino HIL
campaign on an ESP32-S2 at address `0x76`. Run `i2c_20260804_155442` recorded
3,642.719 seconds (60m42.719s) of accepted active soak with no serial
interruption. Its 7,115 classified rows contained no `FAIL` or `TIMEOUT`; final
cleanup proved sleep mode, clean hardware configuration, idle status bits, and
zero consecutive failures. Its manifest-backed package was verified at
generation time; per the retention policy in `README.md`, only the raw
transcript `../hil_logs/i2c_20260804_155442/serial_transcript.txt` is retained.

A follow-up run `i2c_20260804_171428` re-ran the complete gate on the rebuilt
and reflashed Arduino snapshot: 271 rows, zero `FAIL`, zero `TIMEOUT`, zero
serial exceptions, exact strict-usage/help evidence, and all five final-cleanup
rows passing. Its raw transcript is retained on the same terms.

The firmware reported library version `2.0.0`, source commit
`f87b80aeb97fb6a775d063ad3e97638caf315925`, and a dirty working tree. Therefore
the correct runner verdict is `OPERATOR_REVIEW_REQUIRED`: this is strong
flashed dirty-tree snapshot functional evidence, but not an immutable-source
release claim or formal hardware qualification. Electrical values, calibrated
environmental reference data, physical fault injection, additional hardware
targets, and operator qualification sign-off are outside the release acceptance
scope and are not claimed.

The `2.1.0` driver implementation and public contracts, excluding generated
version metadata, are byte-identical to the clean-source HIL commit `dc5df8e`.
Later release changes affect examples, tests, validation tooling,
documentation, and metadata rather than driver behavior. Native tests inject
every terminal transport error, short transfers, partial configuration
failures, health degradation, and recovery outcomes at the callback boundary.
This software fault-path coverage is distinct from physical electrical fault
injection.

A separate clean-source ESP32-S2/Arduino run against commit
`dc5df8e6735bd21f52dea3e5ae7dcf54ec2b498d` is retained at
`../hil_logs/i2c_20260803_144215/serial_transcript.txt`. The retained set is
exactly three runs: the clean-source campaign, the expanded long campaign, and
its post-correction flashed gate. Superseded and interrupted development
snapshots are not current evidence and are not retained.

| Target or condition | Current status | Evidence boundary |
| --- | --- | --- |
| ESP32-S2, Arduino/PlatformIO, address `0x76` | FUNCTIONAL SERIAL PASS / OPERATOR REVIEW REQUIRED | Clean source commit `dc5df8e`; 61-minute COM10 campaign; raw transcript retained. No calibrated accuracy or complete electrical qualification claim. |
| ESP32-S2, Arduino/PlatformIO, expanded `2.1.0` development CLI | FUNCTIONAL SERIAL PASS / OPERATOR REVIEW REQUIRED | Dirty-tree run `i2c_20260804_155442`: 60m42.719s active soak, 7,115 rows, zero FAIL/TIMEOUT, no reconnect. Exact final flashed CLI follow-up `i2c_20260804_171428`: 271 rows, zero FAIL/TIMEOUT, final safe state proved. Dirty provenance and incomplete physical metadata prevent an immutable-release qualification claim. |

Other MCU/framework combinations, address straps, electrical fault campaigns,
shared-bus systems, logic-analyzer captures, and calibrated environmental
comparisons are not release requirements and are not claimed by this ledger.

### Current Expanded-CLI Run: `i2c_20260804_155442`

- Target: ESP32-S2-Saola-1, Arduino/PlatformIO, COM10 at 115200 baud,
  BME280 address `0x76`, SDA GPIO8, SCL GPIO9, and reported 400 kHz bus speed.
- Provenance: branch `main`, source commit
  `f87b80aeb97fb6a775d063ad3e97638caf315925`, library version `2.0.0`, dirty
  working tree, and firmware build time `2026-08-04 15:48:37`. Host provenance
  did not change during the run.
- Evidence: retained raw transcript at
  `../hil_logs/i2c_20260804_155442/serial_transcript.txt`; the derived package
  was verified at generation time and then discarded per the retention policy.
  The transcript is 5,231,872 bytes and 118,318 newline-delimited lines,
  SHA-256
  `b3a196590b81fed1b7331edef1118a2dbfd4cd95c44d33932110f082712dfd8c`;
  all ten manifest artifact hashes were independently rechecked after
  generation. Its run-time runner hash was checked before the later
  strict-expectation-only runner change; the final-gate runner hash is matched
  by the follow-up gate below.
- Runtime: the duration phase accepted 3,642.719 seconds of uninterrupted
  active soak from 15:56:39 to 16:57:21, completing 1,213 full safe cycles. It
  conservatively stopped with 57.281 seconds remaining because the next
  indivisible mixed group reserved 58 seconds. There were zero reconnects,
  zero reconnect attempts, and zero replay/deadline extension.
- Results: 7,115 classified rows: 4,431 `PASS`, 61
  `PASS_WITH_RESET_BUSY_RECOVERED`, 2,622 environmental/operator-check rows,
  and one dirty-provenance review row. There were zero `FAIL`, zero `TIMEOUT`,
  and zero serial exceptions.
- Duration workload: 60,700 forced samples with zero reported stress errors;
  84,910 mixed safe operations with zero reported failures and verified full
  settings restoration; 173 normal-mode reads; 404 probe/identity checkpoints;
  and 60 scheduled reset/recover checkpoints. The fixed plan also covered all
  legal setting enum values with exact register readback, strict invalid input,
  staged jobs, lifecycle, bounded register blocks, transport callback counts,
  and benchmarks.
- Final cleanup: `normal off`, `recover`, `cfg`, `status`, and `drv` all passed.
  The final device was in sleep mode with `measuring=0`, `im_update=0`, clean
  hardware configuration, and zero consecutive failures.
- Evidence boundary: samples were valid and plausible but were not compared
  against a calibrated reference. Electrical qualification and additional
  target/application campaigns are outside this release scope.

### Post-Correction Flashed Follow-up: `i2c_20260804_171428`

- The post-audit firmware build reported `2026-08-04 17:11:23`, library
  version `2.0.0`, commit `f87b80a`, and dirty worktree provenance.
- The comprehensive fixed gate included the exhaustive setting matrix, strict
  invalid input, benchmarks, staged jobs, lifecycle/register/callback checks,
  and ten normal-mode reads. Results were 247 `PASS`, 22
  `OPERATOR_CHECK_REQUIRED`, one `PASS_WITH_RESET_BUSY_RECOVERED`, and one
  provenance `REVIEW_REQUIRED`: zero `FAIL`, zero `TIMEOUT`, zero serial
  exceptions, and no provenance change.
- The tightened `mode normal extra` row returned exact `Usage: mode` output;
  `help` listed every accepted standby alias with its required `ms` suffix.
  Final `normal off`, `recover`, `cfg`, `status`, and `drv` rows all passed.
- Evidence: retained raw transcript at
  `../hil_logs/i2c_20260804_171428/serial_transcript.txt`, 104,316 bytes and
  2,755 lines, SHA-256
  `f5ffdb707ee4c46fcbf1fd5ef136addc6a8fd306929f7cbf4de62860841ae192`.
  At generation time all ten artifact hashes and the runner-script hash matched
  its manifest; the derived package was then discarded per the retention
  policy.

### Retained ESP32-S2 Run: `i2c_20260803_144215`

- Target: ESP32-S2, Arduino/PlatformIO, COM10 at 115200 baud, BME280 address
  `0x76`, reported SDA GPIO8, SCL GPIO9, and 400 kHz.
- Provenance: branch `main`, clean source commit
  `dc5df8e6735bd21f52dea3e5ae7dcf54ec2b498d`, library version `2.0.0`, and
  firmware build time `2026-08-03 14:39:57`.
- Raw artifact: 5,048,514 bytes, 105,839 lines, SHA-256
  `7b858255bfc5053e18c224f3d14621e18b3db21ed1cfef16d92f727f4ba55415`;
  repository attributes disable text normalization for retained transcripts.
- Runtime: 3,677.9 seconds for the full campaign, including a 3,545.5-second
  duration soak; 6,979 classified result rows contained no `FAIL` or `TIMEOUT`.
- Stress coverage: 58,860 measurement samples with zero reported errors and
  81,340 mixed safe operations with zero reported failures.
- Reset coverage: 117 reset/NVM-busy observations all recovered; the final
  cleanup reported `READY`, online, zero consecutive failures, clean hardware
  configuration, `measuring=0`, and `im_update=0`.
- Follow-up negative check: selecting unstrapped address `0x77` returned
  `I2C_SHORT_TRANSFER`; the Arduino transport could not prove a definite
  address NACK. Restoring `0x76` succeeded and a subsequent measurement was
  valid. This separately observed follow-up has no retained transcript, so it
  is an adapter diagnostic only, not qualification evidence or address-`0x77`
  device validation.
- Evidence boundary: observed values were plausible, but no calibrated
  reference instrument was recorded. Electrical qualification and additional
  target/application campaigns are outside this release scope.

## HIL Procedure

1. Select the exact clean commit to test and record it before flashing. The
   runner compares the firmware-reported library version, commit, and clean
   state with host provenance captured at run start. A mismatch fails the
   version row; dirty source, unavailable provenance, or a repository change
   during the run requires review and cannot qualify exact build provenance.
2. Record the setup fields below. Do not infer wiring or electrical facts from
   a successful I2C exchange.
3. Build and flash the diagnostic CLI that matches the connected target.
4. Run the parser unit suite and a dry run.
5. Run the serial plan with setup metadata and `--include-job-api` for the
   staged v2 API.
6. Review `summary.md`, `results.csv`, `operator_checklist.md`, the transcript,
   and manual reference/fault evidence together.
7. Retain the raw `serial_transcript.txt` and record the provenance, result,
   and evidence boundary in this ledger. The repository ignore policy permits
   selected raw transcripts while ignoring derived runner output.
8. For a formal qualification claim, preserve the complete selected artifact
   directory and its `manifest.json` in durable tracked storage or an immutable
   release asset. Derived summaries may be cleaned after ledger verification
   only when the run is explicitly retained as functional evidence rather than
   a complete qualification package.

Install `pyserial` only for a real serial run:

```bash
python -m pip install pyserial
```

Build and upload the Arduino examples:

```powershell
.\scripts\pio.cmd run -e esp32s3dev
.\scripts\pio.cmd run -e esp32s2dev
.\scripts\pio.cmd run -e esp32s3dev -t upload --upload-port <PORT>
.\scripts\pio.cmd run -e esp32s2dev -t upload --upload-port <PORT>
```

Build or flash the native ESP-IDF example when `idf.py` is installed:

```bash
idf.py -C examples/idf/basic set-target esp32s3
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic set-target esp32s2
idf.py -C examples/idf/basic build
idf.py -C examples/idf/basic -p <PORT> flash monitor
```

Verify the runner before hardware access:

```powershell
python tools/test_run_i2c_hil_parser.py
python tools/run_i2c_hil.py --dry-run --include-job-api `
  --include-config-matrix --include-invalid-inputs --include-benchmarks `
  --include-normal-soak --normal-soak-count 10 --soak-duration-s 3700 `
  --out .pio/hil_dry_runs
```

Example evidence run (PowerShell line continuation shown):

```powershell
python tools/run_i2c_hil.py `
  --port <PORT> --baud 115200 --address 0x76 --out hil_logs `
  --include-job-api --include-config-matrix --include-invalid-inputs `
  --include-benchmarks --include-normal-soak --normal-soak-count 10 `
  --soak-duration-s 3700 --reconnect-attempts 3 --reconnect-delay-s 2 `
  --fail-on-review `
  --operator "<name>" --board "<board>" --mcu-target esp32s2 `
  --framework arduino --build-target esp32s2dev --module "<module>" `
  --vdd "3.3 V" --vddio "3.3 V" --pullups "4.7 kohm" `
  --pullup-location external --sda-pin GPIO8 --scl-pin GPIO9 `
  --bus-speed "400 kHz" --sdo-state GND --csb-state VDDIO `
  --environment-ref "<instrument/readings artifact>" `
  --operator-notes "<fixture and scope>"
```

Use `--address 0x77` only when SDO is tied to VDDIO. `scan` proves only that an
address acknowledged; `chipid` or `reg 0xD0` must record `0x60` to identify a
BME280.

The default plan deliberately includes environmental plausibility rows that
require human review. Consequently, its final verdict is normally
`OPERATOR_REVIEW_REQUIRED` even when every deterministic serial check passes.
`--fail-on-review` is the honest automation gate for that plan. `--require-pass`
is useful only for a custom plan containing no operator-review rows; it is not a
complete production-release gate.

## Default Command Sequence

This guarded sequence must match `tools/run_i2c_hil.py` for address `0x76`.
With `--address 0x77`, only the `addr` command changes.

<!-- HIL_DEFAULT_SEQUENCE_START -->
```text
version
help
scan
addr 0x76
begin
settings values
xfer_reset
settings validate forced x1 x1 x1 off ms_125
xfer_assert 0 0 0
freshness
xfer_assert 0 0 0
addr
xfer_assert 0 0 0
normal
mode
osrs
filter
standby
settings
xfer_reset
verbose 1
verbose 0
xfer_assert 0 0 0
probe
chipid
cfg
calib
calib raw
status
timing
reg 0xD0
read
raw
comp
freshness 60000
data
xfer_reset
dump 0xF7 8
xfer_assert 1 0 1
xfer_reset
rregs 0x88 8
xfer_assert 1 0 1
xfer_reset
dump 0x80 32
xfer_assert 1 0 1
xfer_stats
force
reg 0xF4
status
read
normal on
read
read
normal off
reset
status
recover
cfg
status
force
xfer_reset
invalidate
xfer_assert 0 0 0
recover
freshness
xfer_reset
end
xfer_assert 0 0 0
begin
selftest
stress 10
drv
state
```
<!-- HIL_DEFAULT_SEQUENCE_END -->

Repeated `read` entries are intentional; the CLI has no `read 10` or `read 20`
command. A valid deterministic serial run records at least:

- chip ID `0x60`;
- post-`force` `reg 0xF4` mode bits `[1:0] == 00`;
- post-`force` `status` with `measuring=0`;
- reset/recover status with `im_update=0`, `READY`, and clean config;
- zero-callback settings validation, freshness, invalidation, and `end()`
  evidence plus address/verbosity queries, live chip-versus-cache settings
  queries, and one-callback bounded register-block reads;
- `selftest` with `fail=0`, subject to environmental review;
- `stress` with `Errors: 0`, subject to environmental review;
- final consecutive failure count of zero;
- when `--include-job-api` is used, bounded callback counts, distinct resync
  and reset jobs, zero-callback whole-settings admission, and exactly-once
  cancellation-result retrieval;
- when `--include-config-matrix` is used, every legal enum value has register
  readback and the six typed setter categories have exact callback profiles.

## Optional and Unsafe Coverage

- `--include-normal-soak --normal-soak-count N
  --normal-soak-interval-s S` adds normal-mode repeated reads.
- `--include-soak --soak-count N` adds forced-measurement stress; it is not
  normal-mode soak.
- `--soak-duration-s N` is a planning budget for a bounded duration mix. The
  runner reserves complete timeout windows and groups `normal on` with
  `normal off`, and `reset` with `recover`, before starting those transitions.
  It may stop short of `N`, and small host/serial scheduling overhead can make
  wall time differ; use the recorded exact elapsed time in any claim. A positive
  duration that cannot complete one full safe cycle is a hard incomplete-run
  failure even when some individual groups passed. After
  every live plan, the runner makes a bounded best-effort final
  cleanup in the fixed order `normal off`, `recover`, `cfg`, `status`, `drv`.
  This includes successful fixed/custom plans, duration stops, and fixed-plan
  FAIL/TIMEOUT. Cleanup rows remain
  visible evidence and preserve the root cause. A failed safe-state cleanup also
  prevents an overall PASS; inspect the cleanup rows explicitly.
  A state-changing duration group also performs its declared recovery before a
  hard stop when that command may have taken effect.
- `--reconnect-attempts N` allows up to `N` additional retries for the initial
  serial open and separately supplies `N` total in-place reconnect attempts to
  the duration soak. A successful reopen must report the same library version,
  full version string, build timestamp, Git commit, and worktree state in its
  canonical `version` identity probe; this is reported-field equality, not a
  binary hash.
  The interrupted safe command group is replayed from its first row.
  Wall elapsed time, active elapsed time, reconnect downtime, replay extension,
  and total deadline extension are recorded separately. Every recovered
  interruption remains an explicit operator-review row; a device failure
  already proven by partial output remains a hard failure. Exhausted retries
  remain a hard failure, with completed soak rows preserved together with any
  partial row.
- Failed initial serial-open attempts, their error text, and measured retry/open
  downtime are retained as explicit review rows and summarized even when a later
  attempt succeeds. Serial baud must be a positive integer no greater than
  10000000; invalid values are rejected before artifacts or hardware access.
- `--include-config-matrix` exercises sleep/forced/normal mode, whole-settings
  apply, and every legal public oversampling, filter, and standby enum encoding
  with register readback, then restores the example's forced-mode defaults.
  `--include-invalid-inputs` and `--include-benchmarks` add the other safe
  diagnostic groups.
- `--include-destructive --confirm-raw-write BME280_RAW_WRITE` enables a raw
  control-register write followed by resync evidence. Never include it in an
  unattended default run.
- `--include-fault-tests` marks manual fault checks as requested. The runner
  does not unplug hardware or short bus lines.
- A command file supplied with `--commands <path>` remains operator-owned and
  must end in a known safe state. The runner always prepends its canonical
  provenance `version` evidence row, even when the custom file also requests
  `version`, and always appends bounded final cleanup. Raw `wreg` entries remain
  gated even when separated with tabs or other whitespace.

Fault injection is manual and may damage hardware. Perform it only on a
protected, current-limited fixture. Record the action, expected mapping, actual
status, recovery, and final health/config state.

## Evidence Record

A formal qualifying run must include:

| Category | Required fields |
| --- | --- |
| Provenance | Operator; date/time and timezone; branch; Git commit; worktree state / dirty flag; runner command and arguments; command groups and opt-ins; firmware `version` output; library version; exact pioarduino platform, Arduino core, and ESP-IDF versions where applicable; runner final verdict; sign-off. |
| Target | Framework; build target; MCU board model and target; serial port/baud; BME280 module and visible marking; fixture description. |
| Electrical | VDD; VDDIO; supply/current limit; SDA/SCL pins and speed; pull-up values and location; address; verified SDO and CSB state; reset/interrupt wiring or `N/A`. |
| Environmental | Reference instrument models/calibration state; reference and BME280 readings; tolerances/uncertainty; timestamp; stability/altitude notes; per-channel decision. Use `NOT RUN` when no reference was used. |
| Artifacts | `serial_transcript.txt`, `summary.md`, timestamped summary, `summary.json`, `results.csv`, `command_plan.json`, `environment.txt`, `operator_checklist.md`, `hardware_matrix_fragment.md`, `failure_analysis.md`, and `manifest.json`; optional photo or logic-analyzer paths. |
| Manual review | Wiring/pull-up/strap verification; forced sleep return; normal-mode/soak scope; protected fault scope; environmental plausibility; remaining untested rows; blocking issues. |

A functional run that is not claimed as formal qualification may retain the
raw transcript plus this ledger's provenance, numeric results, and explicit
limitations. Generated summaries are review aids, not substitutes for the raw
serial record. Do not upgrade such a run to a qualification claim after its
complete package has been discarded.

The runner accepts the setup values through `--operator`, `--board`,
`--mcu-target`, `--framework`, `--build-target`, `--module`, `--vdd`,
`--vddio`, `--pullups`, `--pullup-location`, `--sda-pin`, `--scl-pin`,
`--bus-speed`, `--sdo-state`, `--csb-state`, `--environment-ref`, and
`--operator-notes`. Blank values become `unknown`; metadata completeness is an
operator qualification decision, not part of the serial classifier.

## Result Vocabulary

- `PASS`: required deterministic serial tokens were captured for that row.
- `PASS_WITH_RESET_BUSY_RECOVERED`: reset reported NVM busy and immediate
  follow-up evidence proved complete recovery.
- `OPERATOR_CHECK_REQUIRED`: serial output matched, but a human must evaluate
  environmental or bench evidence.
- `REVIEW_REQUIRED`: output was captured but was not deterministically
  classified.
- `SERIAL_OK_OR_REVIEW`: serial output exists but expected tokens are missing.
- `UNKNOWN`: bounded but incomplete hardware state was observed.
- `FAIL` / `TIMEOUT`: a failure token was observed or a command exceeded its
  deadline.
- `SKIPPED_DRY_RUN` / `SKIPPED_UNSAFE`: no physical command was executed.

A runner verdict is an evidence input, not a hardware qualification. A
qualifying result requires the serial artifacts, setup record, relevant manual
checks, and operator sign-off to agree.
