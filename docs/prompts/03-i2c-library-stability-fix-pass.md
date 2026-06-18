# AI Coder Prompt: I2C Library Stability Fix Pass

You are working inside one I2C library repository. Do a diligent stability and
correctness pass through the current library implementation, then make small,
concrete fixes.

Goal:
Find bugs, functional vulnerabilities, system-stability risks, inconsistent
patterns, and unclear ownership/state behavior. Propose simple, concrete fixes
and implement them one by one.

Prefer simple, functional, robust, readable code. Before adding new code,
inspect whether existing code can be simplified, reused, tightened, or deleted.

You may spawn subagents for read-only audit work. Use them to inspect separate
areas in parallel, then integrate the findings yourself. Do not delegate final
judgment, edits, or verification to subagents.

Initial context:
1. Read AGENTS.md or equivalent repository instructions first.
2. Inspect README/docs/examples to understand the intended public API.
3. Inspect tests and build configuration before editing.
4. Check git status and preserve dirty user changes.
5. Do not commit unless explicitly asked.

Suggested subagent audit split:
- API/state subagent:
  Inspect public headers, examples, initialization, readiness, status, health,
  reset, probe/detect, and error-access behavior.

- I2C transaction subagent:
  Inspect register reads/writes, transaction boundaries, timeout handling,
  retry behavior, recovery logic, bus ownership, and error propagation.

- Stability/memory subagent:
  Inspect blocking loops, dynamic allocation, buffer bounds, string/container
  growth, integer overflow, wraparound timing, concurrency assumptions, and
  repeated-call behavior.

- Test/diagnostics subagent:
  Inspect unit tests, examples, HIL hooks, diagnostics/self-tests, logging, and
  whether current tests cover error paths.

Audit for these issues:
- Functions that can report success after a failed I2C operation.
- Initialization paths that leave the object half-ready or silently usable.
- Health/status APIs that are inconsistent with actual state.
- Probe/detect functions that mutate state unexpectedly.
- Public methods that use hardware before initialization without a clear error.
- Missing or ignored timeout values.
- Unbounded polling loops, retries, waits, or recovery attempts.
- Hidden bus reset/reconfiguration from inside a device driver.
- I2C errors that are swallowed, overwritten too early, or not observable.
- Buffer overflows, off-by-one errors, invalid register lengths, unchecked
  caller buffers, or invalid null-pointer handling.
- Steady-path dynamic allocation or unbounded container/string growth.
- Inconsistent naming or behavior compared with nearby code.
- Duplicate helper code that can be simplified safely.
- Dead code, unused state, unused abstractions, placeholders, or future stubs.
- Tests that assert only happy paths while failures are untested.
- Diagnostics that can hang, damage state, or hide failures.
- Documentation/examples that contradict the actual API.

Finding format:
For each finding, record:
- Severity: critical, high, medium, or low.
- File and line/function reference.
- Exact current behavior.
- Why it is risky from a functional or system-stability point of view.
- Minimal fix.
- Tests or build checks needed.

Prioritization:
- Fix correctness and hang/data-corruption risks before style or naming.
- Fix issues with small, local changes before proposing API expansion.
- Prefer deleting or simplifying broken code over adding wrappers around it.
- Prefer readable local control flow and explicit state over clever compact
  code.
- Do not introduce a broad framework, registry, generic bus manager, placeholder
  class, or speculative extension point.
- Add an abstraction only when there is a concrete current caller and test.

Implementation workflow:
1. Build a short ordered fix list from the audit findings.
2. Implement one fix at a time.
3. Keep each fix tightly scoped.
4. After each fix, run the smallest relevant test or static check.
5. If a fix changes public behavior, update tests and examples/docs as needed.
6. Stop and report if a required architectural decision is missing.
7. Do not mask failures with silent retries or fake success.

I2C-specific implementation rules:
- Every operation that can block must be timeout-bounded.
- Timeout, NACK, bus error, invalid argument, not initialized, and device
  mismatch should be distinguishable when the existing error model allows it.
- Do not reconfigure or reset a shared bus unless this repository explicitly
  owns the bus at that layer.
- Separate read-only status from active hardware probing.
- Keep chip protocol in the driver/wrapper and application policy outside it.
- Do not add fake devices, simulated buses, or test doubles to production paths.
- HIL tests must be explicit and bounded. Never claim HIL passed unless it ran
  on hardware.

Testing:
- Run existing unit/native tests when available.
- Run library build/example build when available.
- Add focused tests for each fixed bug when the repository has a test setup.
- If hardware is required and unavailable, document exactly what could not be
  verified.
- Do not claim success for commands that were not run.

Final response:
- List files changed.
- List findings fixed, ordered by severity.
- Summarize tests/builds/HIL commands run and results.
- State intentional omissions and open questions.
- Keep the response concise and factual.
