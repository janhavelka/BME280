# AI Coder Prompt: Strengthen AGENTS Instructions

You are working inside one repository. Update this repository's AGENTS.md
instructions so future coding agents prefer simple, robust, safe solutions and
actively simplify existing code.

Goal:
Strengthen the agent rules so future coding agents prefer simple, clear,
correct, robust, and safe solutions. The agent should actively look for code
that can be simplified or deleted before adding new code.

Do not rewrite AGENTS.md from scratch. Preserve all existing project-specific
rules, architecture constraints, naming rules, build instructions, testing
requirements, and safety limits. Make the smallest clear edit that adds the
new guidance.

You may spawn subagents for read-only inspection if that helps you compare
sections or check for duplicated guidance. Keep final judgment, edits, and
verification in the main agent.

Required changes:

1. Inspect the current AGENTS.md first.
   - Identify the best existing section for engineering/workflow rules.
   - If a similar section already exists, extend it instead of creating a
     duplicate section.
   - If no suitable section exists, add a concise section named
     "Engineering Priorities" or "Workflow".

2. Add or strengthen these rules:
   - Prefer simplicity, clarity, correctness, robustness, safety, and code
     readability over clever abstractions or speculative flexibility.
   - Before coding, inspect whether existing code can be simplified, reused, or
     deleted.
   - Prefer deleting unnecessary code over adding more code.
   - Prefer extending the existing owner/module/API over creating a parallel
     abstraction.
   - Add a new service, class, file, interface, or abstraction only when there
     is a concrete current need and a clear test or caller for it.
   - Do not add placeholder classes, future stubs, empty managers, broad
     frameworks, plugin systems, registries, or generic layers unless the
     current task explicitly requires them.
   - Keep changes tightly scoped to the user's request.
   - Preserve dirty user changes and never revert unrelated work.

3. Add or strengthen robustness and safety rules:
   - No unbounded waits, retries, loops, allocations, queues, or buffers in
     steady paths.
   - Every hardware operation that can block must have a timeout and an
     observable failure path.
   - Recovery logic must be bounded, deterministic, and testable.
   - Prefer explicit state, explicit ownership, and small local helpers over
     hidden global state.
   - Do not hide hardware failures behind silent retries or fake success.
   - Avoid dynamic allocation in steady embedded paths unless it is already an
     accepted local pattern and the bound is clear.

4. Add or strengthen I2C-library-specific rules where appropriate:
   - The I2C bus must have one clear owner.
   - Device drivers should not directly own or reconfigure a shared bus unless
     this repository's architecture explicitly says so.
   - I2C transactions must be timeout-bounded and report errors clearly.
   - Do not implement chip protocols manually if an existing hardened project
     library already provides the needed timeout, recovery, and testability
     behavior.
   - Keep chip-level protocol code inside the driver/wrapper. Keep application
     policy outside the chip driver.
   - Do not add fake devices, simulated buses, or test doubles to production
     paths.

5. Keep the wording concrete.
   - Use short bullet points.
   - Avoid vague slogans like "write clean code" unless they are backed by a
     concrete action.
   - Avoid mentioning prompt numbers, temporary milestones, or this prompt in
     runtime code or public APIs.

6. After editing AGENTS.md:
   - Show the exact section(s) changed.
   - Explain whether you merged with an existing section or added a new one.
   - Do not commit unless explicitly asked.
   - If only AGENTS.md changed, tests are not required. Say that no code tests
     were run because only agent instructions changed.

Suggested wording to add, adapted to the existing file style:

- Prefer simplicity, clarity, correctness, robustness, and safety of the code.
- Before coding, inspect whether existing code can be simplified, reused, or
  deleted.
- Prefer deleting unnecessary code over adding new code.
- Prefer extending existing owners/modules/contracts over creating new parallel
  abstractions.
- Before adding a new service/class/file/interface, check whether an existing
  owner/module is the correct home.
- Add abstractions only for a concrete current need with a clear caller or test.
- Do not add placeholder classes, future stubs, empty managers, broad
  frameworks, service registries, or speculative extension points.
- Keep blocking operations timeout-bounded and make failures visible.
- Keep recovery logic bounded, deterministic, and testable.
