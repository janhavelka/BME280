# Contributing

Thank you for considering contributing to this project!

## Quick Start

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Make your changes
4. Run the focused validation gate for your change, including host tests and
   contract checks when API, examples, HIL tooling, or docs change
5. Commit with a clear message: `git commit -m "feat: add X"`
6. Push and open a Pull Request

## Guidelines

### Code Style
- Follow existing code style (see `.clang-format`)
- Use `constexpr` instead of macros for constants
- Prefer explicit over implicit
- No heap allocations in steady-state library code

### Commits
- Use [Conventional Commits](https://www.conventionalcommits.org/) format:
  - `feat:` new feature
  - `fix:` bug fix
  - `docs:` documentation only
  - `refactor:` code change that neither fixes a bug nor adds a feature
  - `test:` adding or updating tests
  - `chore:` maintenance tasks

### Pull Requests
- Keep PRs focused (one feature/fix per PR)
- Update documentation if needed
- Add changelog entry under `[Unreleased]`
- Do not claim hardware, ESP-IDF runtime, fault-injection, or accuracy
  validation unless the exact command and artifact evidence exists
- Ensure CI passes

Baseline local gate for release-facing changes:

```bash
python tools/check_core_timing_guard.py
python tools/test_run_i2c_hil_parser.py
python tools/check_hil_contract.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python tools/check_release_metadata.py
python -m platformio test -e native
python -m platformio run -e esp32s2dev
python -m platformio run -e esp32s3dev
```

### What We Accept
- Bug fixes
- Documentation improvements
- Performance improvements (with benchmarks)
- New examples (if they demonstrate a common use case)

### What We Probably Won't Accept
- Breaking API changes without discussion
- Heavy dependencies
- Platform-specific code in the library core
- Features that add heap allocations in steady state

## Questions?

Open a GitHub Discussion or Issue for questions.
