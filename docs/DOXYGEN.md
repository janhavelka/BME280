# Doxygen Usage

This library ships a standardized `Doxyfile` for unified API documentation.

## Generate docs

```bash
doxygen Doxyfile
```

## Output

Generated HTML documentation is written to:

- `docs/doxygen/html/index.html`

## Scope

The generated docs include:

- public headers under `include/`
- project `README.md` as the main page
- portability and unification notes (`docs/IDF_PORT.md`, `docs/UNIFICATION_STANDARD.md`)

## Notes

- `EXTRACT_ALL = YES` is enabled so the complete API surface is visible, including members that do not yet have explicit doc blocks.
- Doxygen warnings for malformed tags remain enabled (`WARN_IF_DOC_ERROR = YES`).
