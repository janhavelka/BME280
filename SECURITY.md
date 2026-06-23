# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 1.7.x   | :white_check_mark: |
| < 1.7   | Best effort only   |

## Reporting a Vulnerability

If you discover a security vulnerability within this library, please follow responsible disclosure:

1. **Do NOT** open a public GitHub issue.
2. Email the maintainer at: `info@thymos.cz`.
3. Include:
   - A description of the vulnerability
   - Steps to reproduce
   - Potential impact
   - Any suggested fixes (optional)

We will acknowledge receipt within 48 hours and aim to provide a fix or mitigation within 14 days for critical issues.

## Scope

This library is designed for embedded systems. Security considerations include:
- No dynamic memory allocation in steady state (reduces attack surface)
- No network code (networking is out of scope for this library)
- No persistent storage by default (NVS side effects are opt-in)

## Security Best Practices for Users

- Always validate external inputs before passing to `Config`
- Use hardware watchdogs in production deployments
- Keep dependencies updated
