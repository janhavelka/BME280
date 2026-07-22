# Security Policy

## Supported Versions

| Version | Security support |
| --- | --- |
| 2.0.x | Supported |
| 1.x and older | Best effort only |

## Reporting a Vulnerability

Do not open a public issue for a suspected vulnerability. Email
`info@thymos.cz` with:

- the affected version and target;
- a description of the issue and potential impact;
- reproducible steps or a minimal test case;
- any proposed mitigation, if available.

The maintainer aims to acknowledge reports within 48 hours and to provide a fix
or mitigation for critical issues within 14 days. These are response targets,
not service-level guarantees.

## Security Boundary

The library parses no network traffic and owns no persistent storage. Its main
security-relevant boundary is the application-supplied transport and time
callbacks. Applications must:

- keep callback/user-pointer storage valid for the configured driver lifetime;
- serialize each driver instance and shared I2C bus externally;
- enforce finite bus-lock and transfer timeouts;
- validate any untrusted values before mapping them into `Config`, typed
  settings, raw register access, or diagnostic CLI commands;
- avoid calling public APIs from ISRs or recursively from transport callbacks;
- treat raw control/reset register writes and physical fault injection as
  privileged diagnostic operations;
- use a watchdog and application-owned retry/backoff policy appropriate to the
  product.

The driver has no heap allocation in steady-state core paths, but that property
alone is not a security guarantee.
