# Security Policy

## Supported Versions

| Version | Supported          |
|---------|--------------------|
| 3.4.x   | :white_check_mark: |
| 3.3.x   | :x:                |
| < 3.0   | :x:                |

## Reporting a Vulnerability

**Do NOT open a public GitHub issue for security vulnerabilities.**

Send a private report by email (see the GitHub profile `@Mukller` for an address) or via GitHub's private vulnerability reporting on the Security tab of this repository.

Please include:

- Description of the vulnerability and its impact
- Reproduction steps
- Affected version(s)
- Suggested fix (if you have one)

## Response Time

- Initial triage: within 7 days
- Fix release: within 30 days of confirmation

## Security Notes for this Project

- `secrets.h` (WiFi credentials) is gitignored. Use `secrets.h.example` as a template.
- WiFi credentials can also be configured at runtime via the captive portal (`/api/save-wifi`) and are stored in NVS (`Preferences`), not in source.
- The robot is a competition device on your own network — there is no internet exposure by default.
- Do not commit `secrets.h`, captured NVS dumps, or anything that contains WiFi passwords.

## Acknowledgements

Thanks for responsible disclosure. Valid reports are credited in the CHANGELOG unless anonymity is requested.
