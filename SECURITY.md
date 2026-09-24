# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 0.1.x   | :white_check_mark: |

## Reporting a Vulnerability

We take security seriously. If you discover a security vulnerability within Scandium, please report it responsibly.

### How to Report

1. **Do NOT** create a public GitHub issue for security vulnerabilities
2. Email security concerns to: security@scandium-oss.example.com
3. Include the following information:
   - Description of the vulnerability
   - Steps to reproduce
   - Potential impact
   - Suggested fix (if any)

### What to Expect

- **Acknowledgment**: We will acknowledge receipt within 48 hours
- **Assessment**: We will assess the vulnerability within 7 days
- **Resolution**: Critical vulnerabilities will be addressed within 30 days
- **Disclosure**: We follow responsible disclosure practices

## Security Considerations

### Flight Safety

Scandium is designed for UAV precision landing systems. Security vulnerabilities in this context could have safety-critical implications:

- **Marker Spoofing**: The system includes anti-spoofing measures (tag allowlist, size consistency checks)
- **Communication Security**: MAVLink communication should be secured at the network level
- **Failsafe Behavior**: All error conditions default to safe autopilot behavior

### Best Practices for Deployment

1. **Network Isolation**: Run on isolated companion computer networks
2. **Authenticated MAVLink**: Use MAVLink 2.0 signing when available
3. **Configuration Validation**: Use the built-in config validation
4. **Dependency Updates**: Regularly update dependencies (`poetry update`)
5. **Logging**: Enable logging for audit trails

## Security Scanning

This project uses automated security scanning:

- **pip-audit**: Dependency vulnerability scanning
- **gitleaks**: Secret detection in commits
- **Dependabot**: Automated dependency updates

## Scope Limitations

This software is designed for:
- Flight safety
- Safe recovery operations
- Automatic landing
- Operational reliability

This software is **NOT** designed for and must **NOT** be used for:
- Weaponization
- Target engagement
- Attack optimization
- Any harmful purposes
