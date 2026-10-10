# Security & Safety Policy

## Reporting Security Issues

If you discover a security vulnerability or flight-safety bug in Nadir, please report it directly via email:

**Contact**: [emrevaltn@gmail.com](mailto:emrevaltn@gmail.com)

Please include:
- Description of the issue or unexpected behavior
- Steps or SITL scenario to reproduce
- Affected modules (e.g., MAVLink framing, PnP estimator, FSM)

Reports are reviewed promptly and addressed in upcoming release commits.

## Avionics & Telemetry Security Considerations

When deploying Nadir on physical companion computers (Jetson, Raspberry Pi) and autopilot datalinks:

- **MAVLink 2.0 Signing**: Enable MAVLink packet signing on serial telemetry channels (`SERIALx_PROTOCOL = 2`) to prevent unauthorized command injection on unencrypted radios.
- **Fiducial Tag Whitelisting**: Always configure `target_id_allowlist` in `configs/` to prevent tracking spurious or counterfeit markers in contested visual environments.
- **Network Boundaries**: Keep companion computer telemetry ports (UDP 14550 / TCP 5760) bound to local or secured private radio links (e.g., dedicated point-to-point telemetry).
- **Physical Rangefinder Redundancy**: Ensure 1D LiDAR (`DISTANCE_SENSOR`) is enabled to provide an independent physical ground-truth altitude gate for landing touchdown confirmation.
