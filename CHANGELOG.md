# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Initial project structure and build configuration
- Core perception layer with ArUco and AprilTag detection
- MAVLink LANDING_TARGET message publishing
- Finite State Machine (FSM) for landing sequence control
- Landability analysis with heuristic scoring
- Configuration management with Pydantic validation
- SITL integration for ArduPilot and PX4
- AirSim simulation bridge
- Comprehensive test suite with unit and integration tests
- CI/CD pipeline with GitHub Actions
- Docker containerization support
- MkDocs documentation with Material theme

### Changed

- None

### Deprecated

- None

### Removed

- None

### Fixed

- None

### Security

- None

## [0.1.0] - 2025-01-15

### Added

- Initial release of Scandium precision landing system
- ArUco marker detection with OpenCV backend
- AprilTag marker detection support (optional dependency)
- Camera-to-body frame coordinate transformations
- Temporal filtering with exponential smoothing
- MAVLink LANDING_TARGET message generation
- UDP and serial transport layer support
- Landing FSM with SEARCH, ACQUIRE, ALIGN, DESCEND states
- Landability heuristic analysis (texture, motion, edge density)
- YAML-based configuration with schema validation
- CLI interface with run, sim, diagnostics commands
- ArduPilot SITL integration and test scenarios
- PX4 SITL integration and test scenarios
- AirSim bridge for simulation image acquisition
- Unit tests for core components
- Integration tests for pipeline validation
- GitHub Actions CI/CD workflows
- Docker support with multi-stage builds
- Comprehensive documentation

### Security

- Tag ID allowlist validation for anti-spoofing
- Marker size consistency checks
- Reprojection error threshold enforcement

---

## Version History Summary

| Version | Release Date | Description |
|---------|--------------|-------------|
| 0.1.0   | 2025-01-15   | Initial release |

---

## Upgrade Notes

### Upgrading to 0.1.0

This is the initial release. No upgrade path from previous versions.

---

## Links

- [Repository](https://github.com/scandium-oss/scandium)
- [Documentation](https://scandium-oss.github.io/scandium)
- [Issue Tracker](https://github.com/scandium-oss/scandium/issues)

[Unreleased]: https://github.com/scandium-oss/scandium/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/scandium-oss/scandium/releases/tag/v0.1.0
