# Deployment Guide

Guide for deploying Nadir on embedded companion computers (NVIDIA Jetson, Raspberry Pi 5).

---

## Hardware Integration

1. **Companion Computer Mount**:
   - Shock-absorbent mount for companion computer.
   - Global shutter downward camera rigidly coupled to aircraft frame.

2. **Serial Telemetry Connection**:
   - Autopilot TELEM2 port connected to Companion UART (e.g. `/dev/ttyTHS1` on Jetson, `/dev/ttyAMA0` on Raspberry Pi).
   - Baud rate: `921600` baud.

3. **Autopilot Parameter Settings**:
   - `SERIAL2_PROTOCOL = 2` (MAVLink 2)
   - `SERIAL2_BAUD = 921`
   - `PLND_ENABLED = 1`
   - `PLND_TYPE = 1` (Companion Computer MAVLink)

---

## Production Execution via Docker

```bash
docker build -f docker/Dockerfile -t nadir:latest .
docker run --net=host --device=/dev/ttyTHS1 nadir:latest nadir run --config configs/production.yaml
```
