# ADR 0002: Deterministic 9-State Finite State Machine (FSM)

## Status
Accepted

## Context
The landing sequence must transition from searching for a landing zone to safely touching down, while gracefully handling hardware failures, target loss, and dynamic obstacles (like humans).

## Decision
We implemented a strict, purely deterministic 9-state FSM (`INIT`, `IDLE`, `SEARCH`, `ACQUIRE`, `ALIGN`, `DESCEND`, `TOUCHDOWN`, `ABORT`, `FAILSAFE`).

## Rationale
1. **Determinism over Heuristics**: Autonomous systems in defense/aerospace require absolute predictability. By strictly separating states (e.g., `ACQUIRE` validates the filter convergence, `ALIGN` minimizes lateral error, `DESCEND` controls altitude), we can formally verify transitions.
2. **Fail-safe by Default**: Any unhandled exception, MAVLink timeout, or camera failure transitions the system to `FAILSAFE` (handing control back to the human pilot or autopilot's RTL mode).
3. **Go-Around capability**: The `ABORT` state acts as a trap for low landability scores or human presence, issuing a "GO_AROUND" command rather than attempting a risky landing.

## Consequences
- **Positive**: High traceability. Every state transition logs its exact cause. 
- **Negative**: The FSM is rigid. It requires careful tuning of thresholds (`acquire_confidence`, `max_variance`) to avoid oscillating between states (e.g., `ALIGN` and `SEARCH` chattering).
