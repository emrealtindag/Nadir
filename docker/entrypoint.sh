#!/bin/bash
# Nadir Docker Entrypoint Script
# Handles container initialization and signal management

set -e

# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------
NADIR_LOG_LEVEL="${NADIR_LOG_LEVEL:-INFO}"
NADIR_LOG_FORMAT="${NADIR_LOG_FORMAT:-json}"

# ------------------------------------------------------------------------------
# Signal Handlers
# ------------------------------------------------------------------------------
cleanup() {
    echo "[ENTRYPOINT] Received shutdown signal, cleaning up..."

    # Send SIGTERM to child processes
    if [ -n "$NADIR_PID" ]; then
        kill -TERM "$NADIR_PID" 2>/dev/null || true
        wait "$NADIR_PID" 2>/dev/null || true
    fi

    echo "[ENTRYPOINT] Cleanup complete"
    exit 0
}

trap cleanup SIGTERM SIGINT SIGQUIT

# ------------------------------------------------------------------------------
# Pre-flight Checks
# ------------------------------------------------------------------------------
preflight_checks() {
    echo "[ENTRYPOINT] Running pre-flight checks..."

    # Verify Python environment
    if ! command -v python &> /dev/null; then
        echo "[ENTRYPOINT] ERROR: Python not found"
        exit 1
    fi

    # Verify nadir installation
    if ! command -v nadir &> /dev/null; then
        echo "[ENTRYPOINT] ERROR: Nadir CLI not found"
        exit 1
    fi

    # Check configuration file if specified
    for arg in "$@"; do
        if [[ "$arg" == "--config" ]]; then
            shift
            CONFIG_FILE="$1"
            if [ -n "$CONFIG_FILE" ] && [ ! -f "$CONFIG_FILE" ]; then
                echo "[ENTRYPOINT] WARNING: Config file not found: $CONFIG_FILE"
            fi
            break
        fi
        shift
    done

    echo "[ENTRYPOINT] Pre-flight checks passed"
}

# ------------------------------------------------------------------------------
# Main Execution
# ------------------------------------------------------------------------------
main() {
    preflight_checks "$@"

    echo "[ENTRYPOINT] Starting Nadir..."
    echo "[ENTRYPOINT] Log Level: $NADIR_LOG_LEVEL"
    echo "[ENTRYPOINT] Log Format: $NADIR_LOG_FORMAT"
    echo "[ENTRYPOINT] Command: $@"

    # Execute the command
    exec "$@"
}

# ------------------------------------------------------------------------------
# Entry Point
# ------------------------------------------------------------------------------
main "$@"
