#!/bin/bash
# ArduPilot SITL Launcher for Scandium
#
# Launches ArduPilot SITL with appropriate configuration for
# precision landing development and testing.
#
# Prerequisites:
#   - ArduPilot repository cloned to ~/ardupilot
#   - Python 3 with required dependencies
#   - MAVProxy (optional, for GCS connection)
#
# Usage:
#   ./scripts/run_sitl_ardupilot.sh [options]
#
# Options:
#   --home LAT,LON,ALT,HDG   Set home position (default: Ankara, Turkey)
#   --speedup N              Simulation speedup factor (default: 1)
#   --instance N             SITL instance number (default: 0)
#   --mavproxy               Enable MAVProxy console
#   --help                   Show this help message

set -e

# ==============================================================================
# Configuration
# ==============================================================================

ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"
VEHICLE="copter"
FRAME="quad"
HOME_LOCATION="40.072842,32.866287,584,0"  # Ankara, Turkey
SPEEDUP=1
INSTANCE=0
ENABLE_MAVPROXY=false
SYSID=1

# ==============================================================================
# Argument Parsing
# ==============================================================================

show_help() {
    echo "ArduPilot SITL Launcher for Scandium"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --home LAT,LON,ALT,HDG   Set home position"
    echo "  --speedup N              Simulation speedup factor (default: 1)"
    echo "  --instance N             SITL instance number (default: 0)"
    echo "  --mavproxy               Enable MAVProxy console"
    echo "  --help                   Show this help message"
    echo ""
    echo "Example:"
    echo "  $0 --speedup 2 --mavproxy"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --home)
            HOME_LOCATION="$2"
            shift 2
            ;;
        --speedup)
            SPEEDUP="$2"
            shift 2
            ;;
        --instance)
            INSTANCE="$2"
            shift 2
            ;;
        --mavproxy)
            ENABLE_MAVPROXY=true
            shift
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# ==============================================================================
# Validation
# ==============================================================================

if [ ! -d "$ARDUPILOT_HOME" ]; then
    echo "Error: ArduPilot directory not found: $ARDUPILOT_HOME"
    echo "Please clone ArduPilot or set ARDUPILOT_HOME environment variable."
    exit 1
fi

if [ ! -f "$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py" ]; then
    echo "Error: sim_vehicle.py not found in ArduPilot directory."
    exit 1
fi

# ==============================================================================
# Launch SITL
# ==============================================================================

echo "============================================================"
echo "ArduPilot SITL Launcher"
echo "============================================================"
echo "ArduPilot: $ARDUPILOT_HOME"
echo "Vehicle:   $VEHICLE"
echo "Frame:     $FRAME"
echo "Home:      $HOME_LOCATION"
echo "Speedup:   ${SPEEDUP}x"
echo "Instance:  $INSTANCE"
echo "MAVProxy:  $ENABLE_MAVPROXY"
echo "============================================================"

cd "$ARDUPILOT_HOME"

# Build command
CMD=(
    python3
    Tools/autotest/sim_vehicle.py
    --vehicle="$VEHICLE"
    --frame="$FRAME"
    --home="$HOME_LOCATION"
    --speedup="$SPEEDUP"
    --instance="$INSTANCE"
    --sysid="$SYSID"
    -w
)

if [ "$ENABLE_MAVPROXY" = false ]; then
    CMD+=(--no-mavproxy)
fi

# Calculate MAVLink ports
UDP_PORT=$((14550 + INSTANCE * 10))
echo ""
echo "MAVLink UDP port: $UDP_PORT"
echo "Connection string: udp:127.0.0.1:$UDP_PORT"
echo ""
echo "Starting SITL..."
echo "============================================================"

# Execute
exec "${CMD[@]}"
