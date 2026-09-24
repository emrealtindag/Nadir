#!/bin/bash
# PX4 SITL Launcher for Scandium
#
# Launches PX4 SITL with Gazebo simulation for precision landing
# development and testing.
#
# Prerequisites:
#   - PX4-Autopilot repository cloned to ~/PX4-Autopilot
#   - Gazebo Classic or Gazebo Garden installed
#   - ROS 2 (optional, for advanced integration)
#
# Usage:
#   ./scripts/run_sitl_px4.sh [options]
#
# Options:
#   --airframe NAME          PX4 airframe (default: iris)
#   --world NAME             Gazebo world (default: empty)
#   --headless               Run without graphics
#   --speedup N              Simulation speedup factor
#   --help                   Show this help message

set -e

# ==============================================================================
# Configuration
# ==============================================================================

PX4_HOME="${PX4_HOME:-$HOME/PX4-Autopilot}"
AIRFRAME="iris"
WORLD="empty"
HEADLESS=false
SPEEDUP=1
HOME_LAT=40.072842
HOME_LON=32.866287
HOME_ALT=584

# ==============================================================================
# Argument Parsing
# ==============================================================================

show_help() {
    echo "PX4 SITL Launcher for Scandium"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --airframe NAME   PX4 airframe (default: iris)"
    echo "  --world NAME      Gazebo world (default: empty)"
    echo "  --headless        Run without graphics"
    echo "  --speedup N       Simulation speedup factor"
    echo "  --help            Show this help message"
    echo ""
    echo "Available airframes:"
    echo "  iris              Standard quadrotor"
    echo "  iris_rplidar      Quadrotor with LIDAR"
    echo "  iris_vision       Quadrotor with vision sensor"
    echo "  typhoon_h480      Hexarotor with gimbal"
    echo ""
    echo "Example:"
    echo "  $0 --airframe iris --headless --speedup 2"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --airframe)
            AIRFRAME="$2"
            shift 2
            ;;
        --world)
            WORLD="$2"
            shift 2
            ;;
        --headless)
            HEADLESS=true
            shift
            ;;
        --speedup)
            SPEEDUP="$2"
            shift 2
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

if [ ! -d "$PX4_HOME" ]; then
    echo "Error: PX4-Autopilot directory not found: $PX4_HOME"
    echo "Please clone PX4-Autopilot or set PX4_HOME environment variable."
    exit 1
fi

# ==============================================================================
# Environment Setup
# ==============================================================================

export PX4_HOME_LAT=$HOME_LAT
export PX4_HOME_LON=$HOME_LON
export PX4_HOME_ALT=$HOME_ALT
export PX4_SIM_SPEED_FACTOR=$SPEEDUP

if [ "$HEADLESS" = true ]; then
    export HEADLESS=1
fi

# ==============================================================================
# Launch SITL
# ==============================================================================

echo "============================================================"
echo "PX4 SITL Launcher"
echo "============================================================"
echo "PX4 Home:  $PX4_HOME"
echo "Airframe:  $AIRFRAME"
echo "World:     $WORLD"
echo "Headless:  $HEADLESS"
echo "Speedup:   ${SPEEDUP}x"
echo "Home:      $HOME_LAT, $HOME_LON, $HOME_ALT"
echo "============================================================"

cd "$PX4_HOME"

# Build target
TARGET="px4_sitl_default gazebo-classic_${AIRFRAME}"

echo ""
echo "MAVLink UDP port: 14540"
echo "Connection string: udp://127.0.0.1:14540"
echo ""
echo "Building and starting SITL..."
echo "============================================================"

# Execute
exec make $TARGET
