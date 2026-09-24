"""
Autopilot parameter profiles for Scandium.

Provides parameter recommendations for PX4 and ArduPilot.
"""

from dataclasses import dataclass


@dataclass
class ParameterProfile:
    """
    Autopilot parameter profile.

    Attributes:
        name: Profile name.
        autopilot: Target autopilot (ardupilot, px4).
        parameters: Dictionary of parameter name -> value.
        description: Profile description.
    """

    name: str
    autopilot: str
    parameters: dict[str, float | int | str]
    description: str


# ArduPilot Copter parameters for precision landing
ARDUPILOT_PRECISION_LANDING = ParameterProfile(
    name="ardupilot_precland",
    autopilot="ardupilot",
    parameters={
        "PLND_ENABLED": 1,  # Enable precision landing
        "PLND_TYPE": 1,  # Companion computer
        "PLND_EST_TYPE": 0,  # Use raw sensor
        "PLND_LAG": 0.02,  # Sensor lag (20ms)
        "PLND_XY_DIST_MAX": 4.0,  # Max horizontal distance (m)
        "PLND_STRICT": 1,  # Strict mode
        "PLND_TIMEOUT": 4.0,  # Landing timeout (s)
        "PLND_RET_MAX": 4,  # Max retries
        "PLND_OPTIONS": 0,  # Options bitmask
        "LAND_SPEED": 40,  # Landing speed (cm/s)
        "LAND_SPEED_HIGH": 100,  # High landing speed (cm/s)
    },
    description="ArduPilot Copter precision landing configuration",
)


# PX4 parameters for precision landing
PX4_PRECISION_LANDING = ParameterProfile(
    name="px4_precland",
    autopilot="px4",
    parameters={
        "PLD_BTOUT": 5.0,  # Search timeout (s)
        "PLD_HACC_RAD": 0.25,  # Horizontal acceptance radius (m)
        "PLD_FAPPR_ALT": 10.0,  # Final approach altitude (m)
        "PLD_SRCH_ALT": 15.0,  # Search altitude (m)
        "PLD_SRCH_TOUT": 10.0,  # Search timeout (s)
        "RTL_LAND_DELAY": 0.0,  # Land immediately on RTL
        "MPC_LAND_SPEED": 0.7,  # Landing speed (m/s)
        "MPC_LAND_ALT1": 10.0,  # Slow landing altitude (m)
        "MPC_LAND_ALT2": 5.0,  # Very slow landing altitude (m)
    },
    description="PX4 precision landing configuration",
)


def get_profile(autopilot: str) -> ParameterProfile:
    """
    Get parameter profile for autopilot.

    Args:
        autopilot: Autopilot type ('ardupilot' or 'px4').

    Returns:
        ParameterProfile for the specified autopilot.
    """
    if autopilot.lower() == "ardupilot":
        return ARDUPILOT_PRECISION_LANDING
    elif autopilot.lower() == "px4":
        return PX4_PRECISION_LANDING
    else:
        raise ValueError(f"Unknown autopilot: {autopilot}")


def format_params_for_mavlink(profile: ParameterProfile) -> list[tuple[str, float]]:
    """
    Format parameters for MAVLink PARAM_SET.

    Args:
        profile: Parameter profile.

    Returns:
        List of (param_id, value) tuples.
    """
    result = []
    for name, value in profile.parameters.items():
        if isinstance(value, str):
            continue  # Skip string parameters
        result.append((name, float(value)))
    return result
