import os

path = "/home/emre/ardupilot/Tools/autotest/default_params/gazebo-iris.parm"
with open(path, "r", encoding="utf-8") as f:
    lines = f.readlines()

# Filter out old parameters if any
new_lines = []
for line in lines:
    if any(line.strip().startswith(p) for p in ["PLND_ENABLED", "PLND_TYPE", "PLND_EST_TYPE", "PLND_XY_DIST_MAX", "WPNAV_SPEED", "WPNAV_ACCEL", "WP_SPD", "AUTO_OPTIONS", "DISARM_DELAY", "# Nadir"]):
        continue
    new_lines.append(line)

# Add our optimized parameters
new_lines.append("\n# Nadir Precision Landing & Speed Configuration\n")
new_lines.append("PLND_ENABLED 1\n")
new_lines.append("PLND_TYPE 1\n")
new_lines.append("PLND_EST_TYPE 0\n")
new_lines.append("WP_SPD 5.6\n")
new_lines.append("AUTO_OPTIONS 3\n")
new_lines.append("DISARM_DELAY 30\n")

with open(path, "w", encoding="utf-8") as f:
    f.writelines(new_lines)

print("Successfully baked Nadir parameters (WP_SPD=5.6 m/s) into gazebo-iris.parm!")
