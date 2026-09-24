"""
ArduPilot SITL orchestrator for Scandium.

Provides management of ArduPilot Software-In-The-Loop simulation instances
for development, testing, and validation purposes.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Any
import subprocess
import time
import os

from scandium.logging.setup import get_logger

logger = get_logger(__name__)


@dataclass
class SitlConfig:
    """
    ArduPilot SITL configuration parameters.

    Attributes:
        vehicle_type: Vehicle type (copter, plane, rover).
        frame_type: Frame configuration (quad, hexa, octa).
        home_lat: Home latitude in degrees.
        home_lon: Home longitude in degrees.
        home_alt: Home altitude in meters.
        home_heading: Home heading in degrees.
        speedup: Simulation speedup factor.
        instance: SITL instance number for multi-vehicle.
        sysid: System ID for MAVLink.
        defaults_path: Path to parameter defaults file.
        sim_address: Address for SITL communication.
    """

    vehicle_type: str = "copter"
    frame_type: str = "quad"
    home_lat: float = 40.072842
    home_lon: float = 32.866287
    home_alt: float = 584.0
    home_heading: float = 0.0
    speedup: float = 1.0
    instance: int = 0
    sysid: int = 1
    defaults_path: Optional[str] = None
    sim_address: str = "127.0.0.1"


class ArduPilotSitlOrchestrator:
    """
    Orchestrates ArduPilot SITL simulation instances.

    Manages lifecycle of sim_vehicle.py processes for testing
    and development workflows.
    """

    def __init__(
        self,
        ardupilot_path: Optional[Path] = None,
        config: Optional[SitlConfig] = None,
    ) -> None:
        """
        Initialize SITL orchestrator.

        Args:
            ardupilot_path: Path to ArduPilot repository root.
            config: SITL configuration parameters.
        """
        self._ardupilot_path = ardupilot_path or self._find_ardupilot()
        self._config = config or SitlConfig()
        self._process: Optional[subprocess.Popen[str]] = None
        self._mavproxy_process: Optional[subprocess.Popen[str]] = None
        self._started = False

    def _find_ardupilot(self) -> Optional[Path]:
        """Attempt to locate ArduPilot installation."""
        candidates = [
            Path.home() / "ardupilot",
            Path("/opt/ardupilot"),
            Path("../ardupilot"),
        ]

        for candidate in candidates:
            if candidate.exists() and (candidate / "Tools" / "autotest").exists():
                return candidate

        return None

    def start(self, wait_ready: bool = True, timeout_s: float = 60.0) -> bool:
        """
        Start ArduPilot SITL instance.

        Args:
            wait_ready: Whether to wait for vehicle to be ready.
            timeout_s: Timeout for waiting.

        Returns:
            True if started successfully.
        """
        if self._ardupilot_path is None:
            logger.error("ardupilot_path_not_found")
            return False

        sim_vehicle = self._ardupilot_path / "Tools" / "autotest" / "sim_vehicle.py"

        if not sim_vehicle.exists():
            logger.error("sim_vehicle_not_found", path=str(sim_vehicle))
            return False

        home_str = (
            f"{self._config.home_lat},{self._config.home_lon},"
            f"{self._config.home_alt},{self._config.home_heading}"
        )

        cmd = [
            "python3",
            str(sim_vehicle),
            f"--vehicle={self._config.vehicle_type}",
            f"--frame={self._config.frame_type}",
            f"--home={home_str}",
            f"--speedup={self._config.speedup}",
            f"--instance={self._config.instance}",
            f"--sysid={self._config.sysid}",
            "--no-mavproxy",
            "-w",
        ]

        if self._config.defaults_path:
            cmd.append(f"--add-param-file={self._config.defaults_path}")

        try:
            logger.info(
                "sitl_starting",
                vehicle=self._config.vehicle_type,
                frame=self._config.frame_type,
            )

            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=str(self._ardupilot_path),
            )

            if wait_ready:
                self._wait_for_ready(timeout_s)

            self._started = True
            logger.info("sitl_started", pid=self._process.pid)
            return True

        except Exception as e:
            logger.error("sitl_start_failed", error=str(e))
            return False

    def _wait_for_ready(self, timeout_s: float) -> None:
        """Wait for SITL to be ready."""
        start_time = time.time()

        while time.time() - start_time < timeout_s:
            if self._process is None:
                break

            if self._process.poll() is not None:
                raise RuntimeError("SITL process terminated unexpectedly")

            time.sleep(1.0)

            # Check if ready by attempting connection
            # This is a simplified check
            if time.time() - start_time > 5.0:
                break

    def stop(self) -> None:
        """Stop SITL instance."""
        if self._process is not None:
            try:
                self._process.terminate()
                self._process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
            finally:
                self._process = None
                self._started = False
                logger.info("sitl_stopped")

        if self._mavproxy_process is not None:
            try:
                self._mavproxy_process.terminate()
                self._mavproxy_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._mavproxy_process.kill()
            finally:
                self._mavproxy_process = None

    def restart(self) -> bool:
        """Restart SITL instance."""
        self.stop()
        time.sleep(1.0)
        return self.start()

    def send_command(self, command: str) -> bool:
        """
        Send command to SITL via MAVProxy (if available).

        Args:
            command: Command string.

        Returns:
            True if command sent successfully.
        """
        # This would require MAVProxy integration
        logger.warning("send_command_not_implemented")
        return False

    @property
    def is_running(self) -> bool:
        """Check if SITL is running."""
        if self._process is None:
            return False
        return self._process.poll() is None

    @property
    def connection_string(self) -> str:
        """Get MAVLink connection string for this SITL instance."""
        port = 14550 + self._config.instance * 10
        return f"udp:{self._config.sim_address}:{port}"

    def __enter__(self) -> "ArduPilotSitlOrchestrator":
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, *args: object) -> None:
        """Context manager exit."""
        self.stop()
