"""
PX4 SITL orchestrator for Scandium.

Provides management of PX4 Software-In-The-Loop simulation instances
for development, testing, and validation purposes.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import subprocess
import time
import os

from scandium.logging.setup import get_logger

logger = get_logger(__name__)


@dataclass
class Px4SitlConfig:
    """
    PX4 SITL configuration parameters.

    Attributes:
        airframe: PX4 airframe type (iris, typhoon_h480, etc.).
        world: Gazebo world file name.
        headless: Run without graphics.
        home_lat: Home latitude in degrees.
        home_lon: Home longitude in degrees.
        home_alt: Home altitude in meters.
        speedup: Simulation speedup factor.
        instance: SITL instance number for multi-vehicle.
        px4_path: Path to PX4-Autopilot repository.
    """

    airframe: str = "iris"
    world: str = "empty"
    headless: bool = True
    home_lat: float = 40.072842
    home_lon: float = 32.866287
    home_alt: float = 584.0
    speedup: float = 1.0
    instance: int = 0
    px4_path: Optional[str] = None


class Px4SitlOrchestrator:
    """
    Orchestrates PX4 SITL simulation instances.

    Manages lifecycle of PX4 SITL processes with Gazebo simulation
    for testing and development workflows.
    """

    def __init__(self, config: Optional[Px4SitlConfig] = None) -> None:
        """
        Initialize PX4 SITL orchestrator.

        Args:
            config: SITL configuration parameters.
        """
        self._config = config or Px4SitlConfig()
        self._px4_path = self._find_px4()
        self._process: Optional[subprocess.Popen[str]] = None
        self._gazebo_process: Optional[subprocess.Popen[str]] = None
        self._started = False

    def _find_px4(self) -> Optional[Path]:
        """Attempt to locate PX4 installation."""
        if self._config.px4_path:
            return Path(self._config.px4_path)

        candidates = [
            Path.home() / "PX4-Autopilot",
            Path("/opt/PX4-Autopilot"),
            Path("../PX4-Autopilot"),
        ]

        for candidate in candidates:
            if candidate.exists() and (candidate / "build").exists():
                return candidate

        return None

    def start(self, wait_ready: bool = True, timeout_s: float = 90.0) -> bool:
        """
        Start PX4 SITL instance.

        Args:
            wait_ready: Whether to wait for vehicle to be ready.
            timeout_s: Timeout for waiting.

        Returns:
            True if started successfully.
        """
        if self._px4_path is None:
            logger.error("px4_path_not_found")
            return False

        # Set environment variables
        env = os.environ.copy()
        env["PX4_HOME_LAT"] = str(self._config.home_lat)
        env["PX4_HOME_LON"] = str(self._config.home_lon)
        env["PX4_HOME_ALT"] = str(self._config.home_alt)
        env["PX4_SIM_SPEED_FACTOR"] = str(self._config.speedup)

        if self._config.headless:
            env["HEADLESS"] = "1"

        # Construct make command
        target = f"px4_sitl_default gazebo_{self._config.airframe}"

        cmd = [
            "make",
            target,
        ]

        try:
            logger.info(
                "px4_sitl_starting",
                airframe=self._config.airframe,
                headless=self._config.headless,
            )

            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=str(self._px4_path),
                env=env,
            )

            if wait_ready:
                self._wait_for_ready(timeout_s)

            self._started = True
            logger.info("px4_sitl_started", pid=self._process.pid)
            return True

        except Exception as e:
            logger.error("px4_sitl_start_failed", error=str(e))
            return False

    def _wait_for_ready(self, timeout_s: float) -> None:
        """Wait for PX4 SITL to be ready."""
        start_time = time.time()

        while time.time() - start_time < timeout_s:
            if self._process is None:
                break

            if self._process.poll() is not None:
                raise RuntimeError("PX4 SITL process terminated unexpectedly")

            time.sleep(1.0)

            if time.time() - start_time > 15.0:
                break

    def stop(self) -> None:
        """Stop PX4 SITL instance."""
        if self._process is not None:
            try:
                self._process.terminate()
                self._process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
            finally:
                self._process = None
                self._started = False
                logger.info("px4_sitl_stopped")

        if self._gazebo_process is not None:
            try:
                self._gazebo_process.terminate()
                self._gazebo_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self._gazebo_process.kill()
            finally:
                self._gazebo_process = None

    def restart(self) -> bool:
        """Restart PX4 SITL instance."""
        self.stop()
        time.sleep(2.0)
        return self.start()

    @property
    def is_running(self) -> bool:
        """Check if PX4 SITL is running."""
        if self._process is None:
            return False
        return self._process.poll() is None

    @property
    def connection_string(self) -> str:
        """Get MAVLink connection string for this SITL instance."""
        port = 14540 + self._config.instance
        return f"udp://127.0.0.1:{port}"

    def __enter__(self) -> "Px4SitlOrchestrator":
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, *args: object) -> None:
        """Context manager exit."""
        self.stop()
