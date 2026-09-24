"""
AirSim integration bridge for Scandium.

Provides high-level interface for AirSim simulation.
"""

from dataclasses import dataclass
from typing import Any, Optional
import time

import numpy as np
from numpy.typing import NDArray

from scandium.perception.camera import Frame, ICameraSource, CameraHealth
from scandium.logging.setup import get_logger

logger = get_logger(__name__)


@dataclass
class VehicleState:
    """
    Vehicle state from AirSim.

    Attributes:
        position: Position (x, y, z) in NED frame (m).
        velocity: Velocity (vx, vy, vz) in NED frame (m/s).
        orientation: Quaternion (w, x, y, z).
        timestamp: State timestamp.
    """

    position: NDArray[np.float64]
    velocity: NDArray[np.float64]
    orientation: NDArray[np.float64]
    timestamp: float


class AirSimBridge:
    """
    High-level AirSim integration for simulation testing.

    Provides vehicle control and camera access.
    """

    def __init__(
        self,
        ip: str = "127.0.0.1",
        vehicle_name: str = "Drone1",
        camera_name: str = "0",
    ) -> None:
        """
        Initialize AirSim bridge.

        Args:
            ip: AirSim server IP address.
            vehicle_name: Name of the vehicle in AirSim.
            camera_name: Camera name to use.
        """
        self._ip = ip
        self._vehicle_name = vehicle_name
        self._camera_name = camera_name
        self._client: Optional[Any] = None
        self._connected = False

    def connect(self) -> bool:
        """
        Connect to AirSim.

        Returns:
            True if connection successful.
        """
        try:
            import airsim

            self._client = airsim.MultirotorClient(ip=self._ip)
            self._client.confirmConnection()
            self._client.enableApiControl(True, self._vehicle_name)

            self._connected = True
            logger.info("airsim_connected", ip=self._ip, vehicle=self._vehicle_name)
            return True

        except ImportError:
            logger.error("airsim_not_installed")
            return False
        except Exception as e:
            logger.error("airsim_connection_failed", error=str(e))
            return False

    def disconnect(self) -> None:
        """Disconnect from AirSim."""
        if self._client is not None:
            try:
                self._client.enableApiControl(False, self._vehicle_name)
            except Exception:
                pass
            self._client = None
            self._connected = False
            logger.info("airsim_disconnected")

    def arm(self) -> bool:
        """Arm the vehicle."""
        if self._client is None:
            return False

        try:
            self._client.armDisarm(True, self._vehicle_name)
            logger.info("airsim_armed")
            return True
        except Exception as e:
            logger.error("airsim_arm_failed", error=str(e))
            return False

    def disarm(self) -> bool:
        """Disarm the vehicle."""
        if self._client is None:
            return False

        try:
            self._client.armDisarm(False, self._vehicle_name)
            logger.info("airsim_disarmed")
            return True
        except Exception as e:
            logger.error("airsim_disarm_failed", error=str(e))
            return False

    def takeoff(self, altitude: float = 10.0, timeout: float = 30.0) -> bool:
        """
        Execute takeoff.

        Args:
            altitude: Target altitude in meters.
            timeout: Timeout in seconds.

        Returns:
            True if takeoff successful.
        """
        if self._client is None:
            return False

        try:
            future = self._client.takeoffAsync(vehicle_name=self._vehicle_name)
            future.join()

            # Move to altitude
            self._client.moveToZAsync(
                -altitude,
                velocity=2.0,
                vehicle_name=self._vehicle_name,
            ).join()

            logger.info("airsim_takeoff_complete", altitude=altitude)
            return True

        except Exception as e:
            logger.error("airsim_takeoff_failed", error=str(e))
            return False

    def land(self) -> bool:
        """Execute landing."""
        if self._client is None:
            return False

        try:
            self._client.landAsync(vehicle_name=self._vehicle_name).join()
            logger.info("airsim_landing_complete")
            return True
        except Exception as e:
            logger.error("airsim_land_failed", error=str(e))
            return False

    def move_to_position(
        self,
        x: float,
        y: float,
        z: float,
        velocity: float = 2.0,
    ) -> bool:
        """
        Move to position in NED frame.

        Args:
            x: X position (North).
            y: Y position (East).
            z: Z position (Down, negative for altitude).
            velocity: Movement velocity.

        Returns:
            True if movement initiated.
        """
        if self._client is None:
            return False

        try:
            self._client.moveToPositionAsync(
                x,
                y,
                z,
                velocity=velocity,
                vehicle_name=self._vehicle_name,
            ).join()
            return True
        except Exception as e:
            logger.error("airsim_move_failed", error=str(e))
            return False

    def get_state(self) -> Optional[VehicleState]:
        """
        Get current vehicle state.

        Returns:
            VehicleState or None if unavailable.
        """
        if self._client is None:
            return None

        try:
            state = self._client.getMultirotorState(vehicle_name=self._vehicle_name)

            pos = state.kinematics_estimated.position
            vel = state.kinematics_estimated.linear_velocity
            orient = state.kinematics_estimated.orientation

            return VehicleState(
                position=np.array([pos.x_val, pos.y_val, pos.z_val]),
                velocity=np.array([vel.x_val, vel.y_val, vel.z_val]),
                orientation=np.array(
                    [orient.w_val, orient.x_val, orient.y_val, orient.z_val]
                ),
                timestamp=time.time(),
            )
        except Exception:
            return None

    def get_frame(self) -> Optional[Frame]:
        """
        Capture frame from camera.

        Returns:
            Frame or None if unavailable.
        """
        if self._client is None:
            return None

        try:
            import airsim

            responses = self._client.simGetImages(
                [
                    airsim.ImageRequest(
                        self._camera_name, airsim.ImageType.Scene, False, False
                    )
                ],
                vehicle_name=self._vehicle_name,
            )

            if not responses or len(responses) == 0:
                return None

            response = responses[0]
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_bgr = img1d.reshape(response.height, response.width, 3)

            return Frame(
                image_bgr=img_bgr,
                timestamp_s=time.time(),
                frame_id=0,
                meta={"source": "airsim"},
            )
        except Exception:
            return None

    def create_camera_source(self) -> ICameraSource:
        """
        Create camera source for the perception pipeline.

        Returns:
            ICameraSource connected to AirSim.
        """
        from scandium.perception.camera import AirSimCameraSource

        return AirSimCameraSource(
            ip=self._ip,
            vehicle_name=self._vehicle_name,
            camera_name=self._camera_name,
        )

    def reset(self) -> bool:
        """Reset simulation."""
        if self._client is None:
            return False

        try:
            self._client.reset()
            self._client.enableApiControl(True, self._vehicle_name)
            logger.info("airsim_reset")
            return True
        except Exception as e:
            logger.error("airsim_reset_failed", error=str(e))
            return False

    @property
    def is_connected(self) -> bool:
        """Check if connected to AirSim."""
        return self._connected
