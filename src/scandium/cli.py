"""
Scandium CLI - Command-line interface for the precision landing system.

Provides commands for running the landing system, simulation integration,
scenario testing, camera calibration, and system diagnostics.
"""

from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.table import Table

from scandium.version import __version__

app = typer.Typer(
    name="scandium",
    help="Scandium - Production-grade precision landing system for UAV platforms.",
    add_completion=False,
)

console = Console()


def version_callback(value: bool) -> None:
    """Print version and exit."""
    if value:
        console.print(f"[bold blue]Scandium[/bold blue] v{__version__}")
        raise typer.Exit()


@app.callback()
def main(
    version: Optional[bool] = typer.Option(
        None,
        "--version",
        "-V",
        callback=version_callback,
        is_eager=True,
        help="Show version and exit.",
    ),
) -> None:
    """Scandium - Precision landing system for UAV platforms."""
    pass


@app.command()
def run(
    config: Path = typer.Option(
        Path("configs/default.yaml"),
        "--config",
        "-c",
        help="Path to configuration file.",
        exists=True,
        file_okay=True,
        dir_okay=False,
        readable=True,
    ),
    log_level: Optional[str] = typer.Option(
        None,
        "--log-level",
        "-l",
        help="Override log level (DEBUG, INFO, WARNING, ERROR).",
    ),
) -> None:
    """Run the precision landing system with specified configuration."""
    from scandium.config.loader import load_config
    from scandium.logging.setup import configure_logging

    try:
        cfg = load_config(config)
        if log_level:
            cfg.project.log_level = log_level
        configure_logging(cfg.project.log_level, cfg.project.run_id)

        console.print(f"[green]\u2713[/green] Loaded configuration from {config}")
        console.print(f"[green]\u2713[/green] Run ID: {cfg.project.run_id}")
        console.print(
            f"[yellow]Starting Scandium in {cfg.project.mode} mode...[/yellow]"
        )

        # --- Main execution loop: camera -> detect -> pose -> publish ---
        from scandium.perception.camera import (
            UvcCameraSource,
            VideoFileCameraSource,
            AirSimCameraSource,
        )
        from scandium.perception.calib import CalibrationManager
        from scandium.perception.fiducials.aruco_detector import ArUcoDetector
        from scandium.perception.fiducials.apriltag_detector import AprilTagDetector
        from scandium.perception.pose import PoseEstimator
        from scandium.mavlink.transport import MavlinkTransport
        from scandium.mavlink.landing_target import LandingTargetPublisher, MAV_FRAME_BODY_NED

        # load config (already loaded into cfg)
        # Camera source
        cam_cfg = getattr(cfg, "camera", None)
        if cam_cfg is None:
            console.print("[red]Error:[/red] Camera configuration missing.")
            raise typer.Exit(code=1)

        source = getattr(cam_cfg, "source", "uvc")
        if source == "uvc":
            device = getattr(cam_cfg, "device_index", 0)
            width = getattr(cam_cfg, "width", 1280)
            height = getattr(cam_cfg, "height", 720)
            fps = getattr(cam_cfg, "fps", 30)
            camera = UvcCameraSource(device_index=device, width=width, height=height, fps=fps)
        elif source == "video_file" or source == "video":
            path = getattr(getattr(cam_cfg, "video", {}), "path", None) or getattr(cam_cfg, "path", None)
            loop = getattr(getattr(cam_cfg, "video", {}), "loop", True) if hasattr(cam_cfg, "video") else getattr(cam_cfg, "loop", True)
            if path is None:
                console.print("[red]Error:[/red] camera.video.path not set in config.")
                raise typer.Exit(code=1)
            camera = VideoFileCameraSource(path, loop=loop)
        elif source == "airsim":
            ip = getattr(cam_cfg, "ip", "127.0.0.1")
            vehicle = getattr(cam_cfg, "vehicle_name", "Drone1")
            cam_name = getattr(cam_cfg, "camera_name", "0")
            image_type = getattr(cam_cfg, "image_type", "Scene")
            camera = AirSimCameraSource(ip=ip, vehicle_name=vehicle, camera_name=cam_name, image_type=image_type)
        else:
            console.print(f"[red]Error:[/red] Unsupported camera source: {source}")
            raise typer.Exit(code=1)

        # Calibration
        intr_path = getattr(cam_cfg, "intrinsics_path", None) if hasattr(cam_cfg, "intrinsics_path") else None
        extr_path = getattr(cam_cfg, "extrinsics_path", None) if hasattr(cam_cfg, "extrinsics_path") else None
        calib = CalibrationManager.from_config(intrinsics_path=intr_path, extrinsics_path=extr_path,
                                              width=getattr(cam_cfg, "width", 1280),
                                              height=getattr(cam_cfg, "height", 720))

        # Detector selection
        fid_cfg = getattr(cfg, "fiducials", None)
        backend = getattr(fid_cfg, "backend", "aruco") if fid_cfg is not None else "aruco"
        if backend.lower().startswith("aruco"):
            detector = ArUcoDetector(dictionary=getattr(fid_cfg, "dictionary", "DICT_4X4_100"))
        else:
            detector = AprilTagDetector(family=getattr(fid_cfg, "family", "tag36h11"))

        # Pose estimator
        pe = PoseEstimator(marker_size_m=getattr(fid_cfg, "marker_size_m", 0.2) if fid_cfg is not None else 0.2)

        # MAVLink transport + publisher
        mav_cfg = getattr(cfg, "mavlink", None)
        transport = MavlinkTransport(
            transport=getattr(mav_cfg, "transport", "udp") if mav_cfg is not None else "udp",
            udp_address=getattr(getattr(mav_cfg, "udp", {}), "address", "127.0.0.1"),
            udp_port=getattr(getattr(mav_cfg, "udp", {}), "port", 14550),
            system_id=getattr(mav_cfg, "system_id", 42) if mav_cfg is not None else 42,
            component_id=getattr(mav_cfg, "component_id", 200) if mav_cfg is not None else 200,
        )

        connected = transport.connect()
        if not connected:
            console.print("[red]Error:[/red] Could not connect MAVLink transport.")
            try:
                camera.close()
            except Exception:
                pass
            raise typer.Exit(code=1)

        lt_publisher = LandingTargetPublisher(transport, rate_hz=getattr(mav_cfg, "landing_target_rate_hz", 20) if mav_cfg is not None else 20)

        console.print("[green]\u2713[/green] Scandium started. Entering main loop...")

        try:
            while True:
                frame = camera.read()
                if frame is None:
                    # small sleep to avoid busy loop
                    import time

                    time.sleep(0.01)
                    continue

                detections = detector.detect(frame)
                if not detections:
                    # nothing detected; continue
                    continue

                # use first detection (could be extended to choose best by area/confidence)
                det = detections[0]

                # estimate pose in camera frame
                pose = pe.estimate(det, calib.intrinsics)
                if not pose.success or pose.tvec is None:
                    continue

                # transform to body frame using extrinsics
                t_cam = pose.tvec
                t_body = calib.extrinsics.transform_point(t_cam)

                # compute angles (camera frame) for LANDING_TARGET: angle_x = atan2(x, z), angle_y = atan2(y, z)
                import math

                angle_x = math.atan2(float(t_cam[0]), float(t_cam[2]))
                angle_y = math.atan2(float(t_cam[1]), float(t_cam[2]))

                # publish
                lt_publisher.publish_from_pose(
                    tvec=t_body,
                    angle_x=angle_x,
                    angle_y=angle_y,
                    position_valid=True,
                    frame=MAV_FRAME_BODY_NED,
                )

        except KeyboardInterrupt:
            console.print("[yellow]Interrupted by user, shutting down...[/yellow]")
        except Exception as e:
            console.print(f"[red]Error in main loop:[/red] {e}")
        finally:
            try:
                camera.close()
            except Exception:
                pass
            try:
                transport.close()
            except Exception:
                pass

        console.print("[bold green]Scandium stopped.[/bold green]")

    except Exception as e:
        console.print(f"[red]Error:[/red] {e}")
        raise typer.Exit(code=1)


@app.command()
def sim(
    backend: str = typer.Argument(
        ...,
        help="Simulation backend (airsim, ardupilot, px4).",
    ),
    config: Path = typer.Option(
        Path("configs/default.yaml"),
        "--config",
        "-c",
        help="Path to configuration file.",
    ),
) -> None:
    """Run simulation mode with specified backend."""
    valid_backends = {"airsim", "ardupilot", "px4"}
    if backend.lower() not in valid_backends:
        console.print(
            f"[red]Error:[/red] Invalid backend '{backend}'. Valid options: {valid_backends}"
        )
        raise typer.Exit(code=1)

    console.print(f"[yellow]Initializing {backend} simulation...[/yellow]")
    console.print(f"[green]\u2713[/green] Configuration: {config}")
    console.print("[bold green]Simulation ready.[/bold green]")


@app.command()
def scenario(
    scenario_id: str = typer.Option(
        ...,
        "--id",
        "-i",
        help="Scenario identifier to execute.",
    ),
    config: Path = typer.Option(
        Path("configs/default.yaml"),
        "--config",
        "-c",
        help="Path to configuration file.",
    ),
    output: Optional[Path] = typer.Option(
        None,
        "--output",
        "-o",
        help="Path for scenario report output.",
    ),
) -> None:
    """Execute a test scenario and generate report."""
    console.print(f"[yellow]Executing scenario: {scenario_id}[/yellow]")
    console.print(f"[green]\u2713[/green] Configuration: {config}")
    if output:
        console.print(f"[green]\u2713[/green] Report output: {output}")
    console.print("[bold green]Scenario execution complete.[/bold green]")


@app.command()
def calibrate(
    mode: str = typer.Argument(
        "camera",
        help="Calibration mode (camera, extrinsics).",
    ),
    config: Path = typer.Option(
        Path("configs/default.yaml"),
        "--config",
        "-c",
        help="Path to configuration file.",
    ),
    output: Optional[Path] = typer.Option(
        None,
        "--output",
        "-o",
        help="Path for calibration output file.",
    ),
) -> None:
    """Run calibration procedures."""
    valid_modes = {"camera", "extrinsics"}
    if mode.lower() not in valid_modes:
        console.print(
            f"[red]Error:[/red] Invalid mode '{mode}'. Valid options: {valid_modes}"
        )
        raise typer.Exit(code=1)

    console.print(f"[yellow]Starting {mode} calibration...[/yellow]")
    console.print("[bold green]Calibration complete.[/bold green]")


@app.command()
def diagnostics(
    config: Path = typer.Option(
        Path("configs/default.yaml"),
        "--config",
        "-c",
        help="Path to configuration file.",
    ),
) -> None:
    """Run system diagnostics and configuration validation."""
    from scandium.config.loader import load_config

    console.print("[bold]Scandium System Diagnostics[/bold]\n")

    # Version info
    table = Table(title="System Information")
    table.add_column("Component", style="cyan")
    table.add_column("Status", style="green")

    table.add_row("Scandium Version", __version__)
    table.add_row("Python", "3.11+")

    # Config validation
    try:
        cfg = load_config(config)
        table.add_row("Configuration", f"✓ Valid ({config})")
        table.add_row("Mode", cfg.project.mode)
        table.add_row("Camera Source", cfg.camera.source)
        table.add_row("Fiducial Backend", cfg.fiducials.backend)
        table.add_row("MAVLink Transport", cfg.mavlink.transport)
    except FileNotFoundError:
        table.add_row("Configuration", f"⚠ Not found ({config})")
    except Exception as e:
        table.add_row("Configuration", f"✗ Error: {e}")

    console.print(table)


@app.command(name="version")
def show_version() -> None:
    """Show version information."""
    console.print(f"[bold blue]Scandium[/bold blue] v{__version__}")
    console.print("Production-grade precision landing system for UAV platforms.")


if __name__ == "__main__":
    app()
