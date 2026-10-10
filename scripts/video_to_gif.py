import cv2
import os
from PIL import Image

def convert_video_to_gif(
    video_path: str,
    output_path: str,
    start_sec: float = 0.0,
    end_sec: float = 58.5,
    target_duration_sec: float = 12.0,
    fps_out: int = 10,
    width: int = 640,
):
    """
    Convert video to an optimized GIF showing the full autonomous mission:
    Takeoff -> AUTO Waypoint Cruise -> AI Target Lock -> Precision Touchdown.
    
    Args:
        video_path: Source MP4 video file.
        output_path: Target GIF file path.
        start_sec: Start time in seconds.
        end_sec: End time in seconds.
        target_duration_sec: Desired GIF playback duration (e.g., 12 seconds).
        fps_out: GIF playback frames per second.
        width: Output frame width (height scales proportionally).
    """
    if not os.path.exists(video_path):
        print(f"File not found: {video_path}")
        return

    cap = cv2.VideoCapture(video_path)
    video_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    orig_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    height = int(width * (orig_h / orig_w))

    total_time = end_sec - start_sec
    total_output_frames = int(target_duration_sec * fps_out)
    speedup = total_time / target_duration_sec

    print(f"Converting '{video_path}' to GIF:")
    print(f"  Range: {start_sec:.1f}s to {end_sec:.1f}s (Total video duration: {total_time:.1f}s)")
    print(f"  Output duration: {target_duration_sec:.1f}s at {fps_out} FPS ({total_output_frames} frames)")
    print(f"  Effective speedup: {speedup:.1f}x")
    print(f"  Resolution: {width}x{height}")

    frames = []
    for i in range(total_output_frames):
        current_time_sec = start_sec + (i / total_output_frames) * total_time
        target_frame_idx = int(current_time_sec * video_fps)
        cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame_idx)
        ret, frame = cap.read()
        if not ret:
            break
        small = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        frames.append(Image.fromarray(rgb))

    cap.release()

    if frames:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        print("Quantizing palette for optimal size and sharpness...")
        # Quantize to 128 colors for crisp HUD text and minimal file size
        quantized_frames = [f.quantize(colors=128, method=Image.Resampling.LANCZOS) for f in frames]
        frame_duration_ms = int(1000 / fps_out)

        quantized_frames[0].save(
            output_path,
            save_all=True,
            append_images=quantized_frames[1:],
            duration=frame_duration_ms,
            loop=0,
            optimize=True
        )
        size_mb = os.path.getsize(output_path) / (1024 * 1024)
        print(f"[OK] Generated {output_path}: {size_mb:.2f} MB ({len(frames)} frames, {target_duration_sec:.1f}s duration)")

if __name__ == "__main__":
    convert_video_to_gif(
        video_path="donerkanatlihassasinis.mp4",
        output_path="benchmarks/landing_demo.gif",
        start_sec=0.0,
        end_sec=58.5,
        target_duration_sec=12.0,
        fps_out=10,
        width=640
    )
