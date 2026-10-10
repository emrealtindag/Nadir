#!/usr/bin/env python3
"""
Utility script to extract and convert an MP4 flight recording segment into an optimized GIF.

Usage:
    poetry run python scripts/create_demo_gif.py --input path/to/recording.mp4 --output benchmarks/abort_demo.gif --start 30 --duration 12
"""

import argparse
from pathlib import Path
import cv2
from PIL import Image


def convert_video_to_gif(
    input_path: str,
    output_path: str,
    start_sec: float = 0.0,
    duration_sec: float = 10.0,
    target_width: int = 700,
    fps: int = 15,
) -> None:
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Could not open input video: {input_path}")

    native_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    start_frame = int(start_sec * native_fps)
    max_frames = int(duration_sec * native_fps)
    frame_interval = max(1, int(native_fps / fps))

    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    frames = []
    read_count = 0
    saved_count = 0

    print(f"Extracting segment from {start_sec}s for {duration_sec}s at ~{fps} FPS...")

    while True:
        ret, frame = cap.read()
        if not ret or read_count >= max_frames:
            break

        if read_count % frame_interval == 0:
            h, w = frame.shape[:2]
            scale = target_width / float(w)
            target_h = int(h * scale)
            resized = cv2.resize(frame, (target_width, target_h), interpolation=cv2.INTER_AREA)
            rgb_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb_frame)
            frames.append(pil_img)
            saved_count += 1

        read_count += 1

    cap.release()

    if not frames:
        raise ValueError("No frames were extracted from the given time window.")

    out_file = Path(output_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)

    print(f"Saving {saved_count} frames to {out_file}...")
    duration_ms = int(1000 / fps)
    frames[0].save(
        out_file,
        save_all=True,
        append_images=frames[1:],
        optimize=True,
        duration=duration_ms,
        loop=0,
    )
    print(f"Successfully generated GIF: {out_file} ({out_file.stat().st_size / (1024*1024):.2f} MB)")


def main():
    parser = argparse.ArgumentParser(description="Convert video segment to GIF")
    parser.add_argument("--input", "-i", required=True, help="Input video file path (mp4/mkv/avi)")
    parser.add_argument("--output", "-o", default="benchmarks/abort_demo.gif", help="Output GIF path")
    parser.add_argument("--start", "-s", type=float, default=0.0, help="Start time in seconds")
    parser.add_argument("--duration", "-d", type=float, default=10.0, help="Duration in seconds")
    parser.add_argument("--width", "-w", type=int, default=700, help="Output width in pixels")
    parser.add_argument("--fps", type=int, default=15, help="Target GIF frame rate")

    args = parser.parse_args()
    convert_video_to_gif(
        input_path=args.input,
        output_path=args.output,
        start_sec=args.start,
        duration_sec=args.duration,
        target_width=args.width,
        fps=args.fps,
    )


if __name__ == "__main__":
    main()
