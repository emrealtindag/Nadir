"""
I/O utilities for Scandium.

Provides file handling, YAML loading/saving, and path utilities.
"""

from pathlib import Path
from typing import Any

import yaml


def ensure_dir(path: Path) -> Path:
    """
    Ensure directory exists, creating if necessary.

    Args:
        path: Directory path.

    Returns:
        The same path for chaining.
    """
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    return path


def load_yaml(path: Path) -> dict[str, Any]:
    """
    Load YAML file.

    Args:
        path: Path to YAML file.

    Returns:
        Parsed YAML content.

    Raises:
        FileNotFoundError: If file doesn't exist.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"YAML file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def save_yaml(data: dict[str, Any], path: Path) -> None:
    """
    Save data to YAML file.

    Args:
        data: Data to save.
        path: Output path.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)


def load_numpy(path: Path) -> "np.ndarray":
    """
    Load numpy array from file.

    Args:
        path: Path to .npy or .npz file.

    Returns:
        Loaded array.
    """
    import numpy as np

    path = Path(path)
    if path.suffix == ".npz":
        with np.load(path) as data:
            # Return first array in archive
            return data[list(data.keys())[0]]
    return np.load(path)


def save_numpy(data: "np.ndarray", path: Path) -> None:
    """
    Save numpy array to file.

    Args:
        data: Array to save.
        path: Output path.
    """
    import numpy as np

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.save(path, data)


def atomic_write(path: Path, content: str) -> None:
    """
    Write content atomically using temp file + rename.

    Args:
        path: Target file path.
        content: Content to write.
    """
    import tempfile
    import os

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    # Write to temp file in same directory
    fd, temp_path = tempfile.mkstemp(dir=path.parent, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(content)
        # Atomic rename
        os.replace(temp_path, path)
    except Exception:
        # Clean up temp file on error
        if os.path.exists(temp_path):
            os.unlink(temp_path)
        raise
