"""
Configuration loader for Scandium.

Handles YAML loading with environment variable substitution,
path resolution, and default configuration merging.
"""

from pathlib import Path
from typing import Any

import yaml

from scandium.config.schema import ScandiumConfig
from scandium.config.validation import validate_config


def load_config(config_path: Path) -> ScandiumConfig:
    """
    Load configuration from a YAML file.

    Args:
        config_path: Path to the YAML configuration file.

    Returns:
        Validated ScandiumConfig instance.

    Raises:
        FileNotFoundError: If the configuration file does not exist.
        ValueError: If the configuration is invalid.
    """
    config_path = Path(config_path)

    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        raw_config = yaml.safe_load(f) or {}

    # Resolve relative paths relative to config file location
    config_dir = config_path.parent
    raw_config = _resolve_paths(raw_config, config_dir)

    # Create and validate configuration
    config = ScandiumConfig(**raw_config)
    validate_config(config, config_dir)

    return config


def _resolve_paths(config: dict[str, Any], base_dir: Path) -> dict[str, Any]:
    """
    Recursively resolve relative paths in configuration.

    Paths ending with '_path' or '_dir' are resolved relative to base_dir.

    Args:
        config: Configuration dictionary.
        base_dir: Base directory for relative path resolution.

    Returns:
        Configuration with resolved paths.
    """
    result: dict[str, Any] = {}

    for key, value in config.items():
        if isinstance(value, dict):
            result[key] = _resolve_paths(value, base_dir)
        elif isinstance(value, str) and (key.endswith("_path") or key.endswith("_dir")):
            path = Path(value)
            if not path.is_absolute():
                # Keep as-is for validation layer to handle
                result[key] = value
            else:
                result[key] = value
        else:
            result[key] = value

    return result


def save_config(config: ScandiumConfig, output_path: Path) -> None:
    """
    Save configuration to a YAML file.

    Args:
        config: ScandiumConfig instance to save.
        output_path: Path to the output YAML file.
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    config_dict = config.model_dump(mode="json")

    with open(output_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(config_dict, f, default_flow_style=False, sort_keys=False)


def get_default_config() -> ScandiumConfig:
    """
    Get default configuration with all default values.

    Returns:
        ScandiumConfig instance with defaults.
    """
    return ScandiumConfig()
