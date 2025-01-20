"""
Application settings and configuration.
"""
from pathlib import Path
import pyprojroot
from pydantic_settings import BaseSettings

class PathSettings:
    """Path configuration for the application."""
    BASE: Path = pyprojroot.find_root(pyprojroot.has_dir("config"))
    CONFIG_PATH: Path = BASE / "config"
    
class RadarSettings(BaseSettings):
    """Radar device settings."""
    CLI_PORT: str = 'COM6'
    DATA_PORT: str = 'COM7'
    CLI_BAUDRATE: int = 115200
    DATA_BAUDRATE: int = 921600
    CONFIG_FILE: Path = PathSettings.CONFIG_PATH / "mmw_PC_14m.cfg"
    OUTPUT_PATH: Path = Path.home() / "Desktop" / "Output.csv"

settings = RadarSettings()