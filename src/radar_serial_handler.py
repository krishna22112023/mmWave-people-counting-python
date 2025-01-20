from serial import Serial
import time
from pathlib import Path

class RadarSerialHandler:
    """Handle serial communication with radar device."""
    
    def __init__(self, cli_port, data_port):
        """Initialize serial ports."""
        self.cli_port = cli_port
        self.data_port = data_port

    @classmethod
    def configure_ports(cls, config_file_path: Path, settings):
        """Configure and initialize serial ports."""
        cli_port = Serial(settings.CLI_PORT, settings.CLI_BAUDRATE)
        data_port = Serial(settings.DATA_PORT, settings.DATA_BAUDRATE)
        
        # Read and send configuration
        config = cls._read_config(config_file_path)
        cls._send_config(cli_port, config)
        
        return cls(cli_port, data_port)

    @staticmethod
    def _read_config(config_file_path: Path):
        """Read radar configuration file."""
        with open(config_file_path, 'r') as f:
            return [line.rstrip('\r\n') for line in f]

    @staticmethod
    def _send_config(port, config):
        """Send configuration to radar."""
        for cmd in config:
            port.write((cmd + '\n').encode())
            print(cmd)
            time.sleep(0.01)