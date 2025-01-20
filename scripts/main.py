#!/usr/bin/env python3
"""
Main script to run the radar processing system.

This script initializes and coordinates all components of the radar system:
- Serial communication with the radar
- Data processing and analysis
- Visualization
- Data saving

Usage:
    python run_radar.py
"""

import sys
from pathlib import Path
import time

# Add project root to Python path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.append(str(PROJECT_ROOT))

from pyqtgraph.Qt import QtGui
from src.radar.serial_handler import RadarSerialHandler
from src.radar.data_processor import RadarDataProcessor
from src.radar.data_handler import RadarDataHandler
from utils.plotting import RadarPlotter
from config.settings import settings


def setup_radar_system():
    """
    Initialize and configure all radar system components.
    
    Returns:
        tuple: Containing all initialized system components
    """
    # Initialize serial communication
    serial_handler = RadarSerialHandler.configure_ports(
        settings.CONFIG_FILE,
        settings
    )
    
    # Initialize data processing components
    data_processor = RadarDataProcessor()
    config_params = data_processor.parse_config_file(settings.CONFIG_FILE)
    
    # Initialize visualization
    app = QtGui.QApplication([])
    plotter = RadarPlotter()
    
    # Initialize data handling
    data_handler = RadarDataHandler(
        save_path=settings.OUTPUT_PATH,
        plotter=plotter,
        data_processor=data_processor
    )
    
    return serial_handler, config_params, app, plotter, data_handler


def cleanup_radar_system(serial_handler, plotter):
    """
    Perform cleanup operations when shutting down the system.
    
    Args:
        serial_handler: Initialized RadarSerialHandler instance
        plotter: Initialized RadarPlotter instance
    """
    # Stop the sensor
    serial_handler.cli_port.write('sensorStop\n'.encode())
    
    # Close serial ports
    serial_handler.cli_port.close()
    serial_handler.data_port.close()
    
    # Close visualization window
    plotter.win.close()


def main():
    """
    Main execution function for the radar system.
    
    This function:
    1. Sets up all system components
    2. Runs the main processing loop
    3. Handles graceful shutdown on keyboard interrupt
    """
    try:
        # Setup system components
        serial_handler, config_params, app, plotter, data_handler = setup_radar_system()
        
        print("Radar system initialized. Press Ctrl+C to stop...")
        
        # Main processing loop
        while True:
            # Update data processing and visualization
            data_ok = data_handler.update(
                serial_handler.data_port,
                config_params
            )
            
            # Maintain 30 Hz sampling rate
            time.sleep(0.033)
            
    except KeyboardInterrupt:
        print("\nShutting down radar system...")
        cleanup_radar_system(serial_handler, plotter)
        print("Shutdown complete.")
        
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        cleanup_radar_system(serial_handler, plotter)
        raise
        

if __name__ == "__main__":
    main()