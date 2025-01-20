# src/radar/data_handler.py
"""
Data handling and saving functionality for radar data.
"""
from pathlib import Path
import pandas as pd
import numpy as np
from pyqtgraph.Qt import QtGui
from datetime import datetime

class RadarDataHandler:
    """Handle radar data processing and saving."""
    
    def __init__(self, save_path: Path, plotter, data_processor):
        """
        Initialize the data handler.
        
        Args:
            save_path (Path): Path to save the output CSV
            plotter: RadarPlotter instance for visualization
            data_processor: RadarDataProcessor instance
        """
        self.save_path = save_path
        self.plotter = plotter
        self.data_processor = data_processor
        self.target_obj = {}
        self.point_obj = {}
        self.frame_data = {}
        self.current_index = 0

    def update(self, data_port, config_params):
        """
        Update radar data processing and visualization.
        
        Args:
            data_port: Serial port for data
            config_params (dict): Configuration parameters
            
        Returns:
            bool: True if data processing was successful
        """
        data_ok = False
        target_detected = False
        x = []
        y = []

        # Read and parse the received data
        (data_ok, target_detected, frame_number, 
         out, self.target_obj, self.point_obj) = self.data_processor.process_frame(
            data_port, config_params
        )

        if target_detected:
            # Save data to CSV
            self._save_data(out)
            
            # Update plot with target positions
            x = -self.target_obj["posX"]
            y = self.target_obj["posY"]
            self.plotter.update_plot(x, y)
            QtGui.QApplication.processEvents()

        if data_ok:
            # Calculate point positions
            x = -self.point_obj["range"] * np.sin(self.point_obj["azimuth"])
            y = self.point_obj["range"] * np.cos(self.point_obj["azimuth"])
            
            # Update visualization if needed
            # self.plotter.update_points(x, y)  # Commented as in original
            QtGui.QApplication.processEvents()

            # Store frame data
            self.frame_data[self.current_index] = self.target_obj
            self.current_index += 1

        return data_ok

    def _save_data(self, data_frame):
        """
        Save processed data to CSV.
        
        Args:
            data_frame (pd.DataFrame): Data to save
        """
        data_frame.to_csv(self.save_path, mode='a', index=False)