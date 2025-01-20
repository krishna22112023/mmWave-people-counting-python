# People counting using milli-meter wave radar

## Overview
This project implements a radar data processing system using the AWR1642 device. It captures and processes radar data in real-time, providing visualization and tracking capabilities. The code is based on the python script from the people counting demo [here](https://github.com/ibaiGorordo/AWR1642-Read-Data-Python-MMWAVE-SDK-2/blob/master/People%20counting%20demo/peopleCountingDemo.py)

## Features
- Real-time radar data processing
- 2D scatter plot visualization
- People counting (entry/exit tracking)
- Serial communication with radar device

## Installation

### Using pip
```bash
pip install -r dependencies/requirements.txt
```

### Using Conda
```bash
conda env create -f dependencies/environment.yaml
```

## Configuration
1. Update COM port settings in `config/settings.py`
2. Radar configuration parameters are in `config/mmw_PC_14m.cfg`

## Guide
For a detailed step-by-step guide on how to setup your AWR1642 for people counting, refer to the pdf in doc/

## Usage
Run the main script:
```bash
python scripts/run_radar.py
```