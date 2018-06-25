# pyboard-mpu9225
MPU9225 driver library for MicroPython Pyboard. Works with MPU9225 breakout boards such as the one labelled "MPU-92/65". For now, only the accelerometer functionality is implemented.

## Usage
Drop `mpu9225stream.py` into the PyBoard filesystem. If the provided `main.py` file is used, the pyboard outputs raw data from the accelerometer at maximum speed.
