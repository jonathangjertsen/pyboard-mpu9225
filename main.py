"""Main program, runs on system startup"""
import mpu9225stream

# Initialize an MPU9225 board connected to I2C bus 1
mpu9225stream.set_i2c_bus(1)
mpu9225stream.init_accelerometer(do_scan=True)

# Read data as fast as possible
while True:
    print(mpu9225stream.get_data_string())
