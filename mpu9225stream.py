"""Library to stream from a MPU9225 board."""
from pyb import I2C
import pyb
import utime
import sys
import array
from micropython import const

# Set the full-scale range of the accelerometer in g's here. Must be 2, 4, 8, or 16
FULL_SCALE_CHOICE = 2

# I2C address
AX_ADDR = const(104)

# Translates the full-scale value in g's to the right register value
FULL_SCALE = {
    2: 0 << 3,
    4: 1 << 3,
    8: 2 << 3,
    16: 3 << 3
}

# Maximum 16-bit value
TOP_16BIT = const(65536)

# MPU9225 register addresses
INT_STATUS   = const(0x3A)
ACCEL_XOUT_H = const(0x3B)
ACCEL_XOUT_L = const(0x3C)
ACCEL_YOUT_H = const(0x3D)
ACCEL_YOUT_L = const(0x3E)
ACCEL_ZOUT_H = const(0x3F)
ACCEL_ZOUT_L = const(0x40)
SMPLRT_DIV = const(0x19)
WHO_AM_I = const(0x75)
PWR_MGMT_1 = const(0x6b)
PWR_MGMT_1_CLKSEL_MASK = const(0x7)
PWR_MGMT_1_SLEEP_MASK = const(0x40)
ACCEL_CONFIG = const(0x1c)
ACCEL_CONFIG2 = const(0x1d)
ACCEL_FS_SEL_MASK = const(0x18)

_i2c_object = None
_i2c_default_bus = 1
def i2c(bus_no: int=_i2c_default_bus, baudrate: int=400000) -> I2C:
    """Return an I2C object which is initialized the first time the function is called."""
    global _i2c_object
    if _i2c_object is None:
        _i2c_object = I2C(bus_no, I2C.MASTER, baudrate=baudrate)
    return _i2c_object

def set_i2c_bus(bus_no: int) -> None:
    """Sets the I2C bus used by the accelerometer."""
    global _i2c_default_bus
    _i2c_default_bus = bus_no

def twos_complement(val: int, num_bits: int) -> int:
    """Returns the num_bits-bit two's complement of the input value."""
    mask = 2 ** (num_bits - 1)
    twos_comp = -(val & mask) + (val & ~mask)
    return twos_comp

def ax_send(data: int, max_attempts: int=10) -> None:
    """Send data to the accelerometer, trying up to max_attempts times with exponential backoff. Raises OSError if it fails."""
    attempts = 0
    while attempts < max_attempts:
        try:
            i2c().send(data, addr=AX_ADDR)
            return
        except OSError:
            pyb.delay(0.5 * 2 ** attempts)
            attempts += 1
    raise OSError("Failed to send")

def ax_write(reg: int, value: int) -> None:
    """Write a value to a register."""
    ax_send(bytearray([reg, value]))

def ax_write_masked(reg: int, value: int, bitmask: int, read_after: bool=False) -> int or None:
    """Update some bits (specified by the bitmask) of the register with the bits in the value. If read_after is True, returns the actual value of the register write."""
    masked_val = value & bitmask

    old_val = ax_read(reg, convert=True)
    reg_val = (old_val & ~bitmask) | masked_val
    
    ax_write(reg, reg_val)
    return ax_read(reg, convert=True) if read_after else None
    
def ax_read(reg: int, convert: bool=False) -> int:
    """Read an 8-bit register and return the result as an integer."""
    ax_send(reg)
    if convert:
        return int.from_bytes(i2c().recv(1, addr=AX_ADDR), 'big')
    else:
        return i2c().recv(1, addr=AX_ADDR)

def ax_read_double(addr_h: int, addr_l: int, as_list: bool=False) -> list or int:
    """Read two 8-bit registers. If as_list is True, the result is returned as a list. Otherwise, the result is interpreted as a single 16-bit value."""
    res_h = ax_read(addr_h, convert=True)
    res_l = ax_read(addr_l, convert=True)
    if as_list:
        return [res_h, res_l]
    else:
        return res_h * 256 + res_l

def ax_x() -> int:
    """Read the acceleration value along the x axis."""
    return twos_complement(ax_read_double(ACCEL_XOUT_H, ACCEL_XOUT_L), 16) * FULL_SCALE_CHOICE // 4

def ax_y() -> int:
    """Read the acceleration value along the y axis."""
    return twos_complement(ax_read_double(ACCEL_YOUT_H, ACCEL_YOUT_L), 16) * FULL_SCALE_CHOICE // 4

def ax_z() -> int:
    """Read the acceleration value along the z axis."""
    return twos_complement(ax_read_double(ACCEL_ZOUT_H, ACCEL_ZOUT_L), 16) * FULL_SCALE_CHOICE // 4

def init_accelerometer(do_scan=True) -> None:
    """Initialize the accelerometer."""
    # Wait for an I2C device with the correct I2C address to appear.
    while True:
        check_ready = True
        if do_scan:
            slaves = i2c().scan()
            print("I2C device addresses: " + ", ".join([str(slave) for slave in slaves]))
            if not AX_ADDR in slaves:
                check_ready = False
        if check_ready:
            if (i2c().is_ready(AX_ADDR)):
                print("Ready!")
                break
            else:
                print("AX is not ready.")
        pyb.delay(1000)

    # Set accelerometer clock
    ax_write_masked(reg=PWR_MGMT_1, value=1, bitmask=PWR_MGMT_1_CLKSEL_MASK)

    # Set full scale accelerometer range
    ax_write_masked(reg=ACCEL_CONFIG, value=FULL_SCALE[FULL_SCALE_CHOICE], bitmask=ACCEL_FS_SEL_MASK)

    # Disable sleep
    ax_write_masked(reg=PWR_MGMT_1, value=0, bitmask=PWR_MGMT_1_SLEEP_MASK)

def get_data_string() -> str:
    """Get a string with the current time in microseconds and the acceleration along x, y and z."""
    return "{0} {1} {2} {3}".format(utime.ticks_us(), ax_x(), ax_y(), ax_z())

def to_g(ax) -> float:
    """Convert raw value to acceleration in g's."""
    return 2 * FULL_SCALE_CHOICE * ax / TOP_16BIT

def read_buf(number_of_samples, sample_period, prev_t=0) -> array:
    """Read number_of_samples samples spaced (at least) sample_period apart, blocking in the meantime.
    Can be called in a loop by using the second return value as the third argument in the next call."""
    buf = array.array('i')
    for i in range(number_of_samples):
        # Spin in a tight loop until the time is right
        t = utime.ticks_us()
        if utime.ticks_diff(t, prev_t) < sample_period:
            continue
        prev_t = t

        # Add 4 entries to the buffer: time, x, y and z
        buf.append(t)
        buf.append(ax_x())
        buf.append(ax_y())
        buf.append(ax_z())
    return buf, t
