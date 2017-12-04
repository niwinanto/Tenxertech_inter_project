import Adafruit_GPIO as GPIO
import Adafruit_GPIO.Platform as Platform
import Adafruit_GPIO.SPI as SPI
import time
import os
import sys
import math

FIFO_SAMPLES = 0x1F 
PI = 3.14

# Acelerometer sensitivity
SCALE_ACCELEROMETER_FULL = 0.0039 
SCALE_ACCELEROMETER_2 = 0.0039
SCALE_ACCELEROMETER_4 = 0.0078
SCALE_ACCELEROMETER_8 = 0.0156
SCALE_ACCELEROMETER_16 = 0.0312
SPI_MODE = 1

# SDO/ALT ADDRESS pin is pulled HIGH, so our address is : ADXL345_AD = '0x1D'
# Would be 0x53 if SDO/ALT ADDRESS is LOW
ADXL345_AD = '0x1D'
BLAZE_I2C_BUS = '0x01'

# Register map
DEVID = '0x00'
THRESH_TAP = '0x1D'
OFSX = '0x1E'
OFSY = '0x1F'
OFSZ = '0x20'
DUR = '0x21'
LATENT = '0x22'
WINDOW = '0x23'
THRESH_ACT = '0x24'
THRESH_INACT = '0x25'
TIME_INACT = '0x26'
ACT_INACT_CTL = '0x27'
THRESH_FF = '0x28'
TIME_FF = '0x29'
TAP_AXES = '0x2A'
ACT_TAP_STATUS = '0x2B'
BW_RATE = '0x2C'
POWER_CTL = '0x2D'
INT_ENABLE = '0x2E'
INT_MAP = '0x2F'
INT_SOURCE = '0x30'
DATA_FORMAT = '0x31'
DATAX0 = '0x32'
DATAX1 = '0x33'
DATAY0 = '0x34'
DATAY1 = '0x35'
DATAZ0 = '0x36'
DATAZ1 = '0x37'
FIFO_CTL = '0x38'
FIFO_STATUS = '0x39'

# Device ID
ID_DEV = 0xE5
RESET_VAL = 0x00

# ACT_INACT_CTL register bits
ACT_AC_DC = 1 << 7
ACT_X_ENABLE = 1 << 6
ACT_Y_ENABLE = 1 << 5
ACT_Z_ENABLE = 1 << 4
INACT_AC_DC = 1 << 3
INACT_X_ENABLE = 1 << 2
INACT_Y_ENABLE = 1 << 1
INACT_Z_ENABLE = 1 << 0

# TAP_AXES register bits
TAP_SUPPRESS = 1 << 3
TAP_X_ENABLE = 1 << 2
TAP_Y_ENABLE = 1 << 1
TAP_Z_ENABLE = 1 << 0

# BW_RATE register bits
LOW_POWER = 1 << 4
# Set output data rate. Data rate = 2 * BW
#   D3  D2  D1  D0  Bandwidth (Hz)  Idd (uA)
#   1   1   1   1   1600            140
#   1   1   1   0   800             90
#   1   1   0   1   400             140
#   1   1   0   0   200             140
#   1   0   1   1   100             140
#   1   0   1   0   50              140
#   1   0   0   1   25              90
#   1   0   0   0   12.5            60
#   0   1   1   1   6.25            50
#   0   1   1   0   3.13            45
#   0   1   0   1   1.56            40
#   0   1   0   0   0.78            34
#   0   0   1   1   0.39            23
#   0   0   1   0   0.20            23
#   0   0   0   1   0.10            23
#   0   0   0   0   0.05            23
BW_1600 = 0x0F
BW_800 = 0x0E
BW_400 = 0x0D
BW_200 = 0x0C
BW_100 = 0x0B
BW_50 = 0x0A
BW_25 = 0x09
BW_12_5 = 0x08
BW_6_25 = 0x07
BW_3_13 = 0x06
BW_1_56 = 0x05
BW_0_78 = 0x04
BW_0_39 = 0x03
BW_0_20 = 0x02
BW_0_10 = 0x01
BW_0_05 = 0x00

# POWER_CTL register bits
LINK = 1 << 5
AUTO_SLEEP = 1 << 4
MEASURE = 1 << 3
SLEEP = 1 << 2
# Set frequency (Hz) of reading in sleep mode.
#   D1  D0  Frequency (Hz)
#   0   0   8
#   0   1   4
#   1   0   2
#   1   1   1
POWER_CTL_D1 = 1 << 1
POWER_CTL_D0 = 1 << 0
WAKEUP_8 = 0x00
WAKEUP_4 = 0x01
WAKEUP_2 = 0x02
WAKEUP_1 = 0x03

# INT_ENABLE register bits
INT_ENABLE_DATA_READY = 1 << 7
INT_ENABLE_SINGLE_TAP = 1 << 6
INT_ENABLE_DOUBLE_TAP = 1 << 5
INT_ENABLE_ACTIVITY = 1 << 4
INT_ENABLE_INACTIVITY = 1 << 3
INT_ENABLE_FREE_FALL = 1 << 2
INT_ENABLE_WATERMARK = 1 << 1
INT_ENABLE_OVERRUN = 1 << 0

# INT_MAP register bits
INT_MAP_DATA_READY = 1 << 7
INT_MAP_SINGLE_TAP = 1 << 6
INT_MAP_DOUBLE_TAP = 1 << 5
INT_MAP_ACTIVITY = 1 << 4
INT_MAP_INACTIVITY = 1 << 3
INT_MAP_FREE_FALL = 1 << 2
INT_MAP_WATERMARK = 1 << 1
INT_MAP_OVERRUN = 1 << 0
# ADXL345 interrupt pins
ADXL345_INT_PIN1 = 0x00
ADXL345_INT_PIN2 = 0x01

# DATA_FORMAT register bits
SELF_TEST = 1 << 7
SPI_MODE = 1 << 6
INT_INVERT = 1 << 5
FULL_RES = 1 << 3
JUSTIFY = 1 << 2
# Interrupt levels
INT_ACT_HIGH = 0x00
INT_ACT_LOW  = 0x01
# Set range of measurment (g)
#   D1  D0  Range
#   0   0   +/-2 g
#   0   1   +/-4 g
#   1   0   +/-8 g
#   1   1   +/-16 g
RANGE_2G = 0x00
RANGE_4G = 0x01
RANGE_8G = 0x02
RANGE_16G = 0x03

# FIFO_CTL register bits
TRIGGER = 1 << 5
# Set FIFO mode
#   D7  D6  Mode    
#   0   0   Bypass
#   0   1   FIFO
#   1   0   Stream
#   1   1   Trigger
FIFO_BYPASS = 0x00
FIFO_ENABLED = 0x40
FIFO_STREAM = 0x80
FIFO_TRIGGER = 0xC0

# FIFO_STATUS register bit
FIFO_TRIG = 1 << 7

# global variables 
DEBUG = False
aRes = 0
a_zero_z = 0
a_zero_y = 0
a_zero_x = 0
flow_count = 0
readings = []
burst_read = []

def fifo_overflow_handle(channel):
    global flow_count
    disable_interrupts()
    flow_count = flow_count + 1
    enable_fifo_mode(FIFO_BYPASS, FIFO_SAMPLES)
    sys.stdout.write("%d\n" %(flow_count))
    sys.stdout.flush()
    set_interrupt(INT_ENABLE_WATERMARK, 1)
    if (is_interrupt_enable(INT_ENABLE_WATERMARK)):
	print "int enabled\n"
    enable_fifo_mode(FIFO_STREAM, FIFO_SAMPLES)


def toggle_debug():
    """
     Toggle DEBUG global variable
    """
    global DEBUG
    if DEBUG:
        DEBUG = False
        print("debug disabled")
    else:
        DEBUG = True
        print("debug enabled")

def set_bit(val, bit):
    """
     A utility function to set a bit in a byte
    """
    val = val | bit
    return val

def clear_bit(val, bit):
    """
     A utility function to clear a bit in a byte
    """
    val = val & (~bit)
    return val

def blaze_hal_i2c_burst_read(dev_addr, dev_bus, start_reg, reg_count):
    """
     A function call for multibyte read of registers
    """
    global burst_read

    reg = int(start_reg, 16)
    if not SPI_MODE :
        device_addr = int(dev_addr, 16)
        device_bus = int(dev_bus, 16)
        dev = i2c.get_i2c_device(device_addr, device_bus)
        burst_read = dev.readList(reg, reg_count)
    else :
        spi = SPI.SpiDev(2, 0, 5000000)
        spi.set_bit_order(SPI.MSBFIRST)
        spi.set_clock_hz(5000000)
        spi.set_mode(0)
        reg = 0xC0 | reg
        to_send = [reg, 0x00,0x00,0x00,0x00,0x00,0x00,
                0x00,0x00,0x00,0x00,0x00,0x00]
        burst_read = [0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00,
                0x00,0x00,0x00,0x00,0x00,0x00]
        burst_read = spi.transfer(to_send)
        spi.close()
    return

def blaze_hal_read(dev_addr, dev_bus, reg_addr):
    """
     A function to read the content of a single register
    """
    reg = int(reg_addr, 16)
    if SPI_MODE :
        spi = SPI.SpiDev(2, 0, 5000000)
        spi.set_bit_order(SPI.MSBFIRST)
        spi.set_clock_hz(5000000)
        spi.set_mode(0)
        reg = 0x80 | reg
        to_send = [reg, 0x00]
        data = [0x00, 0x00]
        data = spi.transfer(to_send)
        spi.close()
        return data[1]
    else :
        device_addr = int(dev_addr, 16)
        device_bus = int(dev_bus, 16)
        dev = i2c.get_i2c_device(device_addr, device_bus)
        return dev.readU8(reg)
    return 0

def blaze_hal_write(dev_addr, dev_bus, reg_addr, reg_val, overwrite,
        overwrite_mask):
    """
     Write data to a single register
    """
    device_addr = int(dev_addr, 16)
    device_bus = int(dev_bus, 16)
    mask = int(overwrite_mask, 16)
    addr = int(reg_addr, 16)
    val = int(reg_val, 16)
    if not overwrite :
        pre_val = blaze_hal_read(dev_addr, dev_bus, reg_addr)
        pre_val = pre_val & ~mask
        val = pre_val | val

    if SPI_MODE :
        spi = SPI.SpiDev(2, 0, 5000000)
        spi.set_bit_order(SPI.MSBFIRST)
        spi.set_clock_hz(5000000)
        spi.set_mode(0)
        to_send = [addr, val]
        data = [0x00, 0x00]
        data = spi.transfer(to_send)
        spi.close()

    else :
        dev = i2c.get_i2c_device(device_addr, device_bus)
        dev.write8(addr, val)
    return

def device_start():
    """
     Toggle device from standby mode to measure mode. Device is in standby mode
     when power comes up. In order to start reading measure mode should be
     enabled
    """
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, POWER_CTL)
    val = set_bit(val, MEASURE)
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, POWER_CTL, str(hex(val)), 0x01,'0x00')

def device_standby():
    """
     Toggle device from measure mode to standby mode. This is the default mode
     when power comes up. No measurement is done in this mode. Lowest power
     consumption. Contents of FIFO are preserved.
    """
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, POWER_CTL)
    val = clear_bit(val, MEASURE)
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, POWER_CTL, str(hex(val)), 0x01,'0x00')

def is_low_power():
    """
     A function to test wheather low power mode is enabled. Return a positive
     value if low power mode is enabled. Otherwise zero is returned
    """
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, BW_RATE)
    return val & LOW_POWER

def set_power_mode(mode):
    """
     Set the device between low power and normal mode of operation. 
     mode = 1: Low power mode enabled
     mode = 0: Normal mode of operation
    """
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, BW_RATE)
    if (mode):
        val = set_bit(val, LOW_POWER)
    else:
        val = clear_bit(val, LOW_POWER)
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, BW_RATE, str(hex(val)), 0x01,'0x00')

def get_rate():
    """
     Returns the data rate of operation. Data rate = 2 * BW
    """
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, BW_RATE)
    val = val & 0x0F
    return pow(2, int(val, 16) - 6) * 6.25

def set_rate(bandwidth):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, BW_RATE)
    mask = 0xF0
    bandwidth = bandwidth & 0x0F
    val = val & mask
    val = val | bandwidth
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, BW_RATE, str(hex(val)), 0x01,'0x00')

def read_interrupt_source():
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, INT_SOURCE)
    return val

def is_interrupt(interrupt_bit):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, INT_SOURCE)
    return val & interrupt_bit

def get_interrupt_mapping(interrupt_bit):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, INT_MAP)
    return val & interrupt_bit

def set_interrupt_mapping(interrupt_bit, interrupt_pin):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, INT_MAP)
    if (interrupt_pin):
        val = set_bit(val, interrupt_bit)
    else:
        val = clear_bit(val, interrupt_pin)
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, INT_MAP, str(hex(val)), 0x01,'0x00')

def is_interrupt_enable(interrupt_bit):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, INT_ENABLE)
    return val & interrupt_bit

def set_interrupt(interrupt_bit, state):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, INT_ENABLE)
    if (state):
        val = set_bit(val, interrupt_bit)
    else:
        val = clear_bit(val, interrupt_bit)
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, INT_ENABLE, str(hex(val)), 0x01, '0x00')

def disable_interrupts():
    val = 0x00
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, INT_ENABLE, str(hex(val)), 0x01, '0x00')

def get_interrupt_level():
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, DATA_FORMAT)
    return val & INT_INVERT

def set_interrupt_level(level):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, DATA_FORMAT)
    if (level):
        val = set_bit(val, INT_INVERT)
    else:
        val = clear_bit(val, INT_INVERT)
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, DATA_FORMAT, str(hex(val)), 0x01, '0x00')

def get_trigger_map():
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, FIFO_CTL)
    return val & TRIGGER

def set_trigger_map(interrupt_pin):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, FIFO_CTL)
    if (interrupt_pin):
        val = set_bit(val, TRIGGER)
    else:
        val = clear_bit(val, TRIGGER)
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, FIFO_CTL, str(hex(val)), 0x01,
            '0x00')

def is_trigger():
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, FIFO_STATUS)
    return val & FIFO_TRIG

def get_fifo_mode():
    mask = 0xC0
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, FIFO_CTL)
    val = val & mask
    return val

def enable_fifo_mode(mode, samples):
    mode = mode & 0xCF
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, FIFO_CTL)
    val = val & 0x3F
    val = mode | val
    if (samples):
        samples = samples & 0x1F
        val = val & 0xE0
        val = val | samples
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, FIFO_CTL, str(hex(val)), 0x01, '0x00')

def is_full_res():
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, DATA_FORMAT)
    return val & FULL_RES

def set_full_res(mode):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, DATA_FORMAT)
    if (mode):
        val  = set_bit(val, FULL_RES)
    else:
        val = clear_bit(val, FULL_RES)
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, DATA_FORMAT, str(hex(val)), 0x01, '0x00')

def get_range():
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, DATA_FORMAT)
    val = val & 0x03
    return val

def set_range(range_val):
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, DATA_FORMAT)
    range_val = range_val & 0x03
    mask = 0xFC
    val = val & mask
    val = val | range_val
    blaze_hal_write(ADXL345_AD, BLAZE_I2C_BUS, DATA_FORMAT, str(hex(val)), 0x01, '0x00')

def detect_device():
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, DEVID)
    if (val == ID_DEV):
        print "ADXL345 detected\n"
        return 0
    else:
        print "Error Wrong Device\n"
        return 1

def set_accelerometer_sensitivity():
    global aRes
    val = get_range()
    if (is_full_res()):
        aRes = SCALE_ACCELEROMETER_FULL
    elif val == RANGE_2G :
        aRes = SCALE_ACCELEROMETER_2
    elif val == RANGE_4G :
        aRes = SCALE_ACCELEROMETER_4
    elif aRes == RANGE_8G :
        aRes = SCALE_ACCELEROMETER_8
    else :
        aRes = SCALE_ACCELEROMETER_16

def calculate_accelerometer_reading_g(x, y, z):
    global aRes
    global a_zero_x, a_zero_y, a_zero_z
    x = (x - a_zero_x) * aRes
    y = (y - a_zero_y) * aRes
    z = (z - a_zero_z) * aRes
    return [x, y, z]

def samples_collected():
    val = blaze_hal_read(ADXL345_AD, BLAZE_I2C_BUS, FIFO_STATUS)
    val = val & 0x3F
    return val

# The following functions that read data from accelerometer assumes
# the data from the registers are right justified. So the JUSTIFY bit in 
# DATA_FORMAT register should be set to zero (the default value). Using this
# functions with a left justification will cause wrong interpretation.
def read_data(count):
    global readings
    global burst_read

    for i in range(count):
        blaze_hal_i2c_burst_read(ADXL345_AD, BLAZE_I2C_BUS, str(DATAX0), 6)
        index = 0
        if SPI_MODE :
            index = 1
        while index < 6:
            index1 = index + 1
            lsb = burst_read[index]
            msb = burst_read[index1]
            temp = msb << 8 | lsb
            if (temp | 0x8000) :
                temp = temp - 65535
            readings.append(temp)
            index = index + 2

def calibrate():
    print "Keep the sensor on horizontal surface for calibration"
    global readings
    enable_fifo_mode(FIFO_ENABLED, FIFO_SAMPLES)
    while samples_collected() < FIFO_SAMPLES :
        pass

    enable_fifo_mode(FIFO_BYPASS, 0)
    fifo_cnt = FIFO_SAMPLES
    global a_zero_x
    global a_zero_y
    global a_zero_z
    global aRes

    read_data(FIFO_SAMPLES)
    ax = ay = az = 0

    while fifo_cnt :
        a_zero_z = a_zero_z + readings.pop()
        a_zero_z = a_zero_z - 1.0 / aRes
        a_zero_y = a_zero_y + readings.pop()
        a_zero_x = a_zero_x + readings.pop()
        fifo_cnt = fifo_cnt - 1

    a_zero_x = a_zero_x / FIFO_SAMPLES
    a_zero_y = a_zero_y / FIFO_SAMPLES
    a_zero_z = a_zero_z / FIFO_SAMPLES
    print "KNOWN POSITION OF ACCELEROMETER", a_zero_x, a_zero_y, a_zero_z

def read_accelerometer_xyz() :
    global burst_read
    xyz = []

    blaze_hal_i2c_burst_read(ADXL345_AD, BLAZE_I2C_BUS, str(DATAX0), 6)
    index = 0
    if SPI_MODE:
        index = 1
    while index < 6:
        index1 = index + 1
        lsb = burst_read[index]
        msb = burst_read[index1]
        temp = msb << 8 | lsb
        if (temp | 0x8000) :
            temp = temp - 65535
        xyz.append(temp)
        index = index + 2
    return xyz

def pitch_roll(x, y, z):
    try :
        roll = math.atan2(y, z)
        pitch = math.atan2(-x, math.sqrt((y * y) + (z * z)))
        pitch = pitch * 180.0 / PI
        roll = roll * 180.0 / PI

    except ZeroDivisionError :
        print "\n", x, y, z
        return

    sys.stdout.write("\r pitch %f roll %f" %(pitch, roll))
    sys.stdout.flush()

def main():
    print "RUNNING ACCELEROMETER TEST"
    if(detect_device()):
        return
    set_full_res(1)
    set_range(RANGE_8G)
    set_rate(BW_50)
    set_accelerometer_sensitivity()
    device_start()
    calibrate()
    disable_interrupts()
    set_interrupt_level(INT_ACT_LOW)
    gpio = GPIO.get_platform_gpio()

    #checking for the CHIP platform#
    if (5==Platform.platform_detect()):
	print "CHIP platform running\n"
    else:
	print "Not running on CHIP device\n"
	exit()

    fifo_int_pin = "PWM1"
     #set AP-EINT1n as the interrupt pin# 
    gpio.setup(fifo_int_pin,GPIO.IN)
    print "#",gpio.input(fifo_int_pin)
    #add a callback fn() for falling event on fifo_interrupt# 
    gpio.add_event_detect(fifo_int_pin,GPIO.FALLING)
    gpio.add_event_callback(fifo_int_pin,fifo_overflow_handle)
    set_interrupt(INT_ENABLE_WATERMARK, 1)
    if (is_interrupt_enable(INT_ENABLE_WATERMARK)):
		print "int enabled"
    enable_fifo_mode(FIFO_STREAM, FIFO_SAMPLES)
    while 1 :
	pass
		#time.sleep(0.1)
    	#print "#",gpio.input(fifo_int_pin)
		#print samples_collected(),
        #xa, ya, za = read_accelerometer_xyz()
        #sys.stdout.write("\r %d %d %d" %(xa, ya, za))
        #sys.stdout.flush()

if __name__ == "__main__":
    main()
