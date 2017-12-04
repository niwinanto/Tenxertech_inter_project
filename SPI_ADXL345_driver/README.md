
# Python program for testing ADXL345 
ADXL345 is an accelerometer from Analog Devices. This program uses python spidev library and [Adafruit GPIO library for CHIP](https://github.com/xtacocorex/Adafruit_Python_GPIO). It has a collection of functions for modifying the registers in the sensor. Default interface is SPI but I2C is also supported. This code was used to test the SPI interface in CHIP Pro. 

## Connections 
|ADXL345        |  CHIP Pro     |
|---------------|---------------|
|CS(pin 7)      |  PE0(pin 41)  |
|SCLK(pin 14)   |  PE1(pin 40)  |
|SDO(pin 12)    |  PE2(pin 39)  |
|SDI(pin 13)    |  PE3(pin 38)  |  
## ADXL345 Interrupt detection  
We have used Adafruit Python GPIO Library which provide a cross-platform GPIO interface on the Raspberry Pi. Using this library we used one of the GPIO pins in CHIP Pro to detect the edge triggering. We have connected interrupt pin of ADXL345 to this GPIO pin and wrote a handler to do the required actions while changing the ADXL345 interrupt line. The ADXL345 interrupt line is programed to change state when ever FIFO samples overflows. 


