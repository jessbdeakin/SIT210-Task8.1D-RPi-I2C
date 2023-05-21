import RPi.GPIO as gpio
import smbus
import time

ledPin = 11
lightSensorAddress = 0x23



class LightSensor():
    
    UNIT_FACTOR = 1.0 / 1.2
    
    class Mode():
        CONT_HRES  = 0b00010000
        CONT_HRES2 = 0b00010001
        CONT_LRES  = 0b00010011
        OT_HRES    = 0b00100000
        OT_HRES2   = 0b00100001
        OT_LRES    = 0b00100011
    
    def __init__(self, bus, address, mode = None):
        self.bus = bus
        self.address = address
        self.mode = mode
        
        if mode is not None:
            self.bus.write_byte(self.address, mode)
        
    def readLux(self, mode = None):
        if mode is None:
            mode = self.mode
        data = self.bus.read_i2c_block_data(self.address, mode)
        return ((data[0] << 8) | data[1]) * LightSensor.UNIT_FACTOR


def luxToCategory(lux):
    if lux == 0:
        return "Too dark"
    elif 0 < lux <= 20:
        return "Dark"
    elif 20 < lux <= 50:
        return "Medium"
    elif 50 < lux <= 100:
        return "Bright"
    elif 100 < lux:
        return "Too bright"

######

bus = smbus.SMBus(1)

gpio.setmode(gpio.BOARD)
gpio.setup(ledPin, gpio.OUT)

lightSensor = LightSensor(bus, lightSensorAddress, LightSensor.Mode.CONT_HRES)

######

try:
    while True:
        
        reading = lightSensor.readLux()
        print(luxToCategory(reading))
        
        time.sleep(0.2)
except KeyboardInterrupt:
    pass

gpio.cleanup()
