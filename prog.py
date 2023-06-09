import smbus
import time

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
    elif 0 < lux <= 10:
        return "Dark"
    elif 10 < lux <= 15:
        return "Medium"
    elif 15 < lux <= 20:
        return "Bright"
    elif 20 < lux:
        return "Too bright"

######

bus = smbus.SMBus(1)
lightSensor = LightSensor(bus, lightSensorAddress, LightSensor.Mode.CONT_HRES)

######

try:
    while True:
        
        reading = lightSensor.readLux()
        print(luxToCategory(reading))
        
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
