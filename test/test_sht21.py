#! /usr/bin/python
import smbus
import time
bus = smbus.SMBus(16)
address = 0x40

def readSHT21t():
    bus.write_byte(address, 0xf3)
    time.sleep(0.1)
    th = bus.read_byte(address)
    tl = bus.read_byte(address)
    cs = bus.read_byte(address)
#    print(th)
#    print(tl)
#    print(cs)
    t = (th << 8) + tl
    t2 = -46.85 + 175.72 * t / 65536.0
    return t2

def readSHT21rh():
    bus.write_byte(address, 0xf5)
    time.sleep(0.1)
    rh = bus.read_byte(address)
    rl = bus.read_byte(address)
    cs = bus.read_byte(address)
#    print(rh)
#    print(rl)
#    print(cs)
    r = (rh << 8) + rl
    r2 = -6.0 + 125.0 * r / 65536.0
    return r2

t = readSHT21t()
rh = readSHT21rh()

f_t = "{:.1f}".format(t)
f_rh = "{:.1f}".format(rh)

#print(' ' + str(t) + ', ' + str(rh))
now = int( time.time() )
print (str(now) + ' ' + f_t + ' ' + f_rh)
