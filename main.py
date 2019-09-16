"""
CORDOVA-ET logger code compatible with the LoPy 4
The looger reads
* Ambient and object temperature on the MLX90614
* Air temperature and humidity on the SHT3x
* First DS18X20 temperature sensor on the OneWire channels
* 6 spectral channels of the AS726X

Frequencies can be configured in the config file

"""

__author__ = 'Jose A. Jimenez-Berni'
__version__ = '0.1.3'
__license__ = 'MIT'

from network import LoRa
from machine import I2C, RTC, Pin
from sht30 import SHT30
from onewire import DS18X20
from onewire import OneWire
from mlx90614 import MLX90614
from AS726X import AS726X
import socket
import binascii
import struct
import time
import machine
import pycom
import config
import json

rtc = RTC()

MAX_JOIN_RETRY = 100  # Max number of LoRa join before going to deep sleep

print("CORDOVA-ET Node v{version}".format(version=__version__))
# Save battery by disabling the LED
pycom.heartbeat(False)


# Define sleep time in seconds. Default is 1min, modify by downlink message.
# This needs to be read from file since data is lost between reboots
my_config_dict = {'sleep_time': 300, 'lora_ok': False, 'version': __version__}

def save_config(my_config_dict):
    with open("/flash/my_config.json", 'w') as conf_file:
        conf_file.write(json.dumps(my_config_dict))


def load_config(my_config_dict):
    try:
        with open("/flash/my_config.json", 'r') as conf_file:
            my_config_dict =  json.loads(conf_file.read())
    except Exception as ex:
        print("Config file not found")
        with open("/flash/my_config.json", 'w') as conf_file:
            conf_file.write(json.dumps(my_config_dict))

    return my_config_dict

# Give some time for degubbing
time.sleep(2.5)

# If we have a new software version, we reset the settings

config_dict = load_config(my_config_dict)
if 'version' not in config_dict:
    print("New version found...")
    save_config(my_config_dict)
elif config_dict['version'] != __version__:
    print("New version found...")
    save_config(my_config_dict)
else:
    my_config_dict = config_dict


wake_s = machine.wake_reason()

print(wake_s)

# Initialize LoRa in LORAWAN mode.
lora = LoRa(mode=LoRa.LORAWAN)
# create an OTA authentication params for this node
dev_eui = binascii.unhexlify(config.DEV_EUI.replace(' ',''))
app_eui = binascii.unhexlify(config.APP_EUI.replace(' ',''))
app_key = binascii.unhexlify(config.APP_KEY.replace(' ',''))


if wake_s[0] == machine.PIN_WAKE:
    print("Pin wake up")
    lora.nvram_restore()
elif wake_s[0] == machine.RTC_WAKE:
    print("Timer wake up")
    lora.nvram_restore()
    if not lora.has_joined():
        lora.join(activation=LoRa.OTAA, auth=(dev_eui, app_eui, app_key), timeout=0)
elif my_config_dict['lora_ok']:
    print("Have joined before, no need to rejoin")
    lora.nvram_restore()

else:  # deepsleep.POWER_ON_WAKE:
    print("Power ON reset")
    print("Joining LoRa after power ON...")
    # join a network using OTAA
    lora.join(activation=LoRa.OTAA, auth=(dev_eui, app_eui, app_key), timeout=0)
    lora.nvram_save()
    # Allow some time for flashing
    time.sleep(5.0)
join_retry = 0
# wait until the module has joined the network
while not lora.has_joined():
    pycom.rgbled(0x101000) # now make the LED light up yellow in colour
    time.sleep(5.0)
    print('Not joined yet... ', rtc.now())
    wdt.feed()
    join_retry+=1
    if join_retry > MAX_JOIN_RETRY:
        print("Couldn join LoRa, I'm going to sleep...")

        machine.deepsleep(60*1000)  # go to sleep for 1 minute

print("Connected to LoRa")
# Select the right frequencies in the config file

if config.nano_gateway==True:
    frequencies=config.frequency
    print("Setting up LoRa channels to {}MHz".format(frequencies[0]/1e6))
    lora.add_channel(0, frequency=frequencies[0], dr_min=0, dr_max=5)
    lora.add_channel(1, frequency=frequencies[1], dr_min=0, dr_max=5)
    lora.add_channel(2, frequency=frequencies[2], dr_min=0, dr_max=5)
    lora.add_channel(3, frequency=frequencies[3], dr_min=0, dr_max=5)
    lora.add_channel(4, frequency=frequencies[4], dr_min=0, dr_max=5)
    lora.add_channel(5, frequency=frequencies[5], dr_min=0, dr_max=5)
    lora.add_channel(6, frequency=frequencies[6], dr_min=0, dr_max=5)
    lora.add_channel(7, frequency=frequencies[7], dr_min=0, dr_max=5)

pycom.rgbled(0x000000) # now turn the LED off
wdt.feed()
lora.nvram_save()
my_config_dict['lora_ok'] = True
save_config(my_config_dict)

# Data structure is: Ts, To, Tair, RH, Patm, Tsoil, {ch0},{ch1},{ch2},{ch3},{ch4},{ch5}, Tsensor, Volt
float_values = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]

print("Waking up I2C sensors...")
# Init sensor on pins
i2c_irt = I2C(0, I2C.MASTER, pins=('P22', 'P21'))
i2c_sht = I2C(1, I2C.MASTER, pins=('P20', 'P19'))
ow = OneWire(Pin('P23'))

### IRT Sensor
print("Waking IRT...")
irt = None
try:
    irt = MLX90614(i2c_irt, 90)
    time.sleep(1)
    float_values[0] = irt.read_ambient_temp()
    float_values[1] = irt.read_object_temp()
except Exception as error:
    pycom.rgbled(0xff0000) # now make the LED light up red in colour
    print(error)
    time.sleep(5.0)  # Wait 5 senconds with the red LED
    pycom.rgbled(0x000000) # now make the LED light up red in colour
    irt = None
    print("Couldn't find IRT")
wdt.feed()
### Air sensor
print("Waking SHT30...")
sht30 = None
try:
    sht30 = SHT30(i2c_sht)
    time.sleep(1)
    float_values[2],float_values[3] = sht30.measure()
    float_values[4] = 0.0

except Exception as error:
    pycom.rgbled(0xff0000) # now make the LED light up red in colour
    print(error)
    time.sleep(5.0)  # Wait 5 senconds with the red LED
    pycom.rgbled(0x000000) # now make the LED light up red in colour

    print("Couldn't find SHT30")
wdt.feed()
### Soil sensor
print("Waking OWD...")
temp = None
try:
    if len(ow.scan()) > 0:
        print("Devices P11: {}".format(ow.scan()))
        temp = DS18X20(ow)
        temp.start_convertion()
        time.sleep(1)
        float_values[5] = temp.read_temp_async()
        #time.sleep(1)
except Exception as error:
    print(error)
    pycom.rgbled(0xff0000) # now make the LED light up red in colour
    time.sleep(5.0)  # Wait 5 senconds with the red LED
    pycom.rgbled(0x000000) # now make the LED light up red in colour

    print("Couldn't find OWD")
    float_values[5] = -100.0
wdt.feed()
### Pyranometer
print("Waking up pyranometer...")
try:
    sensor = AS726X(i2c=i2c_sht, gain=2)
    sensor_type = sensor.get_sensor_type()
    time.sleep(1)
    print('Ready to read on wavelengths:')
    print(sensor.get_wavelengths())
    sensor.take_measurements()
    float_values[6:12] = sensor.get_calibrated_values()
    float_values[12] = sensor.get_temperature()

except Exception as error:
    print(error)
    pycom.rgbled(0xff0000) # now make the LED light up red in colour
    print(error)
    time.sleep(5.0)  # Wait 5 senconds with the red LED
    pycom.rgbled(0x000000) # now make the LED light up red in colour
    print("Couldn find pyranometer")
    pass
wdt.feed()

print("Setting up battery sensing...")
# Battery sensing
adc = machine.ADC(0)
batt = adc.channel(pin='P16', attn=3)

float_values[13] = (batt.value()/4096.0)*354.8/31.6
print(float_values[13])

# create a LoRa socket
s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

# set the LoRaWAN data rate
s.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)

# make the socket blocking
s.setblocking(False)

print("Joined to LoRa and socket created")
pycom.rgbled(0x001000) # now make the LED light up green in colour

#time.sleep(5.0)

# Payload is sent as byte array with 14*float32 (4 bytes each)
# Total payload size is 56 bytes
# Data structure is: Ts, To, Tair, RH, Patm, {ch0},{ch1},{ch2},{ch3},{ch4},{ch5}, Tsensor, Tsoil, Volt

msg = bytearray(56)

while True:
    try:
        msg = bytearray(struct.pack('14f', *float_values))
        s.send(msg)
        print(float_values)
    except Exception as error:
        print(error)
        pass

    wdt.feed()
    time.sleep(4)
    rx = s.recv(256)
    if rx:
        print("Got a packet from the cloud")
        print(rx)
        in_msg = bytearray(rx)
        if len(in_msg) > 2:
            if in_msg[0] == 1:
                # Sleep time command
                my_config_dict["sleep_time"] = int.from_bytes(in_msg[1:3], 'little')
                print("New sleep time {}s".format(my_config_dict["sleep_time"]))
        save_config(my_config_dict)

    #break
    print("I'm going to sleep...")
    lora.nvram_save()
    pycom.rgbled(0x000000) # now make the LED light up green in colour
    machine.deepsleep(my_config_dict["sleep_time"]*1000)  # go to sleep for 5 minutes
