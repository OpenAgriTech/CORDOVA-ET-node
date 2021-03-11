"""
CORDOVA-ET logger code compatible with the LoPy 4
The looger reads:
* Ambient and object temperature on the MLX90614
* Air temperature and humidity on the SHT3x
* First DS18X20 temperature sensor on the OneWire channels
* 6 spectral channels of the AS726X

This version doesn't include frequencies and the defafult region is defined
when programming the firmware on the Lopy4. Therefore this version is not
compatible with single-frequency gateways (NanoGateway)

"""

__author__ = 'Jose A. Jimenez-Berni'
__version__ = '0.2.3'
__license__ = 'MIT'

from network import LoRa
from machine import I2C, RTC, Pin
import os
from OTA import WiFiOTA
from sht30 import SHT30
from BME280 import BME280, BME280_I2CADDR
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
import status

from config import WIFI_SSID, WIFI_PW, SERVER_IP

DEBUG_MODE = True

status_flag=0x0

rtc = RTC()

MAX_JOIN_RETRY = 100  # Max number of LoRa join before going to deep sleep

print("CORDOVA-ET Node v{version}".format(version=__version__))
print(os.uname().release)
# Save battery by disabling the LED
pycom.heartbeat(False)


# Define sleep time in seconds. Default is 1min, modify by downlink message.
# This needs to be read from file since data is lost between reboots
my_config_dict = {'sleep_time': 300, 'lora_ok': False, 'version': __version__,
                  'air_sensor': config.SHT3x_single}

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

def green_blink(time_ms):
    pycom.rgbled(0x001000) # now make the LED light up green in colour
    time.sleep_ms(time_ms)
    pycom.rgbled(0x000000) # turn off LED

def red_blink(time_ms):
    pycom.rgbled(0xaa0000) # now make the LED light up red in colour
    time.sleep_ms(time_ms)
    pycom.rgbled(0x000000) # turn off LED

def rgb_blink(time_ms, color=0x101010):
    pycom.rgbled(color) # now make the LED light up red in colour
    time.sleep_ms(time_ms)
    pycom.rgbled(0x000000) # turn off LED

def scan_i2c(i2c_bus):
    sensors=i2c_bus.scan()
    return sensors


# Give some time for degubbing
time.sleep(2.5)

# Setup OTA
ota = WiFiOTA(WIFI_SSID,
              WIFI_PW,
              SERVER_IP,  # Update server address
              8000)  # Update server port


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
reset_s = machine.reset_cause()

if reset_s==machine.WDT_RESET:
    status_flag|=status.WDT_ERROR

print(wake_s)
print(reset_s)

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
        lora.join(activation=LoRa.OTAA, auth=(dev_eui, app_eui, app_key), timeout=0, dr= 0)
elif my_config_dict['lora_ok']:
    print("Have joined before, no need to rejoin")
    lora.nvram_restore()

else:  # deepsleep.POWER_ON_WAKE:
    print("Power ON reset")
    print("Joining LoRa after power ON...")
    # join a network using OTAA
    lora.join(activation=LoRa.OTAA, auth=(dev_eui, app_eui, app_key), timeout=0)

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
i2c_air = I2C(1, I2C.MASTER, pins=('P20', 'P19'))
ow = OneWire(Pin('P23'))

if DEBUG_MODE:
    debug_info = {'i2c_irt': scan_i2c(i2c_irt), 'i2c_air': scan_i2c(i2c_air)}
    print(debug_info)

### IRT Sensor
irt = None
try:
    irt = MLX90614(i2c_irt, 90)
    print("Waking IRT...OK")
    time.sleep(1)
    float_values[0] = irt.read_ambient_temp()
    float_values[1] = irt.read_object_temp()
    rgb_blink(100, 0xa013f2)
except Exception as error:
    red_blink(1000)
    irt = None
    print("Couldn't find IRT")
wdt.feed()
### Air sensor
if my_config_dict["air_sensor"] == config.NONE:
    print("Ignoring air sensor...")
    float_values[2] = 0.0
    float_values[3] = 0.0
    float_values[4] = 0.0
elif my_config_dict["air_sensor"] == config.BME280:
    bme = None
    try:
        bme = BME280(address=BME280_I2CADDR, i2c=i2c_air)
        print("Waking BME280...OK")
        time.sleep(1)
        float_values[2] = bme.read_temperature()/100.0
        float_values[3] = bme.read_humidity()/1024.0
        float_values[4] = bme.read_pressure()/256.0/100.0
        rgb_blink(100, 0x0000ff)
    except Exception as error:
        red_blink(1000)
        print("Couldn't find BME")

elif my_config_dict["air_sensor"] == config.SHT3x:
    sht30 = None
    try:
        sht30 = SHT30(i2c_air)
        print("Waking SHT3x...OK")
        time.sleep(1)
        float_values[2],float_values[3] = sht30.measure()
        float_values[4] = 0.0
        green_blink(100)
    except Exception as error:
        red_blink(1000)
        print("Couldn't find SHT30")

elif my_config_dict["air_sensor"] == config.SHT3x_single:
    sht30 = None
    try:
        sht30 = SHT30(i2c_air, i2c_address=0x44)
        print("Waking SHT3x...OK")
        time.sleep(1)
        float_values[2],float_values[3] = sht30.measure()
        float_values[4] = 0.0
        green_blink(100)
    except Exception as error:
        rgb_blink(100, 0x13f2ab)
        print("Couldn't find SHT30")

else:
    print("Sensor not supported")

wdt.feed()
### Soil sensor
temp = None
try:
    if len(ow.scan()) > 0:
        print("Waking OWD...OK")
        print("Devices P11: {}".format(ow.scan()))
        temp = DS18X20(ow)
        temp.start_convertion()
        time.sleep(1)
        float_values[5] = temp.read_temp_async()
        if float_values[5] is None:
            float_values[5] = -100
        rgb_blink(100, 0xf2b313)
    else:
        print("Soil temp not found")
        red_blink(1000)
        #time.sleep(1)
except Exception as error:
    print(error)
    red_blink(1000)

    print("Couldn't find OWD")
    float_values[5] = -100.0
wdt.feed()
### Pyranometer (only if not using SHT3x)
if my_config_dict["air_sensor"] != config.SHT3x_single:
    print("Waking up pyranometer...")
    try:
        sensor = AS726X(i2c=i2c_air, gain=2)
        sensor_type = sensor.get_sensor_type()
        time.sleep(1)
        print('Ready to read on wavelengths:')
        print(sensor.get_wavelengths())
        sensor.take_measurements()
        float_values[6:12] = sensor.get_calibrated_values()
        float_values[12] = sensor.get_temperature()
        green_blink(100)
    except Exception as error:
        red_blink(1000)
        print("Couldn find pyranometer")
        pass
    wdt.feed()

# Battery sensing
adc = machine.ADC(0)
batt = adc.channel(pin='P16', attn=3)

float_values[13] = (batt.value()/4096.0)*354.8/31.6
print("Battery: {}V".format(float_values[13]))

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

# Downlink messages:
# * 1: Define duty cycle in seconds:  [01 LSB MSB]
# * 2: Define air sensor type: [02 XX] See XX values in config.py
# * 3: Send firmware version on port 1: [03]
# * 4: Send I2C bus scan on port 1: [04]
# * 5: Perform OTA update: [05 02 03]

msg = bytearray(56)

while True:
    try:
        print(float_values)
        msg = bytearray(struct.pack('14f', *float_values))
        s.send(msg)
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
            elif in_msg[0] == 5:
                if in_msg == bytes([0x05, 0x02, 0x03]):
                    print("Performing OTA!")
                    pycom.rgbled(0x000011)
                    # Perform OTA
                    try:
                        ota.connect()
                        ota.update()
                    except Exception as ex:
                        print(ex)
        elif len(in_msg) == 2:
            if in_msg[0] == 2:
                # Select sensor type
                my_config_dict["air_sensor"] = in_msg[1]
                print("New sensor type: {}".format(my_config_dict["air_sensor"]))
        elif len(in_msg) == 1:
            if in_msg[0] == 3:
                # Send version
                print("Send version name...")
                s.bind(1)
                s.send(__version__ + " " + os.uname().release)
                time.sleep(4)
            elif in_msg[0] == 4:
                # Send I2C scan
                print("Send I2C scan")
                s.bind(1)
                debug_info = {'irt': scan_i2c(i2c_irt), 'air': scan_i2c(i2c_air)}
                s.send("{}".format(debug_info))
                time.sleep(4)
        save_config(my_config_dict)
    lora.nvram_save()
    time.sleep(2)
    print("I'm going to sleep...")
    pycom.rgbled(0x000000) # now make the LED light up green in colour
    machine.deepsleep(my_config_dict["sleep_time"]*1000)  # go to sleep for 5 minutes
