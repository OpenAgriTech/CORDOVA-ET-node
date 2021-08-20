"""
CORDOVA-ET logger code compatible with the LoPy 4
The looger reads:
* Ambient and object temperature on the MLX90614
* Air temperature and humidity on the SHT3x
* First DS18X20 temperature sensor on the OneWire channels

This version doesn't include frequencies and the defafult region is defined
when programming the firmware on the Lopy4. Therefore this version is not
compatible with single-frequency gateways (NanoGateway)

"""

__author__ = 'Jose A. Jimenez-Berni'
__version__ = '0.4.1'
__license__ = 'MIT'

import network
from network import LoRa, Sigfox, WLAN
from machine import I2C, RTC, Pin, SD
import os
from OTA import WiFiOTA
from sht30 import SHT30
from BME280 import BME280, BME280_I2CADDR
from onewire import DS18X20
from onewire import OneWire
from mlx90614 import MLX90614
from MCP342x import MCP342x
import socket
import ubinascii
import struct
import time
import machine
import pycom
import config
import json
import status
import sys

from config import WIFI_SSID, WIFI_PW, SERVER_IP, NODE_VERSION, DEBUG_MODE

PAYLOAD_VERSION = 0x01
# Supported air temperature and humidity sensors
NONE = const(0)
BME280 = const(1)
SHT3x = const(2)
SHT3x_single = const(3)

status_flag=0x0

rtc = RTC()

MAX_JOIN_RETRY = 100  # Max number of LoRa join before going to deep sleep

print("CORDOVA-ET Node v{version}".format(version=__version__))
print(os.uname().release)
t = rtc.now()
ts = '{:04d}/{:02d}/{:02d} {:02d}:{:02d}:{:02d}'.format(t[0], t[1], t[2], t[3], t[4], t[5])
print("Current time: ", ts)
# Save battery by disabling the LED
pycom.heartbeat(False)
sigfox = Sigfox()
# Initialize LoRa in LORAWAN mode.
lora = LoRa(mode=LoRa.LORAWAN, rx_iq=True)

# Define sleep time in seconds. Default is 1min, modify by downlink message.
# This needs to be read from file since data is lost between reboots
factory_config_dict = {'sleep_time': 300, 'lora_ok': False, 'version': __version__,
                  'air_sensor': SHT3x_single, 'node_version': NODE_VERSION,
                  'sync_counter': 0, 'sync_timestamp': 0,
                  'dev_eui': ubinascii.hexlify(lora.mac()).decode('ascii'),
                  'app_eui': config.APP_EUI,
                  'app_key': ubinascii.hexlify(ubinascii.unhexlify((ubinascii.hexlify(sigfox.mac())+"FFFE"+ubinascii.hexlify(machine.unique_id()).decode('ascii')))).decode('ascii'),
                  }

# Data structure is: Ts, To, Tair, RH, Patm, Tsoil, Volt
# For pyranometer board, in mV: channel_1, channel_2, channel_3, channel_4, channel_1*5.0, 0.0, Volt

float_values = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]

def save_config(my_config_dict):
    with open("/flash/my_config.json", 'w') as conf_file:
        conf_file.write(json.dumps(my_config_dict))


def load_config():
    try:
        with open("/flash/my_config.json", 'r') as conf_file:
            my_config_dict =  json.loads(conf_file.read())
        missing_keys=False
        for key in factory_config_dict.keys():
            if key not in my_config_dict:
                my_config_dict[key] = factory_config_dict[key]
                missing_keys = True
        if missing_keys:
            save_config(my_config_dict)
    except Exception as ex:
        print("Config file not found")
        with open("/flash/my_config.json", 'w') as conf_file:
            conf_file.write(json.dumps(factory_config_dict))
            my_config_dict = factory_config_dict

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

def send_timesync():
    print("Timesync")
    try:
        if my_config_dict['lora_ok']:
            lora.nvram_restore()
            stats = lora.stats()
            s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
            s.setsockopt(socket.SOL_LORA, socket.SO_DR, 0)
            s.bind(1)

            t = rtc.now()
            my_config_dict['sync_counter'] = stats.tx_counter
            my_config_dict['sync_timestamp'] = time.time()
            msg = bytearray(struct.pack('2L', my_config_dict['sync_counter'], my_config_dict['sync_timestamp']))
            s.setblocking(True)
            s.send(msg)
            lora.nvram_save()
            print("Done!")
        else:
            print("Join LoRaWAN before time sync")
    except Exception as ex:
        print("Error doing time sync: ", ex)

def log_to_SD():
    t = rtc.now()
    ts = '{:04d}/{:02d}/{:02d} {:02d}:{:02d}:{:02d}'.format(t[0], t[1], t[2], t[3], t[4], t[5])
    stats = lora.stats()
    print(stats)
    try:
        sd = SD()
        os.mount(sd, '/sd')
        print("Logging to: {}".format('/sd/{eui}.csv'.format(eui=my_config_dict['dev_eui'][-8:])))
        with open('/sd/{eui}.csv'.format(eui=my_config_dict['dev_eui'][-8:]), 'a') as output:
            output.write(ts+",{counter},".format(counter=stats.tx_counter)+",".join(str(x) for x in float_values)+"\n")
    except Exception as ex:
        print("Error writing to SD: ", ex)

def do_measurements():

    if my_config_dict['node_version']==0x02:
        try:
            addr68_ch0 = MCP342x(i2c_irt, 0x68, channel=0, resolution=18, gain=8,
                                 scale_factor=1000.0)
            addr68_ch1 = MCP342x(i2c_irt, 0x68, channel=1, resolution=18, gain=8,
                                 scale_factor=1000.0)
            addr68_ch2 = MCP342x(i2c_irt, 0x68, channel=2, resolution=18, gain=8,
                                 scale_factor=1000.0)
            addr68_ch3 = MCP342x(i2c_irt, 0x68, channel=3, resolution=18, gain=8,
                                 scale_factor=1000.0)
            float_values[0] = addr68_ch0.convert_and_read()
            float_values[1] = addr68_ch1.convert_and_read()
            float_values[2] = addr68_ch2.convert_and_read()
            float_values[3] = addr68_ch3.convert_and_read()
            float_values[4] = float_values[0] * 5.0
            float_values[5] = 0.0
        except Exception as ex:
            rgb_blink(100, 0x13f2ab)
            print("Couldn't find ADC")
            print(ex)


    else:
        print("Waking up I2C sensors...")
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
        if my_config_dict["air_sensor"] == NONE:
            print("Ignoring air sensor...")
            float_values[2] = 0.0
            float_values[3] = 0.0
            float_values[4] = 0.0
        elif my_config_dict["air_sensor"] == BME280:
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

        elif my_config_dict["air_sensor"] == SHT3x:
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

        elif my_config_dict["air_sensor"] == SHT3x_single:
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

    # Battery sensing
    adc = machine.ADC(0)
    batt = adc.channel(pin='P16', attn=3)

    float_values[6] = (batt.value()/4096.0)*354.8/31.6
    print("Battery: {}V".format(float_values[6]))

# Give some time for degubbing
time.sleep(2.5)

# Setup OTA
ota = WiFiOTA(WIFI_SSID,
              WIFI_PW,
              SERVER_IP,  # Update server address
              8000)  # Update server port




p_in = Pin('P18', mode=Pin.IN, pull=Pin.PULL_UP)

# If we have a new software version, we reset the settings

config_dict = load_config()
if 'version' not in config_dict:
    print("New version found...")
    save_config(factory_config_dict)
elif config_dict['version'] != __version__:
    print("New version found...")
    #save_config(factory_config_dict)

my_config_dict = load_config()


# Init sensor on pins according to nde version
if my_config_dict['node_version']==0x01:
    i2c_irt = I2C(0, I2C.MASTER, pins=('P22', 'P21'))
    i2c_air = I2C(1, I2C.MASTER, pins=('P20', 'P19'))
    ow = OneWire(Pin('P23'))
elif (my_config_dict['node_version']==0x03) or (my_config_dict['node_version']==0x02):
    i2c_irt = I2C(0, I2C.MASTER, pins=('P21', 'P22'))
    i2c_air = I2C(1, I2C.MASTER, pins=('P19', 'P20'))
    ow = OneWire(Pin('P11'))

if DEBUG_MODE:
    debug_info = {'i2c_irt': scan_i2c(i2c_irt), 'i2c_air': scan_i2c(i2c_air)}
    print(debug_info)


wake_s = machine.wake_reason()
reset_s = machine.reset_cause()

if reset_s==machine.WDT_RESET:
    status_flag|=status.WDT_ERROR

print(wake_s)
print()

# create an OTA authentication params for this node
dev_eui = ubinascii.unhexlify(my_config_dict['dev_eui'].replace(' ',''))
app_eui = ubinascii.unhexlify(my_config_dict['app_eui'].replace(' ',''))
app_key = ubinascii.unhexlify(my_config_dict['app_key'].replace(' ',''))

print("LoRaWAN information for registering with TTN:")
print("============================================")
print("DevEUI: %s" % (ubinascii.hexlify(lora.mac()).decode('ascii')))
print("AppEUI: %s" % (ubinascii.hexlify(app_eui).decode('ascii')))
print("AppKey: %s" % (ubinascii.hexlify(app_key).decode('ascii')))
print()

if p_in()==0:
    print("CORDOVA-ET Node v{version} Type: {node_type}".format(version=__version__, node_type=my_config_dict['node_version']))

    print("Entering Config Mode. Commands (case sensitive):")
    print("m: test all measurements")
    print("g: clear config")
    print("r: hard reset (looses time)")
    print("o: perform OTA update")
    print("l: reset LoRaWAN session")
    print("t: print current time")
    print("TYYYY/MM/DD HH:MM:SS set current time (UTC). E.g. T20210801 10:05:00")
    print("VXX: set the node version, where XX is the version (01,02,03). E.g. V02 to set to pyranometer")
    print("s: send timesync message")
    print("CXXXX: Set the measurement cycle in seconds. E.g. C{cycle}".format(cycle=my_config_dict['sleep_time']))
    print()
    print("Don't forget the press enter after the command")
    pycom.rgbled(0x000010) # now make the LED light up blue in colour
    wlan = WLAN(mode=WLAN.AP, ssid="CET-"+my_config_dict['dev_eui'][-4:])
    server = network.Server()
    server.deinit() # disable the server
    # enable the server again with new settings
    tmp_str = ""
    in_key = None
    server.init(login=('uco', 'ias_csic'), timeout=600)
    while p_in()==0:
        wdt.feed()
        char = uart.read(1)
        if char is not None:
            if char == b'\r':
                in_key = tmp_str
                tmp_str = ""
            else:
              tmp_str += char.decode("utf-8")
              print(char.decode("utf-8"), end='')
        if in_key is not None:
            print()
        else:
            continue
        if in_key == 'm':
            do_measurements()
            print(float_values)
        if in_key == 'g':
            print("Clear configuration")
            config_dict = factory_config_dict
            save_config(config_dict)
            wdt.init(0)
            sys.exit()
        elif in_key == 'r':
            print("Hard Reset")
            machine.reset()
        elif in_key == 'l':
            print("Reset LoRaWAN")
            my_config_dict['lora_ok'] = False
            save_config(my_config_dict)
            wdt.init(0)
            sys.exit()
        elif in_key == 't':
            t = rtc.now()
            ts = '{:04d}/{:02d}/{:02d} {:02d}:{:02d}:{:02d}'.format(t[0], t[1], t[2], t[3], t[4], t[5])
            print("Current time: ", ts)
        elif in_key.startswith('T'):
            if len(in_key) != 20:
                print("Wrong format, it should be: TYYYY/MM/DD HH:MM:SS")
            else:
                try:
                    rtc.init((int(in_key[1:5]), int(in_key[6:8]), int(in_key[9:11]), int(in_key[12:14]), int(in_key[15:17]), int(in_key[18:19])))
                    t = rtc.now()
                    ts = '{:04d}/{:02d}/{:02d} {:02d}:{:02d}:{:02d}'.format(t[0], t[1], t[2], t[3], t[4], t[5])
                    print("Current time: ", ts)
                except Exception as ex:
                    print("Error parsing. Format should be: TYYYY/MM/DD HH:MM:SS")
        elif in_key.startswith('C'):
            if len(in_key) < 3:
                print("Wrong format, it should be: CY")
            else:
                try:
                    sleep_time = int(in_key[1:])
                    if sleep_time < 60:
                        print("Minimum cycle time is 60s. Try again")
                    else:
                        my_config_dict['sleep_time'] = sleep_time
                        print("New cycle time set to: {sleep_time} seconds.".format(sleep_time=sleep_time))
                        save_config(my_config_dict)

                except Exception as ex:
                    print("Wrong format, it should be: VXX, where XX is the version (01,02,03)")
        elif in_key == 's':
            print("Send time sync")
            send_timesync()

        elif in_key.startswith('V'):
            if len(in_key) != 3:
                print("Wrong format, it should be: VXX, where XX is the version (01,02,03)")
            else:
                try:
                    my_config_dict['node_version'] = int(in_key[1:3])
                    save_config(my_config_dict)
                    print("Set node type to: {node_version}".format(node_version=int(in_key[1:3])))
                    time.sleep(1)
                    wdt.init(0)
                    sys.exit()
                except Exception as ex:
                    print("Wrong format, it should be: VXX, where XX is the version (01,02,03)")
        elif in_key == 'o':
            print("Performing OTA")
            try:
                ota.connect()
                ota.update()
            except Exception as ex:
                print(ex)
        in_key = None
        time.sleep(0.1)
    server.deinit()
    wlan.deinit()
    pycom.rgbled(0x00000)
    print("Exit Config Mode")

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

# Check for time every 24h
if (time.time() - my_config_dict['sync_timestamp']) > 4*3600:
    print("Time diff: ", time.time() , my_config_dict['sync_timestamp'], time.time() - my_config_dict['sync_timestamp'])
    send_timesync()
elif time.time() < 1e3:
    print("Current time: ", time.time())
    send_timesync()

do_measurements()

try:
    # create a LoRa socket
    s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)

    # set the LoRaWAN data rate
    s.setsockopt(socket.SOL_LORA, socket.SO_DR, 0)

    # make the socket blocking
    s.setblocking(False)

    print("Joined to LoRa and socket created")
    pycom.rgbled(0x001000) # now make the LED light up green in colour
except Exception as ex:
    print("Error creating socket, reset LoRaWAN parameters", ex)
    my_config_dict['lora_ok'] = False


#time.sleep(5.0)

# Payload is sent as byte array with 4 bytes + 7*float32 (4 bytes each)
# Total payload size is 32 bytes
# Data structure is: NODE_VERSION, PAYLOAD_VERSION, 2 BYTES, Ts, To, Tair, RH, Patm, Tsoil, Volt

# Downlink messages:
# * 1: Define duty cycle in seconds:  [01 LSB MSB]
# * 2: Define air sensor type: [02 XX] See XX values in config.py
# * 3: Send firmware version on port 1: [03]
# * 4: Send I2C bus scan on port 1: [04]
# * 5: Perform OTA update: [05 02 03]
# * 6: Timesync message

msg = bytearray(32)

while True:
    try:
        print(float_values)
        msg = bytearray(struct.pack('2B7f', my_config_dict['node_version'], PAYLOAD_VERSION, *float_values))
        s.setblocking(True)
        s.send(msg)
        s.setblocking(False)
    except Exception as error:
        print("Error sending packet: ", error)
        pass
    log_to_SD()
    wdt.feed()
    #time.sleep(4)
    try:
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
                            pycom.rgbled(0x110000)
                elif in_msg[0] == 6:
                    if len(in_msg) == 9:
                        print("Got time sync!")
                        ts_count, ts_offset = struct.unpack("Ii", bytearray(in_msg[1:]))
                        if ts_count == my_config_dict['sync_counter']:
                            rtc.init(time.gmtime(time.time()+ts_offset))
                            t = rtc.now()
                            print("New time is: ", rtc.now() , time.time(), ts_offset)
                            my_config_dict['sync_timestamp'] = time.time()
                            save_config(my_config_dict)
                        else:
                            print("Out of sync message, got {ts_count} should be {counter}".format(ts_count=ts_count, counter=my_config_dict['sync_counter']))

            elif len(in_msg) == 2:
                if in_msg[0] == 2:
                    # Select node type
                    my_config_dict['node_version'] = in_msg[1]
                    print("New sensor type: {}".format(my_config_dict['node_version']))
            elif len(in_msg) == 2:
                if in_msg[0] == 7:
                    # Select air sensor type
                    my_config_dict['air_sensor'] = in_msg[1]
                    print("New sensor type: {}".format(my_config_dict['air_sensor']))
            elif len(in_msg) == 1:
                if in_msg[0] == 3:
                    # Send version
                    print("Send version name...")
                    s.setblocking(True)
                    s.bind(1)
                    s.send(__version__ + " " + os.uname().release)
                elif in_msg[0] == 4:
                    # Send I2C scan
                    print("Send I2C scan")
                    s.setblocking(True)
                    s.bind(1)
                    debug_info = {'irt': scan_i2c(i2c_irt), 'air': scan_i2c(i2c_air)}
                    s.send("{}".format(debug_info))
            save_config(my_config_dict)
    except Exception as ex:
        print("Error receiving packet", ex)
    lora.nvram_save()
    print("NV Save")
    time.sleep(4)
    print("I'm going to sleep...")
    pycom.rgbled(0x000000) # now make the LED light up green in colour
    machine.deepsleep(my_config_dict["sleep_time"]*1000)  # go to sleep for 5 minutes
