"""
Config file. Edit these keys and replace with the values from TTN
New OTA parameters
"""

# OTAA authentication params
APP_EUI = "70B3D57ED001C537"

# OTA Firmware Update parameters
WIFI_SSID = "MeteoNet"
WIFI_PW = "ChangeMe"
SERVER_IP = "161.111.157.183"

# Supported air temperature and humidity sensors
NONE = const(0)
BME280 = const(1)
SHT3x = const(2)
SHT3x_single = const(3)
