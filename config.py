"""
Config file.
Edit these parameters according to your node hardware and settings

*** VERY IMPORTANT ***

==>> MAKE SURE TO CHANGE THE WIFI PASSWORD ACCORDINGLY <<==

*****************************
"""
# OTA Firmware Update parameters
WIFI_SSID = "MeteoNet"
WIFI_PW = "ChangeMe"
SERVER_IP = "161.111.157.183"

# Supported node versions
# 0x01: Original boards without SD card (green and black)
# 0x02: Pyranometer version of the node using boards v3.x
# 0x03: Boards version v3.x with SD card (black)

NODE_VERSION = 0x03

#Enable debug mode for extra information on serial terminal
DEBUG_MODE = True

# OTAA authentication params
APP_EUI = "70B3D57ED001C537"
