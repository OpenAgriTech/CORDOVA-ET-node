"""
Config file. Edit these keys and replace with the values from TTN
"""

import frequencies

# OTAA authentication params
DEV_EUI = "XX"
APP_EUI = "XX"
APP_KEY = "XXXX"

# Select the radio frequecies. See frequencies.py
# Options are: frequencies_europe, frequencies_nano_europe, frequencies_australia, frequencies_nano_australia
frequency = frequencies.frequencies_nano_europe

# If we are using nano_gateway, we override settings from LoRaWAN server
nano_gateway = True
