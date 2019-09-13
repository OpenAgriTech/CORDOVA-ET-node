"""
Config file. Edit these keys and replace with the values from TTN
"""

import frequencies

# OTAA authentication params
DEV_EUI = "XX"
APP_EUI = "XX"
APP_KEY = "XXXX"

# Select the radio frequecies. See frequencies.py
frequency = frequencies.frequencies_nano_europe
nano_gateway = True

# Reset settings to delete any LoRaWAN confing and sampling interval
reset_settings = False
