from machine import UART
import machine
from network import WLAN
import os
import time
from machine import WDT

uart = UART(0, 115200)
os.dupterm(uart)

rtc = machine.RTC()
wdt = WDT(timeout=50000)

wlan = WLAN() # get current object, without changing the mode
wlan.deinit()
wdt.feed()
