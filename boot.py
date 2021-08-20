from machine import UART
import machine
from network import WLAN
import os
import time
from machine import WDT

uart = UART(0, 115200)
os.dupterm(uart)

rtc = machine.RTC()


wlan = WLAN() # get current object, without changing the mode
wlan.deinit()

wdt = WDT(timeout=50000)
wdt.feed()
