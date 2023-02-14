# barry.mcmullin@dcu.ie
# Jan 2023

import time

print("Initialising...")
led_red = LED(1)
led_green = LED(2)
led_blue = LED(3)
led_yellow = LED(4)

print("entering loop...")
cycle = 0
while (True):
    print("Cycle: {:06d}".format(cycle))

    led_blue.on()
    time.sleep_ms(1000)
    led_blue.off()

    led_red.on()
    time.sleep_ms(1000)
    led_red.off()

    led_green.on()
    time.sleep_ms(1000)
    led_green.off()

    led_yellow.on()
    time.sleep_ms(1000)
    led_yellow.off()

    time.sleep_ms(1000)
    cycle = (cycle + 1)

