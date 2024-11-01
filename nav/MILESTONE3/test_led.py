from led import LED
import time
# LED("green", "on")
led = LED()

print("stuff")
led.toggle("all", "off")
while(True):
    led.toggle("red", "on")
    time.sleep(0.5)
#     led.toggle("red", "off")
#     time.sleep(0.5)
    led.toggle("yellow", "on")
    time.sleep(0.5)
#     led.toggle("yellow", "off")
#     time.sleep(0.5)
    led.toggle("green", "on")
    time.sleep(0.5)

#     led.toggle("blue", "on")
#     time.sleep(0.5)
#     led.toggle("BLUE", "off")
#     time.sleep(1)
