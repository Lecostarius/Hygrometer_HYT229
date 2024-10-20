# Hygrometer_HYT229
Build a hygrometer with a HYT229 sensor.

First shot uses a ESP32 (ESP32 Thing) board. This works, and the form factor is acceptable (57x26mm).
The power consumption if powered via USB is 40 mA at 5 V with the OLED just showing the bare
minimum of information.

The main disadvantage of the ESP32 is how to battery-power it. I can not use a LiPoly battery
since voltage is too high, so I could use 2S with LiPoly and the linear regulator on the ESP32 Thing,
or I use LiFePO4 battery which seems to be ideally suited for the purpose - but I have zero experience
with those, and no charger.

My OLED is too big, and the ESP32 Thing board could also be smaller. A better choice could be a 
Raspberry Pi Pico (there is a Pico2 now, but it seems they are just looking at performance and do not
care about power consumption), since the Pico can accept 1.8 to 5.5V DC input which allows me to use
any battery and the form factor is slightly smaller, 21x51 mm.
