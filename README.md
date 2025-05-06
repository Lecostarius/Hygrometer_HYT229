# Hygrometer_HYT229
Build a hygrometer with a HYT229 sensor.

First shot uses a ESP32 (ESP32 Thing) board. This works, and the form factor is acceptable (57x26mm).
The power consumption if powered via USB is 40 mA at 5 V with the OLED just showing the bare
minimum of information (not connecting the OLED does not help, so almost all of the power is eaten by
the ESP32 Thing). 

The main disadvantage of the ESP32 is how to battery-power it. I can not use a LiPoly battery
since voltage is too high, so I could use 2S with LiPoly and the linear regulator on the ESP32 Thing,
or I use LiFePO4 battery which seems to be ideally suited for the purpose - but I have zero experience
with those, and no charger.

My OLED is too big, and the ESP32 Thing board could also be smaller. A better choice could be a 
Raspberry Pi Pico (there is a Pico2 now, but it seems they are just looking at performance and do not
care about power consumption), since the Pico can accept 1.8 to 5.5V DC input which allows me to use
any battery and the form factor is slightly smaller, 21x51 mm.

A third option is a traditional Arduino nano, which is by and large the smallest form factor of normal
mC boards. However, this needs 7V input (2 S LiPoly). Current consumption is said to be 20 mA.

After some measurements and experiments, I settled for the Pico2. The reason is that it offers twice the
amount of Flash (4 MB instead of 2 MB) compared to the Pico, and I can create a data logging function more easily with that.
Also, in my measurements, the Pico2 is not more power hungry than the Pico; I can get along with something
in the order of 25 mA current consumption if driven by a single LiPo cell of 4 V, which is really low and
should allow me battery life times of 2-4 days when using a 2000 mAh LiPo.  

I decided to add a BMP280 barometer, it is very cheap and I can add a nice height-measurement and also
traditional barometer function to the device. It also uses I2C.

Wiring is easy. I have three external components to the Pico, all share - except the 3.3V and GND - the same
wires: SDA (I color code it green, Pin 6 of the Pico) and SCL (I color code it yellow, Pin 7 of the Pico).
The three are the 0.91 inch OLED display (I2C adress 0x3C), the HYT221 sensor itself (I2C adress 0x28) and
the BMP280, which has I2C adress 0x76. 

The BMP280 is a cheap breakout, but the manual says that the I2C adress is switchable between 0x76 and 0x77.
However, if I try - and I tried twice with two such boards - the sensor returns only 0.0 after setting the
I2C adress to 0x77 even once - it seems that applying the 3.3V to the SDO pin kills the sensor. It still
responds to I2C protocol though, it is just that it gives no data any more. So, I decided to keep adress 0x76
which is not a problem anyway.

I also purchased a Lipo Shim for the Pico2: https://shop.pimoroni.com/products/pico-lipo-shim, this contains
a charging circuit for 1S LiPo, which is really nice - it allows me to drive the Hygrometer from battery,
or from USB directly, and also allows me to recharge the battery using USB.

With the Pico, it is possible to get the logged data out using the USB connector and USB mass storage protocol
(well, at least, almost). That makes for very easy usage and, along with the capability to run on 1S LiPo, made
the Pico my favorite.

Because of limited space, I plan to use a single button for user interface. Currently, I can only think of one
function for the UI, namely: start logging. But maybe there could be different modes (focus on humidity, focus
on barometric pressure or height...), and also, I would really need a real time clock to make sense of my
logging data, and a clock would need to be set, which requires a UI.

After the first 3d prints, I found an issue with my layout: the Lipo Shim is adding almost a cm to the thickness
of the Pico2, which makes it difficult for me to fit above the battery. And, I put the HYT into a hole in the
wall, where it fits snugly. But that means that the temperature sensor becomes slow - probably because it has
now great thermal contact to the case, and therefore the entire case needs to warm up or cool down before the HYT
will see the actual new temperature value. This is a big problem since the effect of temperature on measured
moisture is very large: even an offset of as little as 0.5 degree might result in a moisture reading that is
several percent off. It might be better to glue the HYT into a large hole with space around it, so that it keeps 
most of its speed. Speed is not the only thing; my electronics eats a bit of power (20 mA at 4 V or 80 mW) which
will cause some self-heating which will make the measured relative moisture lower than reality due to a higher than
actual temperature reading.

The Lipo Shim has a white LED that is always on when the device is on. With my 3d printed PLA, I can see that LED
from the outside, which looks weird. Either I remove the LED, or print with 100 percent material so that this light
is somewhat dampened.

After a lot of trying, I could only switch off the Pico 2 and restart it again after 5 seconds time. On an empty
Pico, that seemed to bring current consumption (on 1S Lipo) to below 1 mA, on my final setup however, which still
has the display on, the LED of the Shim on, the barometer on, current consumption drops to some 7 mA. While the
Pico2 is running to take the next measurement and to update the display, it draws 18 mA; this is already using
a reduced clock of 50 MHz.

I used the SingleFileDrive facility of LittleFS to export the logfile to the outside world. When pushing the button,
logging starts, and in this mode, the device switches itself off and awakes every 5 seconds (still consuming the
abovementioned 7 mA though). The restart process lets the display flicker shortly.

Note: the current code that uses powman and switches off the Pico 2 is in the branch sleep and not in main!

