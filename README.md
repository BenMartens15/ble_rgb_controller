# ESP32 RGB Lights Controller

This firmware allows an ESP32 to control light switches and RGB strips using an Android app.

## Set up
To select what type of light (RGB strip or light switch) the ESP32 can control, open the ESP-IDF menuconfig and under "Lightning Light Controller", select the device type. Then just rebuild the program and flash it.

## Hardware
A light switch controller uses: 
- an active-high PIR motion detection module ([this one](https://www.amazon.ca/gp/product/B07X31RVJY/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1), to be precise)
- a servo
- a mounting fixture (https://www.thingiverse.com/thing:6282545)

<img src="/images/light-switch-front-view.jpg"> <img src="/images/light-switch-side-view.jpg>

For an RGB strip controller... every RGB strip is different, but you will probaly need a few transistors controlled by the ESP32 in order to provide enough power to control the strip.

## Android App
The Android app to control devices can be found here: https://github.com/BenMartens15/SmartLightController