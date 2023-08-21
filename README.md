# LTE GPS tracker 

## **English | [French](README_fr.md)**

This project aims to give to the end user a gps tracker for one of his asset ( car, motor home, bag ).

This project is based on [ LilyGO T A7670X repositery]( https://github.com/Xinyuan-LilyGO/LilyGO-T-A7670X ). This board feature an esp32, a 1S lipo manager and a SimCom A7670X modem. ( with or without gps).

## Dependancy instalation (Arduino Setup)

You will end to install the following Arduino librairies before flashing the software : 

1. Install the current upstream Arduino IDE at the 1.8 level or later. The current version is at the [Arduino website](http://www.arduino.cc/en/main/software).
2. Start Arduino and open Preferences window. In additional board manager add url: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json .separating them with commas.
3. Select Tools -> Board Management -> Search for ESP32 and install
4. Select Tools -> Board -> ESP32 Wrover Modelu Module
5. Need to install the following dependencies (Unzip and copy to the '~Arduino/libraries' directory)

   - [TinyGSM](https://github.com/vshymanskyy/TinyGSM)
   - [StreamDebugger](https://github.com/vshymanskyy/StreamDebugger)
   - [ArduinoHttpClient](https://github.com/ricemices/ArduinoHttpClient)

## Usage 
You only need to send an SMS to the number attached to the sim card in order to get the position of the tracker. A first message will be sent back in order to confirm the reception of the request and a second one will be sent when gps is available.

## Feature 

## Futur goal

I aim to turn this project into a tiny global asset tracker  using internet request to track multiple asset realtime
