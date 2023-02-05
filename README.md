# ESP32-Serial-Monitor

simple ESP32 monitor using a tft touch screen

Since the beginning of my maker career I struggled with the nonexistent debugging and remote monitoring capabilities of some chips liek the standard arduino.
This device may help you in seeing those serial messages which of course only occur when your circuit is not connected to your computer :)

- tft screen with live config of font, orientation and baud rate
- telnet server to remotely monitor serial output (port 24 on ip address given to the esp32)

important:
- create the secrets file with write_data sketch (see my other repositories) first
- make sure to follow pin mapping when assembling

check out more details in the source code

BOM:
- an esp32 dev board
- a tft lcd screen which is supported by tft_espi library (ili9341, ST7789 etc.)
- lion charging circuit
- step up module to 5v (voltage might be unstable otherwise in battery mode)
- lion battery
- two switches (on/off) one for power on/off the other to toggle pause mode
- a 220uf, 10v cpacitator, close to 5v and ground on the esp32 board to buffer peaks during wlan transmission
- wires
- the 3d printed case - files are in the source directory

